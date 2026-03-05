#!/usr/bin/env python3
import argparse
import errno
import json
import os
import pty
import socket
import struct
import threading
import time
from pathlib import Path


class EncoderState:
    def __init__(self, start_turns: float, speed_rps: float, unit_id: int, turns_mode: str):
        self._lock = threading.Lock()
        self.start_time = time.time()
        self.start_turns = start_turns
        self.turns = start_turns
        self.last_update_time = time.time()
        self.turns_mode = turns_mode
        self.speed_rps = speed_rps
        self.unit_id = unit_id
        self.device_addr = unit_id
        self.baud_code = 2  # 9600
        self.direction = 0  # clockwise addition
        self.parity = 1  # no check

    def total_turns(self) -> float:
        with self._lock:
            now = time.time()
            if self.turns_mode == "read_rate":
                dt = max(0.0, now - self.last_update_time)
                self.turns += self.speed_rps * dt
                self.last_update_time = now
                return self.turns
            return self.start_turns + self.speed_rps * (now - self.start_time)

    def set_position(self, position: int) -> None:
        # For compatibility, "position" is interpreted as whole turns.
        with self._lock:
            self.start_turns = float(position)
            self.turns = float(position)
            self.start_time = time.time()
            self.last_update_time = self.start_time

    def read_holding(self, addr: int, qty: int):
        regs = []
        turns = self.total_turns()
        whole = int(turns)
        frac = int((turns - whole) * 8192.0) & 0xFFFF

        for i in range(qty):
            a = addr + i
            if a == 0x0000:
                regs.append((whole >> 16) & 0xFFFF)
            elif a == 0x0001:
                regs.append(whole & 0xFFFF)
            elif a == 0x0002:
                regs.append(whole & 0xFFFF)
            elif a == 0x0003:
                regs.append(frac)
            elif a == 0x0044:
                regs.append(self.device_addr & 0xFFFF)
            elif a == 0x0045:
                regs.append(self.baud_code & 0xFFFF)
            elif a == 0x0046:
                regs.append(self.direction & 0xFFFF)
            elif a == 0x0047:
                regs.append(self.parity & 0xFFFF)
            else:
                regs.append(0)
        return regs

    def write_single(self, addr: int, value: int):
        if addr == 0x0044:
            self.device_addr = value & 0xFF
            return True
        if addr == 0x0045:
            self.baud_code = value & 0xFFFF
            return True
        if addr == 0x0046:
            self.direction = value & 0xFFFF
            return True
        if addr == 0x0047:
            self.parity = value & 0xFFFF
            return True
        return False

    def write_multi(self, addr: int, values):
        if addr == 0x004A and len(values) >= 2:
            pos = ((values[0] & 0xFFFF) << 16) | (values[1] & 0xFFFF)
            # signed int32 decode
            if pos & 0x80000000:
                pos -= 0x100000000
            self.set_position(pos)
            return True
        return False


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def with_crc(payload: bytes) -> bytes:
    crc = crc16_modbus(payload)
    return payload + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def recv_exact(conn: socket.socket, n: int) -> bytes:
    out = bytearray()
    while len(out) < n:
        chunk = conn.recv(n - len(out))
        if not chunk:
            raise ConnectionError("client disconnected")
        out.extend(chunk)
    return bytes(out)


def pdu_exception(fc: int, exc_code: int) -> bytes:
    return struct.pack(">BB", (fc | 0x80) & 0xFF, exc_code & 0xFF)


def handle_request(state: EncoderState, unit_id: int, pdu: bytes) -> bytes:
    if len(pdu) < 1:
        return pdu_exception(0, 0x03)
    fc = pdu[0]

    if fc in (0x03, 0x04):
        if len(pdu) != 5:
            return pdu_exception(fc, 0x03)
        addr, qty = struct.unpack(">HH", pdu[1:5])
        if qty <= 0 or qty > 125:
            return pdu_exception(fc, 0x03)
        regs = state.read_holding(addr, qty)
        payload = b"".join(struct.pack(">H", r) for r in regs)
        return struct.pack(">BB", fc, len(payload)) + payload

    if fc == 0x06:
        if len(pdu) != 5:
            return pdu_exception(fc, 0x03)
        addr, value = struct.unpack(">HH", pdu[1:5])
        ok = state.write_single(addr, value)
        if not ok:
            return pdu_exception(fc, 0x02)
        return pdu  # Echo request

    if fc == 0x10:
        if len(pdu) < 6:
            return pdu_exception(fc, 0x03)
        addr, qty, byte_count = struct.unpack(">HHB", pdu[1:6])
        expected = 6 + byte_count
        if len(pdu) != expected or byte_count != qty * 2:
            return pdu_exception(fc, 0x03)
        values = []
        data = pdu[6:]
        for i in range(qty):
            values.append(struct.unpack(">H", data[i * 2:(i + 1) * 2])[0])
        ok = state.write_multi(addr, values)
        if not ok:
            return pdu_exception(fc, 0x02)
        return struct.pack(">BHH", fc, addr, qty)

    return pdu_exception(fc, 0x01)


def handle_client(conn: socket.socket, addr, state: EncoderState):
    print(f"[sim] client connected: {addr}")
    try:
        while True:
            mbap = recv_exact(conn, 7)
            tx_id, proto_id, length, unit_id = struct.unpack(">HHHB", mbap)
            if proto_id != 0:
                break
            if length < 2:
                break
            pdu_len = length - 1
            pdu = recv_exact(conn, pdu_len)
            if unit_id != state.unit_id:
                resp_pdu = pdu_exception(pdu[0] if pdu else 0, 0x0B)
            else:
                resp_pdu = handle_request(state, unit_id, pdu)
            resp_mbap = struct.pack(">HHHB", tx_id, 0, len(resp_pdu) + 1, unit_id)
            conn.sendall(resp_mbap + resp_pdu)
    except Exception:
        pass
    finally:
        try:
            conn.close()
        except Exception:
            pass
        print(f"[sim] client disconnected: {addr}")


def ensure_device_link(target_dev: str, slave_path: str):
    target = Path(target_dev)
    if target.exists() or target.is_symlink():
        if target.is_symlink():
            try:
                current = os.path.realpath(str(target))
                if current == os.path.realpath(slave_path):
                    return
            except Exception:
                pass
            target.unlink()
        else:
            raise RuntimeError(
                f"device path exists and is not symlink: {target_dev}. "
                "Please remove it or pass another --device path."
            )
    parent = target.parent
    if not parent.exists():
        raise RuntimeError(f"parent dir not exists: {parent}")
    os.symlink(slave_path, target_dev)


def expected_rtu_req_len(buf: bytearray):
    if len(buf) < 2:
        return None
    fc = buf[1]
    if fc in (0x03, 0x04, 0x06):
        return 8
    if fc == 0x10:
        if len(buf) < 7:
            return None
        byte_count = buf[6]
        return 9 + byte_count
    # Unknown FC requests in this project are 8-byte fixed format.
    return 8


def handle_rtu_adu(state: EncoderState, adu: bytes) -> bytes:
    if len(adu) < 8:
        return b""
    uid = adu[0]
    pdu = adu[1:-2]
    recv_crc = adu[-2] | (adu[-1] << 8)
    calc_crc = crc16_modbus(adu[:-2])
    if recv_crc != calc_crc:
        return b""

    fc = pdu[0] if pdu else 0
    if uid != state.unit_id:
        resp_pdu = pdu_exception(fc, 0x0B)
    else:
        resp_pdu = handle_request(state, uid, pdu)
    return with_crc(bytes([uid]) + resp_pdu)


def serve_rtu(state: EncoderState, device: str, verbose: bool):
    master_fd, slave_fd = pty.openpty()
    slave_path = os.ttyname(slave_fd)
    # Keep slave side open to avoid EIO when external peer is not connected yet.
    keep_slave_fd = slave_fd
    ensure_device_link(device, slave_path)

    print("[sim] multi_turn_encoder rtu simulator started")
    print(f"[sim] virtual serial: link {device} -> {slave_path}")
    print(
        f"[sim] unit_id={state.unit_id} speed_rps={state.speed_rps} "
        f"turns_mode={state.turns_mode}"
    )
    print("[sim] supported: FC03/04 read, FC06 single write, FC10 multi-write")

    buf = bytearray()
    try:
        while True:
            try:
                data = os.read(master_fd, 256)
            except OSError as e:
                if e.errno == errno.EIO:
                    time.sleep(0.05)
                    continue
                raise
            if not data:
                time.sleep(0.01)
                continue
            buf.extend(data)
            while True:
                exp_len = expected_rtu_req_len(buf)
                if exp_len is None or len(buf) < exp_len:
                    break
                adu = bytes(buf[:exp_len])
                del buf[:exp_len]
                resp = handle_rtu_adu(state, adu)
                if resp:
                    if verbose:
                        print(f"[sim] rtu rx={adu.hex()} tx={resp.hex()}")
                    os.write(master_fd, resp)
    except KeyboardInterrupt:
        print("\n[sim] stopped")
    finally:
        try:
            os.close(master_fd)
        except Exception:
            pass
        try:
            os.close(keep_slave_fd)
        except Exception:
            pass


def serve_tcp(state: EncoderState, host: str, port: int):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((host, port))
    sock.listen(8)

    print("[sim] multi_turn_encoder tcp simulator started")
    print(
        f"[sim] listen={host}:{port} unit_id={state.unit_id} "
        f"speed_rps={state.speed_rps} turns_mode={state.turns_mode}"
    )
    print("[sim] supported: FC03/04 read, FC06 single write, FC10 multi-write")

    try:
        while True:
            conn, client_addr = sock.accept()
            t = threading.Thread(target=handle_client, args=(conn, client_addr, state), daemon=True)
            t.start()
    except KeyboardInterrupt:
        print("\n[sim] stopped")
    finally:
        sock.close()


def load_encoder_config(config_path: str):
    if not config_path:
        return {}
    try:
        with open(config_path, "r", encoding="utf-8") as f:
            root = json.load(f)
        runtime = root.get("runtime", {})
        enc = runtime.get("multi_turn_encoder", {})
        return enc if isinstance(enc, dict) else {}
    except Exception as e:
        print(f"[sim] warning: failed to load config '{config_path}': {e}")
        return {}


def resolve_config_path(cli_path: str):
    candidates = []
    if cli_path:
        candidates.append(Path(cli_path))
    env_cfg = os.getenv("ASC_CONFIG")
    if env_cfg:
        candidates.append(Path(env_cfg))

    script_root = Path(__file__).resolve().parents[1]
    candidates.extend(
        [
            Path("config/common_config.json"),
            Path("../config/common_config.json"),
            Path("../../config/common_config.json"),
            script_root / "config" / "common_config.json",
        ]
    )

    for c in candidates:
        try:
            if c.exists():
                return str(c.resolve())
        except Exception:
            pass
    return ""


def main():
    parser = argparse.ArgumentParser(description="Multi-turn encoder Modbus simulator (TCP/RTU)")
    parser.add_argument("--config", default="", help="path to common_config.json")
    parser.add_argument(
        "--transport",
        choices=["tcp", "rtu"],
        default="",
        help="override transport mode (default from config runtime.multi_turn_encoder.transport)",
    )
    parser.add_argument("--host", default="", help="bind host (default from config runtime.multi_turn_encoder.ip)")
    parser.add_argument("--port", type=int, default=-1, help="bind port (default from config runtime.multi_turn_encoder.port)")
    parser.add_argument("--device", default="", help="rtu device path (default from config runtime.multi_turn_encoder.device)")
    parser.add_argument("--unit-id", type=int, default=-1, help="modbus unit id/slave (default from config runtime.multi_turn_encoder.slave)")
    parser.add_argument("--start-turns", type=float, default=12.0, help="initial turn count")
    parser.add_argument("--speed-rps", type=float, default=0.05, help="turns per second")
    parser.add_argument("--verbose", action="store_true", help="verbose rtu frame logs")
    parser.add_argument(
        "--turns-mode",
        choices=["time", "read_rate"],
        default="read_rate",
        help="time=按真实时间连续变化, read_rate=按读取间隔累加变化",
    )
    args = parser.parse_args()

    cfg_path = resolve_config_path(args.config)
    enc_cfg = load_encoder_config(cfg_path)
    transport = args.transport if args.transport else str(enc_cfg.get("transport", "rtu")).lower()

    host = args.host if args.host else str(enc_cfg.get("ip", "0.0.0.0"))
    port = args.port if args.port > 0 else int(enc_cfg.get("port", 1502))
    device = args.device if args.device else str(enc_cfg.get("device", "/tmp/ttyENC0"))
    unit_id = args.unit_id if args.unit_id > 0 else int(enc_cfg.get("slave", 1))

    state = EncoderState(args.start_turns, args.speed_rps, unit_id, args.turns_mode)
    if cfg_path:
        print(f"[sim] config={cfg_path}")
    else:
        print("[sim] config=not found, use built-in defaults/CLI")
    if transport == "rtu":
        serve_rtu(state, device, args.verbose)
        return
    serve_tcp(state, host, port)


if __name__ == "__main__":
    main()
