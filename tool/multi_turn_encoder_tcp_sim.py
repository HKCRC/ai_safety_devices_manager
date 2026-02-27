#!/usr/bin/env python3
import argparse
import json
import os
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
        state.write_single(addr, value)
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
        state.write_multi(addr, values)
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
    parser = argparse.ArgumentParser(description="Multi-turn encoder Modbus TCP simulator")
    parser.add_argument("--config", default="", help="path to common_config.json")
    parser.add_argument("--host", default="", help="bind host (default from config runtime.multi_turn_encoder.ip)")
    parser.add_argument("--port", type=int, default=-1, help="bind port (default from config runtime.multi_turn_encoder.port)")
    parser.add_argument("--unit-id", type=int, default=-1, help="modbus unit id/slave (default from config runtime.multi_turn_encoder.slave)")
    parser.add_argument("--start-turns", type=float, default=12.0, help="initial turn count")
    parser.add_argument("--speed-rps", type=float, default=0.05, help="turns per second")
    parser.add_argument(
        "--turns-mode",
        choices=["time", "read_rate"],
        default="read_rate",
        help="time=按真实时间连续变化, read_rate=按读取间隔累加变化",
    )
    args = parser.parse_args()

    cfg_path = resolve_config_path(args.config)
    enc_cfg = load_encoder_config(cfg_path)
    transport = str(enc_cfg.get("transport", "rtu")).lower()

    host = args.host if args.host else str(enc_cfg.get("ip", "0.0.0.0"))
    port = args.port if args.port > 0 else int(enc_cfg.get("port", 1502))
    unit_id = args.unit_id if args.unit_id > 0 else int(enc_cfg.get("slave", 1))

    if transport and transport != "tcp":
        print(f"[sim] warning: config transport='{transport}', simulator serves tcp only")

    state = EncoderState(args.start_turns, args.speed_rps, unit_id, args.turns_mode)
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((host, port))
    sock.listen(8)

    print("[sim] multi_turn_encoder tcp simulator started")
    if cfg_path:
        print(f"[sim] config={cfg_path}")
    else:
        print("[sim] config=not found, use built-in defaults/CLI")
    print(
        f"[sim] listen={host}:{port} unit_id={unit_id} "
        f"speed_rps={args.speed_rps} turns_mode={args.turns_mode}"
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


if __name__ == "__main__":
    main()
