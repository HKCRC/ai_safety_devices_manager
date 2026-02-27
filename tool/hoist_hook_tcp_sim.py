#!/usr/bin/env python3
import argparse
import json
import os
import socket
import struct
import threading
from pathlib import Path


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


def load_hook_config(config_path: str):
    if not config_path:
        return {}
    try:
        with open(config_path, "r", encoding="utf-8") as f:
            root = json.load(f)
        runtime = root.get("runtime", {})
        hook_cfg = runtime.get("hoist_hook", {})
        return hook_cfg if isinstance(hook_cfg, dict) else {}
    except Exception as e:
        print(f"[sim] warning: failed to load config '{config_path}': {e}")
        return {}


class HoistHookState:
    def __init__(self):
        self.lock = threading.Lock()
        self.warning_light = 0  # 0x0001
        self.speaker = 0  # 0x0002 bit0=7m bit1=3m
        self.valid_mask = 0x0003  # 0x0003: group1+group2 valid
        self.uid_base = 0xA1B20000

    def _rfid_group_regs(self):
        regs = []
        for i in range(8):
            uid = (self.uid_base + i) & 0xFFFFFFFF
            uid_hi = (uid >> 16) & 0xFFFF
            uid_lo = uid & 0xFFFF
            # Keep deterministic, fixed values for connectivity testing.
            rssi = max(20, 85 - i * 2)
            batt = max(20, 90 - i * 4)
            rssi_batt = ((rssi & 0xFF) << 8) | (batt & 0xFF)
            regs.extend([uid_hi, uid_lo, rssi_batt])
        return regs

    def read_holding(self, unit_id: int, addr: int, qty: int):
        with self.lock:
            out = []
            if unit_id == self.hook_uid:
                rfid_regs = self._rfid_group_regs()
                for i in range(qty):
                    a = addr + i
                    if a == 0x0001:
                        out.append(self.warning_light)
                    elif a == 0x0002:
                        out.append(self.speaker)
                    elif a == 0x0003:
                        out.append(self.valid_mask)
                    elif 0x0004 <= a <= 0x001B:
                        out.append(rfid_regs[a - 0x0004])
                    else:
                        out.append(0)
            elif unit_id == self.power_uid:
                for i in range(qty):
                    a = addr + i
                    if 0x0064 <= a <= 0x0069:
                        # Example power-status raw words
                        sample = {
                            0x0064: 5240,   # 52.40V (if interpreted with 0.01 scale)
                            0x0065: 112,    # 1.12A
                            0x0066: 5870,   # 58.70% or raw energy
                            0x0067: 0x0001, # status flag
                            0x0068: 318,    # temp/raw
                            0x0069: 0x0000,
                        }
                        out.append(sample.get(a, 0))
                    else:
                        out.append(0)
            else:
                out = [0] * qty
            return out

    def write_single(self, unit_id: int, addr: int, value: int):
        with self.lock:
            if unit_id != self.hook_uid:
                return False
            if addr == 0x0001:
                self.warning_light = 1 if (value & 0x1) else 0
                return True
            if addr == 0x0002:
                self.speaker = value & 0x0003
                return True
            if addr == 0x0003:
                self.valid_mask = value & 0x00FF
                return True
            return False


def handle_request(state: HoistHookState, unit_id: int, pdu: bytes) -> bytes:
    if len(pdu) < 1:
        return pdu_exception(0, 0x03)
    fc = pdu[0]

    if fc == 0x03:
        if len(pdu) != 5:
            return pdu_exception(fc, 0x03)
        addr, qty = struct.unpack(">HH", pdu[1:5])
        if qty <= 0 or qty > 125:
            return pdu_exception(fc, 0x03)
        regs = state.read_holding(unit_id, addr, qty)
        payload = b"".join(struct.pack(">H", r & 0xFFFF) for r in regs)
        return struct.pack(">BB", fc, len(payload)) + payload

    if fc == 0x06:
        if len(pdu) != 5:
            return pdu_exception(fc, 0x03)
        addr, value = struct.unpack(">HH", pdu[1:5])
        ok = state.write_single(unit_id, addr, value)
        if not ok:
            return pdu_exception(fc, 0x02)  # illegal data address
        return pdu

    return pdu_exception(fc, 0x01)


def handle_client(conn: socket.socket, client_addr, state: HoistHookState):
    print(f"[sim] client connected: {client_addr}")
    try:
        while True:
            mbap = recv_exact(conn, 7)
            tx_id, proto_id, length, unit_id = struct.unpack(">HHHB", mbap)
            if proto_id != 0 or length < 2:
                break
            pdu = recv_exact(conn, length - 1)
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
        print(f"[sim] client disconnected: {client_addr}")


def main():
    parser = argparse.ArgumentParser(description="Hoist hook Modbus TCP simulator")
    parser.add_argument("--config", default="", help="path to common_config.json")
    parser.add_argument("--host", default="", help="bind host (default from runtime.hoist_hook.module_ip)")
    parser.add_argument("--port", type=int, default=-1, help="bind port (default from runtime.hoist_hook.module_port)")
    parser.add_argument("--hook-unit-id", type=int, default=-1, help="hook slave id (default from runtime.hoist_hook.hook_slave_id)")
    parser.add_argument("--power-unit-id", type=int, default=-1, help="power slave id (default from runtime.hoist_hook.power_slave_id)")
    args = parser.parse_args()

    cfg_path = resolve_config_path(args.config)
    cfg = load_hook_config(cfg_path)
    host = args.host if args.host else str(cfg.get("module_ip", "0.0.0.0"))
    port = args.port if args.port > 0 else int(cfg.get("module_port", 502))
    hook_uid = args.hook_unit_id if args.hook_unit_id > 0 else int(cfg.get("hook_slave_id", 3))
    power_uid = args.power_unit_id if args.power_unit_id > 0 else int(cfg.get("power_slave_id", 4))

    state = HoistHookState()
    state.hook_uid = hook_uid
    state.power_uid = power_uid

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((host, port))
    sock.listen(8)

    print("[sim] hoist_hook tcp simulator started")
    if cfg_path:
        print(f"[sim] config={cfg_path}")
    else:
        print("[sim] config=not found, use built-in defaults/CLI")
    print(f"[sim] listen={host}:{port} hook_uid={hook_uid} power_uid={power_uid}")
    print("[sim] supported: FC03 read, FC06 write")
    print("[sim] hook regs: 0x0001(light),0x0002(speaker),0x0003(mask),0x0004~0x001B(rfid groups)")
    print("[sim] power regs: 0x0064~0x0069")

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
