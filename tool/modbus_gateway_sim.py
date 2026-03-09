#!/usr/bin/env python3
import argparse
import json
import math
import os
import socket
import struct
import threading
import time
from pathlib import Path


def log_info(enabled: bool, msg: str):
    if enabled:
        print(msg)


def log_error(msg: str):
    print(msg)


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


def i16_to_u16(v: int) -> int:
    return v & 0xFFFF


def resolve_config_path(cli_path: str) -> str:
    candidates = []
    if cli_path:
        candidates.append(Path(cli_path))
    env_cfg = os.getenv("ASC_CONFIG")
    if env_cfg:
        candidates.append(Path(env_cfg))
    root = Path(__file__).resolve().parents[1]
    candidates.extend(
        [
            Path("config/common_config.json"),
            Path("../config/common_config.json"),
            Path("../../config/common_config.json"),
            root / "config" / "common_config.json",
        ]
    )
    for c in candidates:
        try:
            if c.exists():
                return str(c.resolve())
        except Exception:
            pass
    return ""


def load_runtime_cfg(path: str) -> dict:
    if not path:
        return {}
    try:
        with open(path, "r", encoding="utf-8") as f:
            root = json.load(f)
        runtime = root.get("runtime", {})
        return runtime if isinstance(runtime, dict) else {}
    except Exception as e:
        log_error(f"[sim] warning: failed to load config '{path}': {e}")
        return {}


class GatewayState:
    def __init__(self, module_uid: int, battery_uid: int, solar_uid: int):
        self.lock = threading.Lock()
        self.module_uid = module_uid
        self.battery_uid = battery_uid
        self.solar_uid = solar_uid

        # io_relay: 16-way coils
        self.relay_coils = [False] * 16

        # battery registers (holding)
        self.battery_regs = {
            0x0000: 5870,               # SOC 58.70%
            0x0001: i16_to_u16(112),    # current 1.12A
            0x0002: 5240,               # voltage 52.40V
            0x0007: 240,                # remaining discharge min
            0x0008: 60,                 # remaining charge min
            0x000A: 1,                  # charge MOS on
            0x000B: 1,                  # discharge MOS on
            0x0062: 0,                  # protect status
            0x0064: battery_uid,        # battery slave id
        }
        for i in range(16):
            self.battery_regs[0x0010 + i] = 3300 + (i % 4) * 3
        self.battery_regs[0x0050] = i16_to_u16(250)   # 25.0C
        self.battery_regs[0x0051] = i16_to_u16(260)   # 26.0C

        # solar registers (input/holding)
        pv_power = 18000
        load_power = 9500
        batt_current_raw = 135  # 1.35A
        self.solar_regs = {
            0x3100: 4860,                     # pv voltage 48.60V
            0x3101: 370,                      # pv current 3.70A
            0x3102: pv_power & 0xFFFF,        # pv power low
            0x3103: (pv_power >> 16) & 0xFFFF,  # pv power high
            0x310C: 2410,                     # load voltage 24.10V
            0x310D: 390,                      # load current 3.90A
            0x310E: load_power & 0xFFFF,
            0x310F: (load_power >> 16) & 0xFFFF,
            0x311A: 76,                       # battery SOC %
            0x3200: 0x0000,                   # battery status
            0x3201: 0x0004,                   # charging mode bits -> float charge
            0x3202: 0x0000,                   # discharging status
            0x331A: 5230,                     # battery voltage 52.30V
            0x331B: batt_current_raw & 0xFFFF,
            0x331C: (batt_current_raw >> 16) & 0xFFFF,
        }
        # solar control coils: 0x0000~0x000E
        self.solar_coils = [False] * 15

        # dynamic simulation timeline
        self._sim_start_ts = time.monotonic()

    def _update_dynamic_locked(self):
        # Built-in dynamic model: no external config required.
        t = max(0.0, time.monotonic() - self._sim_start_ts)

        # 10-minute triangle cycle for battery SOC: 95% -> 15% -> 95%
        period = 600.0
        phase = (t % period) / period
        ratio = 1.0 - abs(phase * 2.0 - 1.0)  # 0..1..0
        soc_pct = 15.0 + ratio * 80.0

        charging = 1 if phase >= 0.5 else 0
        current_a = (1.6 + 1.2 * math.sin(t / 17.0)) * (1.0 if charging else -1.0)
        voltage_v = 47.0 + soc_pct * 0.06 + (0.3 if charging else -0.2)

        remain_discharge = int(max(0.0, (soc_pct / 100.0) * 360.0))
        remain_charge = int(max(0.0, ((100.0 - soc_pct) / 100.0) * 180.0))

        self.battery_regs[0x0000] = int(soc_pct * 100.0)  # SOC: 0.01%
        self.battery_regs[0x0001] = i16_to_u16(int(current_a * 100.0))  # 0.01A signed
        self.battery_regs[0x0002] = int(voltage_v * 100.0)  # 0.01V
        self.battery_regs[0x0007] = remain_discharge
        self.battery_regs[0x0008] = remain_charge if charging else 0
        self.battery_regs[0x000A] = 1 if charging else 0
        self.battery_regs[0x000B] = 1 if not charging else 0

        # Cell voltages / temperatures drift with SOC and time.
        for i in range(16):
            self.battery_regs[0x0010 + i] = int(3050 + soc_pct * 10 + 8 * math.sin(t / 11.0 + i * 0.4))
        self.battery_regs[0x0050] = i16_to_u16(int((24.0 + 3.0 * math.sin(t / 35.0)) * 10.0))
        self.battery_regs[0x0051] = i16_to_u16(int((25.0 + 3.5 * math.sin(t / 37.0 + 0.8)) * 10.0))

        # Solar side follows day-like power wave.
        daylight = max(0.0, math.sin(t / 30.0))
        pv_voltage = 34.0 + 18.0 * daylight
        pv_current = 0.5 + 4.8 * daylight
        pv_power = int(pv_voltage * pv_current * 100.0)
        load_power = int((7.5 + 1.8 * math.sin(t / 12.0 + 0.6)) * 1000.0)
        batt_current = int((0.2 + 1.8 * daylight - 0.6) * 100.0)

        self.solar_regs[0x3100] = int(pv_voltage * 100.0)
        self.solar_regs[0x3101] = int(pv_current * 100.0)
        self.solar_regs[0x3102] = pv_power & 0xFFFF
        self.solar_regs[0x3103] = (pv_power >> 16) & 0xFFFF
        self.solar_regs[0x310E] = load_power & 0xFFFF
        self.solar_regs[0x310F] = (load_power >> 16) & 0xFFFF
        self.solar_regs[0x311A] = int(soc_pct)
        self.solar_regs[0x331A] = self.battery_regs[0x0002]
        self.solar_regs[0x331B] = batt_current & 0xFFFF
        self.solar_regs[0x331C] = (batt_current >> 16) & 0xFFFF

    def read_holding(self, unit_id: int, addr: int, qty: int):
        out = []
        with self.lock:
            self._update_dynamic_locked()
            if unit_id == self.battery_uid:
                for i in range(qty):
                    out.append(self.battery_regs.get(addr + i, 0))
                return out
            if unit_id == self.solar_uid:
                for i in range(qty):
                    out.append(self.solar_regs.get(addr + i, 0))
                return out
        return None

    def write_single_register(self, unit_id: int, addr: int, value: int):
        with self.lock:
            if unit_id == self.battery_uid:
                self.battery_regs[addr] = value & 0xFFFF
                return True
            if unit_id == self.solar_uid:
                self.solar_regs[addr] = value & 0xFFFF
                return True
        return None

    def read_coils(self, unit_id: int, addr: int, qty: int):
        if unit_id != self.module_uid:
            return None
        with self.lock:
            bits = []
            for i in range(qty):
                idx = addr + i
                on = self.relay_coils[idx] if 0 <= idx < 16 else False
                bits.append(1 if on else 0)
            return bits

    def write_single_coil(self, unit_id: int, addr: int, value: int):
        if value not in (0xFF00, 0x0000):
            return False
        with self.lock:
            if unit_id == self.module_uid:
                if not (0 <= addr < 16):
                    return False
                self.relay_coils[addr] = (value == 0xFF00)
                return True
            if unit_id == self.solar_uid:
                # Solar driver allows FC05 on 0x0000~0x000E control area.
                if not (0 <= addr <= 0x000E):
                    return False
                self.solar_coils[addr] = (value == 0xFF00)
                return True
        return None


def pack_coils(bits):
    byte_count = int(math.ceil(len(bits) / 8.0))
    out = bytearray([0] * byte_count)
    for i, b in enumerate(bits):
        if b:
            out[i // 8] |= 1 << (i % 8)
    return bytes(out)


def handle_request(state: GatewayState, unit_id: int, pdu: bytes) -> bytes:
    if len(pdu) < 1:
        return pdu_exception(0, 0x03)
    fc = pdu[0]

    # 0x03/0x04 read registers
    if fc in (0x03, 0x04):
        if len(pdu) != 5:
            return pdu_exception(fc, 0x03)
        addr, qty = struct.unpack(">HH", pdu[1:5])
        if qty <= 0 or qty > 125:
            return pdu_exception(fc, 0x03)
        regs = state.read_holding(unit_id, addr, qty)
        if regs is None:
            return pdu_exception(fc, 0x0B)
        payload = b"".join(struct.pack(">H", r & 0xFFFF) for r in regs)
        return struct.pack(">BB", fc, len(payload)) + payload

    # 0x06 write single register
    if fc == 0x06:
        if len(pdu) != 5:
            return pdu_exception(fc, 0x03)
        addr, value = struct.unpack(">HH", pdu[1:5])
        ok = state.write_single_register(unit_id, addr, value)
        if ok is None:
            return pdu_exception(fc, 0x0B)
        if ok is False:
            return pdu_exception(fc, 0x02)
        return pdu

    # 0x01 read coils
    if fc == 0x01:
        if len(pdu) != 5:
            return pdu_exception(fc, 0x03)
        addr, qty = struct.unpack(">HH", pdu[1:5])
        if qty <= 0 or qty > 2000:
            return pdu_exception(fc, 0x03)
        bits = state.read_coils(unit_id, addr, qty)
        if bits is None:
            return pdu_exception(fc, 0x0B)
        payload = pack_coils(bits)
        return struct.pack(">BB", fc, len(payload)) + payload

    # 0x05 write single coil
    if fc == 0x05:
        if len(pdu) != 5:
            return pdu_exception(fc, 0x03)
        addr, value = struct.unpack(">HH", pdu[1:5])
        ok = state.write_single_coil(unit_id, addr, value)
        if ok is None:
            return pdu_exception(fc, 0x0B)
        if ok is False:
            return pdu_exception(fc, 0x02)
        return pdu

    return pdu_exception(fc, 0x01)


def handle_client(conn: socket.socket, client_addr, state: GatewayState, verbose: bool):
    log_info(verbose, f"[sim] client connected: {client_addr}")
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
    except ConnectionError:
        # Expected for short-lived client connections (request/response then close).
        pass
    except Exception as e:
        log_error(f"[sim] client error {client_addr}: {e}")
    finally:
        try:
            conn.close()
        except Exception:
            pass
        log_info(verbose, f"[sim] client disconnected: {client_addr}")


def main():
    parser = argparse.ArgumentParser(description="AIMOX-style Modbus TCP gateway simulator (battery/solar/io_relay)")
    parser.add_argument("--config", default="", help="path to common_config.json")
    parser.add_argument("--host", default="", help="bind host (default from runtime.io_relay.module_ip)")
    parser.add_argument("--port", type=int, default=-1, help="bind port (default from runtime.io_relay.module_port)")
    parser.add_argument("--module-unit-id", type=int, default=-1, help="io_relay unit id (default runtime.io_relay.module_slave_id)")
    parser.add_argument("--battery-unit-id", type=int, default=-1, help="battery unit id (default runtime.battery.battery_slave_id)")
    parser.add_argument("--solar-unit-id", type=int, default=-1, help="solar unit id (default runtime.solar.solar_slave_id)")
    parser.add_argument("--verbose", action="store_true", help="enable verbose connect/disconnect and startup logs")
    args = parser.parse_args()

    cfg_path = resolve_config_path(args.config)
    runtime = load_runtime_cfg(cfg_path)
    io_cfg = runtime.get("io_relay", {}) if isinstance(runtime.get("io_relay", {}), dict) else {}
    batt_cfg = runtime.get("battery", {}) if isinstance(runtime.get("battery", {}), dict) else {}
    solar_cfg = runtime.get("solar", {}) if isinstance(runtime.get("solar", {}), dict) else {}

    host = args.host if args.host else str(io_cfg.get("module_ip", "127.0.0.1"))
    port = args.port if args.port > 0 else int(io_cfg.get("module_port", 502))
    module_uid = (
        args.module_unit_id if args.module_unit_id > 0 else int(io_cfg.get("module_slave_id", 3))
    )
    battery_uid = (
        args.battery_unit_id if args.battery_unit_id > 0 else int(batt_cfg.get("battery_slave_id", 2))
    )
    solar_uid = (
        args.solar_unit_id if args.solar_unit_id > 0 else int(solar_cfg.get("solar_slave_id", 4))
    )

    state = GatewayState(module_uid=module_uid, battery_uid=battery_uid, solar_uid=solar_uid)

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    bind_host = host
    try:
        sock.bind((bind_host, port))
    except OSError as e:
        # Some configs use a device-side LAN IP; local simulator falls back to all interfaces.
        log_error(f"[sim] warning: bind {bind_host}:{port} failed: {e}")
        bind_host = "0.0.0.0"
        sock.bind((bind_host, port))
    sock.listen(16)

    log_info(args.verbose, "[sim] Modbus gateway simulator started")
    if cfg_path:
        log_info(args.verbose, f"[sim] config={cfg_path}")
    else:
        log_info(args.verbose, "[sim] config=not found, use built-in defaults/CLI")
    log_info(args.verbose, f"[sim] listen={bind_host}:{port}")
    log_info(args.verbose, f"[sim] module_uid(io_relay)={module_uid}, battery_uid={battery_uid}, solar_uid={solar_uid}")
    log_info(args.verbose, "[sim] supported FC: 0x01/0x05 (io_relay), 0x03/0x04/0x06 (battery/solar), 0x05 (solar coils)")

    try:
        while True:
            conn, addr = sock.accept()
            t = threading.Thread(target=handle_client, args=(conn, addr, state, args.verbose), daemon=True)
            t.start()
    except KeyboardInterrupt:
        log_info(args.verbose, "\n[sim] stopping...")
    finally:
        try:
            sock.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
