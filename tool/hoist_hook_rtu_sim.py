#!/usr/bin/env python3
import argparse
import errno
import json
import os
import pty
import struct
import threading
import time
from pathlib import Path


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
        print(f"[sim-rtu] warning: failed to load config '{config_path}': {e}")
        return {}


class HoistHookState:
    def __init__(self):
        self.lock = threading.Lock()
        self.warning_light = 0  # 0x0000
        self.speaker_7m = 0  # 0x0001
        self.speaker_3m = 0  # 0x0002
        self.valid_mask = 0x0003  # 0x0003
        self.uid_base = 0xA1B20000
        self.heartbeat = 1
        self.volume = 20  # 0x0067
        self.time_sync_hhmm = 0  # 0x0074, high byte=hour low byte=minute

    def _hook_status_regs_100_110(self):
        battery_raw = 5870
        remain_discharge_min = 240
        work_mode = 1
        charging_flag = 0
        remain_charge_min = 0
        voltage_raw = 5240
        current_raw = 112

        horn_status = 1 if (self.speaker_7m or self.speaker_3m) else 0
        return {
            0x0064: 1 if self.warning_light else 0,
            0x0065: horn_status,
            0x0066: battery_raw,
            0x0067: self.volume,
            0x0068: self.heartbeat,
            0x0069: remain_discharge_min,
            0x006A: work_mode,
            0x006B: charging_flag,
            0x006C: remain_charge_min,
            0x006D: voltage_raw,
            0x006E: current_raw,
        }

    def _rfid_group_regs(self):
        regs = []
        for i in range(8):
            uid = (self.uid_base + i) & 0xFFFFFFFF
            uid_hi = (uid >> 16) & 0xFFFF
            uid_lo = uid & 0xFFFF
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
                status_regs = self._hook_status_regs_100_110()
                for i in range(qty):
                    a = addr + i
                    if a == 0x0000:
                        out.append(self.warning_light)
                    elif a == 0x0001:
                        out.append(self.speaker_7m & 0x0001)
                    elif a == 0x0002:
                        out.append(self.speaker_3m & 0x0001)
                    elif a == 0x0003:
                        out.append(self.valid_mask)
                    elif 0x0004 <= a <= 0x001B:
                        out.append(rfid_regs[a - 0x0004])
                    elif 0x0064 <= a <= 0x006E:
                        out.append(status_regs.get(a, 0))
                    elif a == 0x0074:
                        out.append(self.time_sync_hhmm)
                    else:
                        out.append(0)
            elif unit_id == self.power_uid:
                status_regs = self._hook_status_regs_100_110()
                for i in range(qty):
                    a = addr + i
                    if 0x0064 <= a <= 0x006E:
                        out.append(status_regs.get(a, 0))
                    else:
                        out.append(0)
            else:
                out = [0] * qty
            return out

    def write_single(self, unit_id: int, addr: int, value: int):
        with self.lock:
            if unit_id != self.hook_uid:
                return False
            if addr == 0x0000:
                self.warning_light = 1 if (value & 0x1) else 0
                return True
            if addr == 0x0001:
                self.speaker_7m = 1 if (value & 0x1) else 0
                return True
            if addr == 0x0002:
                self.speaker_3m = 1 if (value & 0x1) else 0
                return True
            if addr == 0x0003:
                self.valid_mask = value & 0x00FF
                return True
            if addr == 0x0067:
                # Simulate writable speaker volume register DEC103.
                self.volume = max(0, min(30, value & 0xFFFF))
                return True
            if addr == 0x0068:
                # Simulate writable heartbeat register DEC104.
                self.heartbeat = value & 0xFFFF
                return True
            if addr == 0x0074:
                # Simulate writable time-sync register DEC116.
                self.time_sync_hhmm = value & 0xFFFF
                return True
            return False


def build_exception(uid: int, fc: int, exc_code: int) -> bytes:
    payload = struct.pack("BBB", uid & 0xFF, (fc | 0x80) & 0xFF, exc_code & 0xFF)
    return with_crc(payload)


def handle_rtu_request(state: HoistHookState, req8: bytes) -> bytes:
    if len(req8) != 8:
        return b""
    uid = req8[0]
    fc = req8[1]
    addr = (req8[2] << 8) | req8[3]
    data = (req8[4] << 8) | req8[5]

    recv_crc = req8[6] | (req8[7] << 8)
    calc_crc = crc16_modbus(req8[:6])
    if recv_crc != calc_crc:
        return b""

    if fc == 0x03:
        qty = data
        if qty <= 0 or qty > 125:
            return build_exception(uid, fc, 0x03)
        regs = state.read_holding(uid, addr, qty)
        body = bytearray([uid, fc, qty * 2])
        for r in regs:
            body.extend([(r >> 8) & 0xFF, r & 0xFF])
        return with_crc(bytes(body))

    if fc == 0x06:
        ok = state.write_single(uid, addr, data)
        if not ok:
            return build_exception(uid, fc, 0x02)
        return with_crc(req8[:6])

    return build_exception(uid, fc, 0x01)


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


def main():
    parser = argparse.ArgumentParser(description="Hoist hook Modbus RTU simulator (virtual serial)")
    parser.add_argument("--config", default="", help="path to common_config.json")
    parser.add_argument("--device", default="", help="virtual serial path (default runtime.hoist_hook.device)")
    parser.add_argument("--hook-unit-id", type=int, default=-1, help="hook slave id (default from runtime.hoist_hook.hook_slave_id)")
    parser.add_argument("--power-unit-id", type=int, default=-1, help="power slave id (default from runtime.hoist_hook.power_slave_id)")
    parser.add_argument("--verbose", action="store_true", help="print rx/tx frame debug")
    args = parser.parse_args()

    cfg_path = resolve_config_path(args.config)
    cfg = load_hook_config(cfg_path)
    device = args.device if args.device else str(cfg.get("device", "/tmp/ttyHOIST0"))
    hook_uid = args.hook_unit_id if args.hook_unit_id > 0 else int(cfg.get("hook_slave_id", 3))
    power_uid = args.power_unit_id if args.power_unit_id > 0 else int(cfg.get("power_slave_id", 4))

    state = HoistHookState()
    state.hook_uid = hook_uid
    state.power_uid = power_uid

    master_fd, slave_fd = pty.openpty()
    slave_path = os.ttyname(slave_fd)
    # Keep slave fd open in simulator process; otherwise master read may raise EIO
    # when no external client currently holds the slave endpoint.
    keep_slave_fd = slave_fd

    ensure_device_link(device, slave_path)

    print("[sim-rtu] hoist_hook rtu simulator started")
    if cfg_path:
        print(f"[sim-rtu] config={cfg_path}")
    print(f"[sim-rtu] virtual serial: link {device} -> {slave_path}")
    print(f"[sim-rtu] hook_uid={hook_uid} power_uid={power_uid}")
    print("[sim-rtu] supported: FC03 read, FC06 write")

    buf = bytearray()
    try:
        while True:
            try:
                data = os.read(master_fd, 256)
            except OSError as e:
                if e.errno == errno.EIO:
                    # No active peer on slave side yet; keep simulator alive.
                    time.sleep(0.05)
                    continue
                raise
            if not data:
                time.sleep(0.01)
                continue
            buf.extend(data)
            while len(buf) >= 8:
                frame = bytes(buf[:8])
                # FC03/FC06 request length in this project is fixed 8 bytes in RTU.
                if frame[1] not in (0x03, 0x06):
                    del buf[0]
                    continue
                resp = handle_rtu_request(state, frame)
                del buf[:8]
                if resp:
                    if args.verbose:
                        print(f"[sim-rtu] rx={frame.hex()} tx={resp.hex()}")
                    os.write(master_fd, resp)
    except KeyboardInterrupt:
        print("\n[sim-rtu] stopped")
    finally:
        try:
            os.close(master_fd)
        except Exception:
            pass
        try:
            os.close(keep_slave_fd)
        except Exception:
            pass


if __name__ == "__main__":
    main()
