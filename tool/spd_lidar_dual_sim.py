#!/usr/bin/env python3
import argparse
import json
import os
import socket
import threading
import time
from pathlib import Path


HDR1 = 0x55
HDR2 = 0xAA
CMD_SINGLE = 0x88
FRAME_SIZE = 8


def checksum_send(frame7):
    # SDK send-side checksum: sum bytes [2..6]
    return sum(frame7[2:7]) & 0xFF


def checksum_recv(frame7):
    # SDK recv-side checksum: sum bytes [0..6]
    return sum(frame7[0:7]) & 0xFF


def resolve_config_path(cli_path: str):
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


def load_lidar_instances(cfg_path: str):
    if not cfg_path:
        return []
    try:
        with open(cfg_path, "r", encoding="utf-8") as f:
            root = json.load(f)
        runtime = root.get("runtime", {})
        spd = runtime.get("spd_lidar", {})
        instances = spd.get("instances", [])
        if not isinstance(instances, list):
            return []
        out = []
        for one in instances:
            if not isinstance(one, dict):
                continue
            if not one.get("enable", False):
                continue
            out.append(one)
        return out
    except Exception as e:
        print(f"[spd_sim] warning: failed to load config '{cfg_path}': {e}")
        return []


class LidarInstanceServer:
    def __init__(self, one: dict, default_host: str, default_port: int):
        self.id = str(one.get("id", "lidar"))
        self.mode = str(one.get("mode", "client")).lower()
        self.local_ip = str(one.get("local_ip", "127.0.0.1"))
        self.local_port = int(one.get("local_port", 8234))
        self.device_ip = str(one.get("device_ip", "127.0.0.1"))
        self.device_port = int(one.get("device_port", 8234))
        self.distance_mm = int(one.get("sim_distance_mm", 1234))
        self.status = int(one.get("sim_status", 0))
        self.running = True
        self._conn = None
        self._sock = None

        # For sdk mode=client, sdk connects to device_ip:device_port.
        # This simulator acts as device-side TCP server by default.
        self.host = default_host if default_host else self.device_ip
        self.port = default_port if default_port > 0 else self.device_port

    def _build_single_reply(self):
        d = self.distance_mm & 0xFFFF
        frame7 = [HDR1, HDR2, CMD_SINGLE, self.status & 0xFF, 0x00, (d >> 8) & 0xFF, d & 0xFF]
        return bytes(frame7 + [checksum_recv(frame7)])

    def _try_parse_frame(self, data: bytes):
        if len(data) < FRAME_SIZE:
            return False
        # Only parse first full frame for simplicity.
        frame = list(data[:FRAME_SIZE])
        if frame[0] != HDR1 or frame[1] != HDR2 or frame[2] != CMD_SINGLE:
            return False
        expect = checksum_send(frame[:7])
        return frame[7] == expect

    def serve_client_mode(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self.host, self.port))
        self._sock.listen(4)
        print(f"[spd_sim:{self.id}] mode=client-sim server listen={self.host}:{self.port}")

        while self.running:
            try:
                conn, addr = self._sock.accept()
            except Exception:
                if self.running:
                    time.sleep(0.2)
                continue
            self._conn = conn
            print(f"[spd_sim:{self.id}] connected by {addr}")
            try:
                while self.running:
                    data = conn.recv(256)
                    if not data:
                        break
                    if self._try_parse_frame(data):
                        conn.sendall(self._build_single_reply())
            except Exception:
                pass
            finally:
                try:
                    conn.close()
                except Exception:
                    pass
                self._conn = None
                print(f"[spd_sim:{self.id}] disconnected")

    def close(self):
        self.running = False
        try:
            if self._conn:
                self._conn.close()
        except Exception:
            pass
        try:
            if self._sock:
                self._sock.close()
        except Exception:
            pass


def main():
    parser = argparse.ArgumentParser(description="SPD lidar dual-instance TCP simulator")
    parser.add_argument("--config", default="", help="path to common_config.json")
    parser.add_argument("--host", default="", help="override server host for all instances")
    parser.add_argument("--base-port", type=int, default=0, help="override base port for all instances; second instance uses +1")
    parser.add_argument("--distance-front", type=int, default=1234, help="front distance mm")
    parser.add_argument("--distance-rear", type=int, default=2345, help="rear distance mm")
    args = parser.parse_args()

    cfg_path = resolve_config_path(args.config)
    instances = load_lidar_instances(cfg_path)
    if not instances:
        # fallback two default instances for quick bring-up
        instances = [
            {"id": "front", "enable": True, "mode": "client", "device_ip": "127.0.0.1", "device_port": 8234},
            {"id": "rear", "enable": True, "mode": "client", "device_ip": "127.0.0.1", "device_port": 8235},
        ]

    # keep at most two instances as requested
    instances = instances[:2]
    if len(instances) == 1:
        # auto add second peer if config only enabled one
        first = instances[0]
        instances.append(
            {
                "id": "rear" if str(first.get("id", "")).lower() != "rear" else "front",
                "enable": True,
                "mode": "client",
                "device_ip": first.get("device_ip", "127.0.0.1"),
                "device_port": int(first.get("device_port", 8234)) + 1,
            }
        )

    # assign deterministic distances
    for one in instances:
        if str(one.get("id", "")).lower() == "rear":
            one["sim_distance_mm"] = args.distance_rear
        else:
            one["sim_distance_mm"] = args.distance_front

    if args.base_port > 0:
        for i, one in enumerate(instances):
            one["device_port"] = args.base_port + i
            one["device_ip"] = args.host if args.host else one.get("device_ip", "127.0.0.1")

    print("[spd_sim] dual-instance simulator starting")
    if cfg_path:
        print(f"[spd_sim] config={cfg_path}")
    else:
        print("[spd_sim] config=not found, using defaults")

    sims = []
    threads = []
    try:
        for i, one in enumerate(instances):
            sim = LidarInstanceServer(one, args.host, 0)
            sims.append(sim)
            t = threading.Thread(target=sim.serve_client_mode, daemon=True)
            t.start()
            threads.append(t)
            print(
                f"[spd_sim] instance[{i}] id={sim.id} mode={sim.mode} "
                f"device={sim.device_ip}:{sim.device_port} listen={sim.host}:{sim.port} "
                f"distance_mm={sim.distance_mm}"
            )
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n[spd_sim] stopping ...")
    finally:
        for sim in sims:
            sim.close()
        for t in threads:
            t.join(timeout=0.5)
        print("[spd_sim] stopped")


if __name__ == "__main__":
    main()
