# AI Safety Controller SDK

`ai_safety_controller` is organized as a standalone SDK project, aligned with the layered structure of `CRC_driver_sdk`.

## Layout

- `core/`: common utilities, sensor factory, and six driver libraries
- `application/`: unified SDK interface (`ai_safety_controller::Interface`)
- `demo/`: minimal executable to verify SDK integration
- `config/`: SDK build/runtime configuration

## Build (standalone)

```bash
cd src/ai_safety_controller
mkdir -p build
cd build
cmake ..
make -j
./demo/ai_safety_controller_demo
```

## Demo CLI Quick Start

- `help`
- `showcfg`
- `loadcfg ./config/common_config.json`
- `sensors`
- `cmds battery`
- `battery basic`
- `solar status`
- `io_relay read`
- `multi_turn_encoder status`
- `quit`

## Multi-turn Encoder TCP Simulator

When `multi_turn_encoder.transport` is set to `tcp`, you can use the built-in simulator for integration tests.
The simulator now reads `runtime.multi_turn_encoder` from `common_config.json` by default:

```bash
cd src/ai_safety_controller
python3 tool/multi_turn_encoder_tcp_sim.py
```

Recommended config snippet in `config/common_config.json`:

```json
"multi_turn_encoder": {
  "enable": true,
  "transport": "tcp",
  "ip": "127.0.0.1",
  "port": 1502,
  "slave": 1,
  "query_hz": 1.0
}
```

Optional: override with CLI or a specific config file:

```bash
python3 tool/multi_turn_encoder_tcp_sim.py --config ./config/common_config.json --port 1502 --unit-id 1
```

If you want turns to change with encoder polling frequency, use:

```bash
python3 tool/multi_turn_encoder_tcp_sim.py --turns-mode read_rate --speed-rps 0.05
```

- `read_rate`: turns increment by `speed_rps * (time since last read)` and only update on reads
- `time`: turns change continuously with wall clock time

## Hoist Hook TCP Simulator

For `runtime.hoist_hook`, you can run a matching Modbus TCP simulator:

```bash
cd src/ai_safety_controller
python3 tool/hoist_hook_tcp_sim.py
```

It auto-loads `runtime.hoist_hook.module_ip/module_port/hook_slave_id/power_slave_id` from `common_config.json`
(CLI flags override config values).

## SPD Lidar Dual-instance Simulator

For two single-point lidar instances (`front/rear`), run:

```bash
cd src/ai_safety_controller
python3 tool/spd_lidar_dual_sim.py
```

It reads enabled instances from `runtime.spd_lidar.instances` in `common_config.json` and starts one TCP server per instance.
By default it replies to `single` command frames (`55 AA 88 FF FF FF FF chk`) with fixed distance frames.

## Notes

- Legacy ROS packages remain untouched. Migration is additive under `ai_safety_controller/`.
- Stage-1 has switched all six drivers to SDK-owned bridge paths under `core/drivers/`.
- Stage-2 has fully localized all six driver source/header files into SDK paths.
- Stage-3 adds a unified command dispatch entry in `application/interface`.
- Stage-4 loads runtime defaults from `config/common_config.json` (or `ASC_CONFIG`) for all Modbus modules and encoder.
