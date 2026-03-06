# main_test 测试说明

## 0. 前置准备：放到 `ai_safety_ui_test` 目录下

请先将本工程 **拷贝** 或 **git clone** 到 `ai_safety_ui_test/` 目录下（与 UI 测试工程保持同级/同环境），后续文档中的“仓库根目录”均指 `ai_safety_ui_test/ai_safety_devices_manager/`。

`main_test` 是设备管理客户端（DevicesManagerClient）的终端交互测试程序，用于验证 **Pull 槽**（由你设置、设备管理端来读）与 **Push 槽**（设备管理端推送、你按需查询）的行为。

---

## 1. 模拟器启动方法

在**无实体硬件**时，需先启动与 `config/common_config.json` 配置一致的模拟器，否则 `main_test` 在 `init`/`start` 阶段可能因连不上设备而失败。

### 1.1 一键启动（推荐）

在**本仓库根目录**（`ai_safety_devices_manager/`）下执行：

```bash
bash tool/run_all_sims.sh start
```

可选：指定配置文件路径（默认 `config/common_config.json`）：

```bash
bash tool/run_all_sims.sh start --config /path/to/your/common_config.json
```

会按配置自动启动：

- **modbus_gateway**：电池/太阳能/IO 继电器 Modbus TCP 模拟（对应 `battery`/`solar`/`io_relay` 的 `module_ip`/`module_port`）
- **hoist_hook**：吊钩模拟；若配置中 `hoist_hook.transport` 为 `rtu` 则启动 `hoist_hook_rtu_sim.py`（虚拟串口），为 `tcp` 则启动 `hoist_hook_tcp_sim.py`
- **multi_turn_encoder**：多圈编码器 TCP 模拟
- **spd_lidar**：双实例 SPD 激光雷达模拟

### 1.2 停止与状态

```bash
bash tool/run_all_sims.sh stop    # 停止所有已启动的模拟器
bash tool/run_all_sims.sh status  # 查看当前模拟器进程状态
bash tool/run_all_sims.sh restart # 先 stop 再 start
```

### 1.3 配置与模拟器对应关系

- `runtime.battery` / `solar` / `io_relay` 使用 **module_ip:module_port**（如 127.0.0.1:15020）→ 由 **modbus_gateway_sim** 监听。
- `runtime.hoist_hook.transport=rtu` 且 `device=/dev/ttyUSB0` → 需先启动 **hoist_hook_rtu_sim**（会创建虚拟串口或占用指定串口）；若为 `tcp` 则用 **hoist_hook_tcp_sim**。
- `runtime.multi_turn_encoder` 若 `enable=true` 且为 TCP → 需 **multi_turn_encoder_tcp_sim**；若为 RTU 需串口或对应 RTU 模拟。
- `runtime.spd_lidar.instances` → **spd_lidar_dual_sim** 按配置端口起双实例。

若某模块在配置中 `enable=false`，可不启动对应模拟器；未禁用的模块需有对应模拟器或实体设备，否则 `main_test` 可能报连接错误。

---

## 2. 运行 main_test 可执行文件与配置文件

### 2.1 构建

在 **ai_safety_devices_manager** 目录下构建（依赖同级的 `ai_safety_common`，或由父工程提供）：

```bash
cd ai_safety_devices_manager
mkdir -p build
cd build
cmake ..
make -j4 main_test
```

可执行文件生成在：`ai_safety_devices_manager/build/main_test`。

### 2.2 配置文件与路径

- **默认配置文件**：程序默认使用相对路径 `config/common_config.json`。
- **工作目录**：该路径是相对于**当前工作目录（CWD）**解析的，不是相对于可执行文件所在目录。

推荐用法：

**方式 A：在模块根目录下运行（推荐）**

```bash
cd ai_safety_devices_manager
./build/main_test
```

此时 CWD 为 `ai_safety_devices_manager`，默认会找到 `config/common_config.json`。

**方式 B：在 build 目录下运行**

```bash
cd ai_safety_devices_manager/build
./main_test ../config/common_config.json
```

**方式 C：从仓库根目录运行**

若在父仓库根目录执行（例如 `./ai_safety_devices_manager/build/main_test`），默认的 `config/common_config.json` 会在仓库根下查找，通常不存在，需显式传入配置路径：

```bash
./ai_safety_devices_manager/build/main_test ai_safety_devices_manager/config/common_config.json
```

或使用绝对路径：

```bash
./ai_safety_devices_manager/build/main_test /path/to/ai_safety_devices_manager/config/common_config.json
```

### 2.3 命令行参数

```text
./main_test [配置文件路径]
```

- 不传参数：使用默认 `config/common_config.json`（相对当前工作目录）。
- 传一个参数：使用该路径作为配置文件；路径可为相对路径或绝对路径。

若配置文件不存在或无法打开，程序会打印 `loadConfig failed` 并退出。

---

## 3. Slot / Signal 测试方法（交互指令与打印结果）

程序通过**终端输入**驱动 **Pull 槽**（你设值、设备管理端来读），并通过**终端命令**按需查看 **Push 槽**（设备管理端推送的缓存）。设备管理端内部定时轮询并推送，**不会**每推一次就打印一次，只有在你输入对应命令时才打印当前缓存。

### 3.1 命令总览

| 命令 | 说明 |
|------|------|
| `help` | 打印本帮助 |
| `alert [key on\|off]...` | 设置 getAlertMessage 返回值，可多组 key（见下表） |
| `power <none\|on\|off>` | 设置 getBatteryButtonSignals 返回值 |
| `status` | 从 Push 槽读取并打印最近一次 DeviceStatus |
| `crane` | 从 Push 槽读取并打印最近一次 CraneState |
| `quit` | 退出程序（也可 Ctrl+C） |

### 3.2 Pull 槽：alert（getAlertMessage）

- **含义**：你设置的值会在设备管理端轮询时通过 `getAlertMessage()` 被读取，用于告警与喇叭控制逻辑（如 3m/7m 喇叭、双路轮播等）。
- **用法**：`alert <key> <on|off>`，可多组，中间用空格分隔。

**key 取值：**

| key | 含义 |
|-----|------|
| `enable3` | 3m 告警使能 |
| `enable7` | 7m 告警使能 |
| `3m` | 3m 告警状态 |
| `7m` | 7m 告警状态 |

**示例与打印：**

```text
alert enable3 on 7m on
[pull] getAlertMessage: Enable3Alert=1 Enable7Alert=0 Alert3M=0 Alert7M=1
```

```text
alert enable3 on enable7 on 3m on 7m on
[pull] getAlertMessage: Enable3Alert=1 Enable7Alert=1 Alert3M=1 Alert7M=1
```

当 `Enable3Alert && Alert3M` 与 `Enable7Alert && Alert7M` 同时为真时，设备管理端会进入 3m/7m 喇叭轮播（此时为减少刷屏，喇叭控制输出为静默）。

### 3.3 Pull 槽：power（getBatteryButtonSignals）

- **含义**：你设置的值会在设备管理端轮询时通过 `getBatteryButtonSignals()` 被读取，用于电源按钮/继电器控制（0=None, 1=PowerOn, 2=PowerOff）。
- **用法**：`power <none|on|off>`

**示例与打印：**

```text
power on
[pull] getBatteryButtonSignals: on (1)
```

```text
power off
[pull] getBatteryButtonSignals: off (2)
```

```text
power none
[pull] getBatteryButtonSignals: none (0)
```

### 3.4 Push 槽：status（setDeviceStatus 缓存）

- **含义**：设备管理端定时聚合各驱动状态后，通过 `setDeviceStatus` 推送到客户端；程序只做缓存，**不**在每次推送时打印。输入 `status` 时从缓存读取并打印最近一次。
- **用法**：`status`

**可能打印：**

- 尚未收到过推送时：
  ```text
  [push] 尚未收到 setDeviceStatus，请稍后再试
  ```
- 已收到过推送时（示例）：
  ```text
  [push] DeviceStatus:
    solarCharge=Charging trolleyState=Standby hookState=Active
    trolleyBattery: percent=59% remainingMin=240 isCharging=1 chargingTimeMin=60 voltageV=52.4 currentA=1.12
    hookBattery: percent=59% remainingMin=240 isCharging=0 chargingTimeMin=0 voltageV=52.4 currentA=1.12
  ```

字段含义：`solarCharge`（太阳能充电状态）、`trolleyState`/`hookState`（小车/吊钩状态）、`trolleyBattery`/`hookBattery`（电池信息：电量、剩余时间、是否充电、电压电流等）。

### 3.5 Push 槽：crane（setCraneState 缓存）

- **含义**：设备管理端通过 `setCraneState` 推送吊钩/小车距离等；程序只缓存，输入 `crane` 时打印最近一次。
- **用法**：`crane`

**可能打印：**

- 尚未收到过推送时：
  ```text
  [push] 尚未收到 setCraneState，请稍后再试
  ```
- 已收到过推送时（示例）：
  ```text
  [push] CraneState: hookToTrolley=0m groundToTrolley=0m
  ```

### 3.6 小结：Slot 与命令对应关系

| Slot 类型 | 槽名称 | 测试方式 | 说明 |
|-----------|--------|----------|------|
| Pull | getAlertMessage | `alert [key on\|off]...` | 设置后由设备管理端轮询读取，影响告警/喇叭逻辑 |
| Pull | getBatteryButtonSignals | `power none\|on\|off` | 设置后由设备管理端轮询读取，影响电源按钮/继电器 |
| Push | setDeviceStatus | `status` | 按需打印最近一次推送的 DeviceStatus |
| Push | setCraneState | `crane` | 按需打印最近一次推送的 CraneState |

所有交互均通过终端输入完成；Push 数据仅在输入 `status`/`crane` 时打印，避免刷屏。
