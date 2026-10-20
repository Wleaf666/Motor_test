# 上位机通信

USART3：115200、8N1，无流控。PC10 为 TX、PC11 为 RX；使用 3.3 V 电平 USB-UART，交叉连接 TX/RX 并共地，不能直接连接 RS-232 电平。

接收复用 `protocol::UartRxSync`（找帧头阶段 IT，帧内容 RX DMA Circular）；反馈使用 TX DMA Normal。`HostCommunication` 独占 UART 收发与错误回调，不与串口打印混用。CubeMX 静态配置保持现状，没有修改 BasicComponents。

## 行为

- 上电先完成 ADS1220 内部校准和传感器无载归零，再启动控制和通信；这期间不要加载，也不要发送控制帧。看到首个反馈帧后即可发送指令。
- `V_target` 上电为 0 V。合法下行帧设置电压目标，电压 PID 保持闭环；扭矩只测量和上传，不参与控制。
- 电压目标只接受 `[0, 18] V` 的有限数，不钳位；CRC 错误、类型/版本错误、保留字段非零或非法目标均丢弃，不改变目标。
- 多条有效命令积压时只保留最新值。序号用于观测，不实现应答、去重、重发或乱序保护。
- 每 50 ms（20 Hz）发送最新电压、扭矩测量值；两项取自现有业务反馈，不保证同一采样时刻。扭矩保持原 `TorqueRange=20` 的工程单位，物理单位须与传感器标定一致。它不是目标扭矩。
- ADC/传感器测量失败时沿用业务层最近有效值，协议没有数据龄期或有效性标志；不能用收到帧代替传感器健康诊断。
- TX 忙时跳过本周期，不覆盖发送中缓冲；`app.cpp` 的 `Host_tx_status` 可用于调试。断联不自动停机，继续保持最后目标；停止需发送 0 V，独立急停由实际设备承担。

## 代码组织

- `HostProtocol.hpp`：只负责固定帧的字节编码、解码和 CRC，不访问 UART 或业务变量。
- `HostCommunication`：负责 UART 配置检查、回调绑定、命令邮箱及 TX DMA 缓冲生命周期；初始化、收帧、发帧各自独立。
- `app.cpp` 的 `Host_update()`：把已经校验的最新目标交给 `V_target`。复用现有 `Task_get_torque` 的 1 ms 周期调用，不新增接收线程。
- `Task_host_communication`：只负责 `osDelay(50)` 后发送反馈；按当前 FreeRTOS 的 1 kHz 时基约为 20 Hz，不再使用当前时间、上次发送时间和周期差值判断。

任务总数保持不变。发送周期是任务延时加本次处理时间，不承诺硬实时 50 ms；更改内核 tick 频率时须同步调整任务延时。命令校验、最新值语义、DMA 忙保护和错误恢复保持不变，PC 上位机无需修改。


## 固定帧：双向均为 14 字节

所有多字节字段均为小端。浮点数使用 IEEE 754 float32。没有帧尾、换行或字符串。

| 字节偏移 | 长度 | 下行：设置电压 | 上行：测量反馈 |
| --- | --- | --- | --- |
| 0 | 1 | 帧头 `A5` | 帧头 `A5` |
| 1 | 1 | 版本 `01` | 版本 `01` |
| 2 | 1 | 类型 `01` | 类型 `81` |
| 3 | 1 | 上位机序号，0～255 | 单片机发送序号，成功提交后递增并回绕 |
| 4 | 4 | 目标输出电压，V | 实测输出电压 `V_feedback`，V |
| 8 | 4 | 保留，四个字节必须全为 0 | 实测扭矩 `Torque_feedback` |
| 12 | 2 | CRC16，低字节在前 | CRC16，低字节在前 |

CRC 使用 **CRC-16/CCITT-FALSE**：多项式 `0x1021`，初值 `0xFFFF`，输入/输出不反射，最终异或 `0x0000`。覆盖偏移 0～11 共 12 字节，不包含 CRC 自身。标准向量 `123456789` 对应 `0x29B1`；本协议不是 MODBUS。

可直接用于串口助手的十六进制帧：

- 设置 6.5 V，序号 7：`A5 01 01 07 00 00 D0 40 00 00 00 00 22 FD`
- 反馈示例：6.25 V、扭矩 -1.5、序号 0：`A5 01 81 00 00 00 C8 40 00 00 C0 BF 1B FF`

建议控制帧不超过 20 Hz，帧间至少留出 2 ms；PC 端一次提交完整的 14 字节。接收采用单缓冲同步器，掉字节后可能丢失后续一帧才重新同步；不保证超过一整帧的 ISR 延迟下无丢帧。任务只取已校验的最新目标，不在接收中断中运行 PID。

## Python 最小联调示例

安装 `pyserial`，将下例保存为脚本；修改端口和 `TARGET`。程序先等首个有效反馈，再设置一次目标，并持续打印反馈。它不会在退出时自动停止输出。

```python
import binascii
import math
import struct
import serial

PORT = "COM5"
TARGET = 6.5


def command(voltage, sequence=0):
    if not math.isfinite(voltage) or not 0 <= voltage <= 18:
        raise ValueError("目标电压必须在 0～18 V")
    body = struct.pack("<BBBBf4x", 0xA5, 1, 1, sequence, voltage)
    return body + struct.pack("<H", binascii.crc_hqx(body, 0xFFFF))


def feedback_frames(buffer):
    while True:
        start = buffer.find(b"\xA5")
        if start < 0:
            buffer.clear()
            return
        del buffer[:start]
        if len(buffer) < 14:
            return
        frame = bytes(buffer[:14])
        crc = struct.unpack_from("<H", frame, 12)[0]
        if frame[1:3] != b"\x01\x81" or binascii.crc_hqx(frame[:12], 0xFFFF) != crc:
            del buffer[0]
            continue
        del buffer[:14]
        sequence, voltage, torque = struct.unpack_from("<Bff", frame, 3)
        if math.isfinite(voltage) and math.isfinite(torque):
            yield sequence, voltage, torque


with serial.Serial(PORT, 115200, timeout=0.1) as uart:
    buffer = bytearray()
    sent = False
    print("等待单片机校准完成后的首个反馈；上电期间请保持无载")
    while True:
        buffer.extend(uart.read(128))
        for sequence, voltage, torque in feedback_frames(buffer):
            if not sent:
                uart.write(command(TARGET))
                sent = True
            print(f"seq={sequence:3d}  voltage={voltage:.4f} V  torque={torque:.4f}")
```

## 已执行验证

固件通过宿主工程构建。`tests/host_communication/communication.cpp` 使用真实协议实现、真实 UartRxSync 和 DMA 缓冲类型，配合模拟 HAL 检查：Python 独立生成的黄金帧、逐位 CRC 损坏、边界和非有限数、分段输入、错位重同步、最新命令邮箱、TX 忙保护和错误恢复。

主机回归需要支持 C++17 的编译器（例如 `clang++` 或 `zig c++`），在工程根目录运行：

```text
clang++ -std=c++17 -Wall -Wextra -Werror -Itests/host_communication -IUserCode/HostCommunication -IModules/BasicComponents/protocol/UartRxSync -IModules/BasicComponents/memory/DMABuffer -IModules/BasicComponents/services/watchdog -IModules/BasicComponents/libs/utils/crc UserCode/HostCommunication/HostCommunication.cpp Modules/BasicComponents/services/watchdog/watchdog.cpp tests/host_communication/communication.cpp -o build/host-communication-test.exe
build/host-communication-test.exe
```

另已抽取重构后的 `Host_update()`、采样任务和反馈任务执行冒烟场景：最新合法命令在采样任务中生效、坏帧不覆盖目标、采样继续更新、反馈任务每轮延时 50 tick 后发送；反馈 float32 扭矩精度保留。固件构建与协议回归均通过；上述为编译/模拟结果，未代替实板串口、实际 PWM 输出和传感器联调。
