# 上位机通信

上位机开发交接见 [HANDOFF.md](HANDOFF.md)：包含 UI 按钮对应命令、启停与保存流程、参数语义、应答匹配、超时策略及接入验收清单。

USART3：115200、8N1，无流控。PC10 为 TX、PC11 为 RX；使用 3.3 V 电平 USB-UART，交叉连接 TX/RX 并共地，不能直接连接 RS-232 电平。

接收复用 `protocol::UartRxSync`（找帧头阶段 IT，帧内容 RX DMA Circular）；发送使用 TX DMA Normal。`HostCommunication` 独占 UART 收发与错误回调，不与串口打印混用。CubeMX 静态配置不变，没有修改 BasicComponents。

## 任务和缓冲区所有权

- 一个长期存活的对象独占 UART；`Init()` 在任务启动前注册回调并开启接收。构造函数不启动硬件。
- 接收 ISR 把完整帧复制为栈快照，校验后发布到 **容量恰为 8 的 SPSC FIFO**；ISR 不执行 PID、参数修改或 Flash 操作，不分配内存。单生产者为接收 ISR，单消费者为命令任务，32 位原子读写索引采用 release/acquire 交接。
- FIFO 满时**丢弃新到的帧**，保留原来八条命令，既不覆盖旧命令也不合并目标。无效帧不会修改已排队的命令。序号只用于应答关联，不做去重、重发或乱序保护；丢弃帧没有应答。
- 命令任务顺序调用 `TakeCommand(Command&)`，在任务上下文执行命令，再调用 `PublishResponse(Response)`。仅有一个待发应答槽；已有应答时 `PublishResponse` 返回 `false`，原应答不变；非有限应答值也返回 `false`。`HasPendingResponse()` 为真时，`TakeCommand` 返回 `false` 且不消耗 FIFO。
- **同一个发送任务独占** `SendPendingResponse()` 和 `SendFeedback()`，优先尝试应答。待发应答与活动 DMA 使用两个不同的持久帧缓冲区。应答只有在 HAL 成功提交 DMA 后才释放待发槽位，随后允许处理下一条命令，即使前一条应答仍在发送。
- `SendPendingResponse()` 在没有应答时返回 `HAL_OK`，不启动 DMA；提交返回 `HAL_BUSY`、`HAL_ERROR` 或 `HAL_TIMEOUT` 时保留应答供之后重试。未初始化时两种发送方法均返回 `HAL_ERROR`。
- 两种发送共用原子 TX 所有权标记；TX 忙时返回 `HAL_BUSY`，不得改写正在 DMA 读取的帧。只有发送完成回调或 TX DMA 错误后的**成功中止**才释放活动发送缓冲区；中止失败继续保持占用。RX 错误只重启 RX 同步，不释放正常 TX。

FIFO 是本模块的固定原子实现：已有 cpkg `libs::RingBuffer` 使用非原子索引，不满足此处 ISR/任务 SPSC 发布要求。CRC、UART 同步和 RX DMA 缓冲继续复用现有依赖。

## 命令和控制边界

- 上电先完成 ADS1220 内部校准和传感器无载归零，再启动控制和通信；这期间不要加载，也不要发送控制帧。看到首个反馈帧后再开始联调。
- 保留旧版电压命令 `0x01` 和反馈 `0x81` 的字节格式；电压命令在传输层仍只接受 `[0, max_voltage]` 有限数（本应用为 `[0, 18] V`），非法值静默丢弃。
- 原 0 V 电压命令在任意模式下都关闭扭矩环并停止 PWM；扭矩环启用时，非零电压命令返回 `ModeConflict`。关闭后不会恢复旧的非零电压目标。
- 新增扭矩幅值目标、使能/失能、PID/Kalman 参数、运行时扭矩目标上限、显式 Flash 保存和状态查询。默认参数为 `Kp=1, Ki=0, Kd=0, Q=0.01, R=0.01, torque_limit=3`；目标和使能状态不持久化。
- 扭矩 PID 仅在既有 TIM3 100 Hz 回调运行，直接使用 `PIDMotor`，不传入或缩放 `dt`。软件 Kalman 仅在任务收到新的 ADS1220 样本时更新；Q 是每个新样本的过程方差，R 是测量方差，不按 Hz 或时间缩放。
- 扭矩模式按暂定 0.5 N·m/V 使用库前馈接口和动态限幅：`Vff=Vmax=min(目标扭矩/0.5,18 V)`；有效零扭矩反馈不能使电压目标越过该上限。手动电压模式不变；不新增通信参数或保存字段。上位机显示及限制说明见 [交接文档](HANDOFF.md#扭矩前馈与动态电压上限)。
- `DecodeCommand` 只检查帧结构、CRC、操作码、参数 ID 和保留字节。新命令的非有限数、负目标、超限目标以及不为 0/1 的使能值交由应用判定，返回 `InvalidValue`，不在接收层静默吞掉。电压命令保留上述历史传输层例外。
- 应用负责检查反馈就绪、模式互斥、硬件调用结果以及保存前输出已停止。保存只持久化六个参数，不持久化目标或使能；启动恢复使用新布局，旧布局不兼容。`SaveParameters` 不是恢复出厂设置，也不是重新读取命令。
- `0x81` 反馈仍是电压、扭矩两个 float，本应用上传的扭矩是卡尔曼滤波后的非负幅值，不是带符号原始读数。新应答中的 `feedback_valid` 提供执行时状态，但不携带数据龄期；两项测量不保证同一采样时刻，单位仍取决于传感器标定。
- 软件使能、零目标和协议应答不代表硬件急停。断联不增加自动停机策略；独立急停由实际设备承担。有限 FIFO 不是可靠传输协议，PC 应等候应答后再发下一条命令，超时后先查询状态，不能盲目重发保存等有副作用操作。

## 固定帧：双向均为 14 字节

所有多字节字段均为小端，浮点使用 IEEE 754 float32。没有帧尾、换行、字符串或打包结构体。

| 偏移 | 长度 | 含义 |
| --- | --- | --- |
| 0 | 1 | 帧头 `A5` |
| 1 | 1 | 版本 `01` |
| 2 | 1 | 下行命令操作码，或上行 `81` / `82` |
| 3 | 1 | 请求序号 / 应答回显请求序号；反馈使用独立发送序号 |
| 4 | 4 | 负载 word0 |
| 8 | 4 | 负载 word1 |
| 12 | 2 | CRC16，低字节在前 |

CRC 使用 **CRC-16/CCITT-FALSE**：多项式 `0x1021`，初值 `0xFFFF`，输入/输出不反射，最终异或 `0x0000`。覆盖偏移 0～11 共 12 字节，不包含 CRC 自身。标准向量 `123456789` 对应 `0x29B1`；不是 MODBUS。

### 下行命令

| 操作码 | 命令 | word0（4～7） | word1（8～11） |
| --- | --- | --- | --- |
| `01` | `SetVoltage` | float32 电压目标 V | 全零 |
| `02` | `SetTorque` | float32 扭矩幅值目标 | 全零 |
| `03` | `SetTorqueEnabled` | uint32，0 失能 / 1 使能 | 全零 |
| `10` | `SetParameter` | uint32 参数 ID | float32 新值 |
| `11` | `GetParameter` | uint32 参数 ID | 全零 |
| `12` | `SaveParameters` | 全零 | 全零 |
| `13` | `GetState` | 全零 | 全零 |

参数 ID：`1=Kp`、`2=Ki`、`3=Kd`、`4=ProcessNoise(Q)`、`5=MeasurementNoise(R)`、`6=TorqueLimit`。所有参数必须有限；PID 增益和 Q 必须 `>=0`，R 和扭矩目标上限必须 `>0`。扭矩目标必须有限且在 `[0, torque_limit]` 内。

未知操作码、未知参数 ID、非零保留字节、错误版本或 CRC 均丢弃，无应答、不入队。

### 上行反馈 `81`

word0 为实测电压，word1 为扭矩反馈，均为 float32。只有成功提交反馈 DMA 后反馈序号才递增并以 256 回绕；应答不会推进反馈序号。非有限反馈不发送。当前工作区任务延时为 10 tick（1 kHz tick 下约 10 ms），无待发应答时约 100 Hz；TX 忙时跳过反馈，不覆盖活动缓冲区，不能据此推断板上旧固件已升级。

### 上行命令应答 `82`

字节 3 回显请求序号。word0 是 uint32 状态字：

| 位 | 含义 |
| --- | --- |
| 0～7 | Result：`0=Ok`、`1=InvalidValue`、`2=NotReady`、`3=ModeConflict`、`4=OutputActive`、`5=StorageError`、`6=HardwareError` |
| 8 | `torque_enabled` |
| 9 | `feedback_valid` |
| 16～23 | 原命令操作码 |
| 其余 | 全零 |

word1 是应用返回的有限 float32 数值：

| 命令 | 成功时 word1 |
| --- | --- |
| `SetVoltage` / `SetTorque` | 已应用的目标 |
| `SetTorqueEnabled` | 使能为 1，失能为 0 |
| `SetParameter` / `GetParameter` | 已应用 / 读取的参数值 |
| `GetState` | 当前选定的扭矩目标，不是电压目标 |
| `SaveParameters` | 存储状态码，成功为 0 |

降低 `TorqueLimit` 会把选定扭矩目标钳位到新上限；需要读取钳位后的目标时发送 `GetState`。应答标志和值来自同一应用状态快照。

通常错误应答 value 为 0，不回显 NaN/Inf。`SaveParameters` 调用存储层后，value 为 `ParameterStorage::Status` 数值：`0=Ok`、`1=NotFound`、`2=InvalidLayout`、`3=InvalidParameters`、`4=OutputActive`、`5=UnlockFailed`、`6=EraseFailed`、`7=ProgramFailed`、`8=VerifyFailed`、`9=LockFailed`。存储返回非零状态时 Result 为 `StorageError`；若应用在调用前发现输出未停止，则 Result 为 `OutputActive`、value 为 0。状态标志是执行时快照，不是传感器健康保证。

### 黄金帧示例

- 设置 6.5 V，序号 7：`A5 01 01 07 00 00 D0 40 00 00 00 00 22 FD`
- 设置扭矩幅值 2.25，序号 8：`A5 01 02 08 00 00 10 40 00 00 00 00 89 6A`
- 使能扭矩，序号 9：`A5 01 03 09 01 00 00 00 00 00 00 00 D0 A2`
- 失能扭矩，序号 10：`A5 01 03 0A 00 00 00 00 00 00 00 00 47 C8`
- 设置 R=0.03125，序号 11：`A5 01 10 0B 05 00 00 00 00 00 00 3D 7D A9`
- 查询 R，序号 12：`A5 01 11 0C 05 00 00 00 00 00 00 00 6D 90`
- 显式保存，序号 13：`A5 01 12 0D 00 00 00 00 00 00 00 00 3F 82`
- 查询状态，序号 14：`A5 01 13 0E 00 00 00 00 00 00 00 00 3E C0`
- 反馈 6.25 V、扭矩 -1.5，序号 0：`A5 01 81 00 00 00 C8 40 00 00 C0 BF 1B FF`
- 设置参数成功应答，序号 80，已使能且反馈有效，值 0.03125：`A5 01 82 50 00 03 10 00 00 00 00 3D 71 85`

建议控制帧不超过 20 Hz，帧间至少留出 2 ms，一次提交完整的 14 字节。接收采用单缓冲同步器；掉字节后可能丢失后续一帧才重新同步。快照复制不构成 DMA 双缓冲，不能保证超过一整帧的 ISR 延迟或 DMA 覆盖期间仍无丢帧。

## 主机回归入口

`tests/host_communication/communication.cpp` 使用真实协议、`UartRxSync` 和 DMA 缓冲类型，配合模拟 HAL；覆盖独立生成的命令/应答黄金帧、逐位 CRC 损坏、全部参数 ID、保留字段、实际 FIFO8 顺序和溢出、单待发应答背压、HAL 提交失败重试、RX 恢复及 TX 缓冲区所有权。应用层控制、参数边界和 Flash 行为由应用/参数测试负责。

已使用 WSL g++ 的 C++17 编译并执行上述主机回归；这不代表已通过真实串口或实板验证。在 WSL 工程目录运行：

```text
g++ -std=c++17 -Wall -Wextra -Werror -Itests/host_communication -IUserCode/HostCommunication -IModules/BasicComponents/protocol/UartRxSync -IModules/BasicComponents/memory/DMABuffer -IModules/BasicComponents/services/watchdog -IModules/BasicComponents/libs/utils/crc UserCode/HostCommunication/HostCommunication.cpp Modules/BasicComponents/services/watchdog/watchdog.cpp tests/host_communication/communication.cpp -o build/host-communication-test
./build/host-communication-test
```
