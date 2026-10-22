# Motor_test

基于 **STM32G431RBT6** 的电机测试固件：保留电压调试控制，新增可显式启停的扭矩幅值外环；使用板载 ADC 进行电压内环控制，复用 ADS1220 与卡尔曼库采集、滤波扭矩，并通过串口调参、查询和保存。

**上电目标为零，PWM 停止，扭矩闭环默认关闭；参数恢复不会自动启动输出。**

## 开始前

- 上电后保持传感器无负载，等待校准、归零及首个串口反馈，不用固定等待时长代替就绪判断。开启扭矩环还须收到 `feedback_valid=1`，否则返回 `NotReady`。
- 目标电压上电为 **0 V**，允许范围为 **0～18 V**；此范围是软件限制，不是功率电路的安全认证。
- 扭矩目标按绝对值控制，默认范围为 **0～3**（沿用传感器标定的工程单位）。`TorqueLimit` 可通过串口动态调整并保存；降低上限会钳位当前扭矩目标。软件无法判断用户提高后的上限是否满足机械安全范围。
- **串口断联不会自动停机**，固件保持最后一次有效目标。发送 0 V 是软件控制命令，不能替代硬件急停或断电保护。
- ADC 换算、分压比例及扭矩标定必须与实际电路一致。更换硬件后不能直接沿用本工程参数。

## 工程组成

| 路径 | 职责 |
| --- | --- |
| `Motor_test.ioc` | CubeMX 外设、时钟、DMA、中断及 RTOS 配置 |
| `Core/` | CubeMX 生成的初始化、IRQ 分发和 HAL/RTOS 配置 |
| `UserCode/app.cpp` | 原有初始化、采样/通信任务；TIM2 电压内环与 TIM3 扭矩外环接入 |
| `UserCode/app_control.cpp` | 应用层命令执行、参数映射、输出启停和反馈交接；不实现 PID 或协议算法 |
| `UserCode/app_tasks.cpp` | 原有两个业务任务的栈、控制块、创建及失败清理；不增加任务或改变频率 |
| `UserCode/ADS1220/` | 外部 ADC 驱动及使用案例 |
| `UserCode/HostCommunication/` | 上位机协议、CRC 校验和 UART DMA 收发 |
| `UserCode/Parameters/` | 扭矩 PID、卡尔曼 Q/R 与目标上限的双页 Flash 存储接口 |
| `UserCode/Torque/` | 复用 Math::EKF 的扭矩幅值滤波适配 |
| `UserCode/arena.cpp` | 全局 C++ `new` 使用的 4 KB 静态线性分配区；支持 DMA 类对象的 C++17 对齐分配，单块内存不回收 |
| `Modules/BasicComponents/` | Git 子模块，由 cpkg 管理组件依赖 |
| `Drivers/`、`Middlewares/` | STM32 HAL、CMSIS、FreeRTOS |
| `tests/` | 使用模拟 HAL 的主机端行为回归测试 |
| `cmake/`、`CMakePresets.json` | 工具链、组件链接和 Debug/Release 构建入口 |

模块细节不在本文重复维护：

- [ADS1220 接入和最小案例](UserCode/ADS1220/README.md)
- [串口帧格式、CRC、Python 联调示例和通信约束](UserCode/HostCommunication/README.md)

## 当前硬件配置

以下是本工程的实际资源分配，不是通用板卡接线规范；接入前仍需核对原理图。

| 外设 | 引脚或资源 | 用途 |
| --- | --- | --- |
| ADC2 Rank 1 / IN12 | PB2，DMA 缓冲下标 0 | 辅助电压采集，当前任务记录为 `Vref_5V`，未参与 ADS1220 参考电压补偿 |
| ADC2 Rank 2 / IN15 | PB15，DMA 缓冲下标 1 | 输出电压分压反馈 |
| ADC2 DMA | DMA1 Channel1，Circular，Halfword | 两路扫描数据持续搬运 |
| SPI1 | PA5 SCK、PA6 MISO、PA7 MOSI | ADS1220 通信，模式 1，8 位，当前 SCLK 为 2.25 MHz |
| TIM1 CH1 | PA8 | PWM 输出，当前载波频率 20 kHz |
| TIM2 | 定时器更新中断 | 电压 PID，当前频率 10 kHz |
| TIM3 | 定时器更新中断 | 扭矩 PID 外环，沿用已配置的 100 Hz |
| USART3 | PC10 TX、PC11 RX | 上位机通信，115200、8N1，无流控 |
| USART3 RX DMA | DMA1 Channel2，Circular，Byte | 接收命令帧 |
| USART3 TX DMA | DMA1 Channel3，Normal，Byte | 发送测量反馈 |

频率基于当前 72 MHz 时钟及现有定时器分频。RTOS tick 为 1 kHz；业务中的 `osDelay(1)`、`osDelay(50)` 分别对应约 1 ms、50 ms 的任务延时。

ADS1220 当前使用 AIN0-AIN1 差分输入、128 倍增益、正常模式 20 SPS、外部 REFP0/REFN0 的 5 V 参考、50/60 Hz 抑制。外部参考电压必须真实接入。驱动未管理 CS 引脚，接线与时序限制见其头文件和模块 README。

## 运行流程

启动由 FreeRTOS 的 `Init` 任务组织：

1. 在初始化路径 `new` 创建长期存活的类对象，读取已保存参数；没有有效记录时使用明确默认值。
2. 初始化 ADS1220，执行内部短接偏移校准；启动板载 ADC 校准及循环 DMA。
3. 无负载稳定后，以约 60 ms 间隔平均 32 次有效传感器读数，保存业务零点。源码当前稳定等待为 1 秒，不能以旧注释中的 3 秒推断实际时序。
4. 初始化主机串口接收，创建两个 PID 与卡尔曼对象，启动 TIM2/TIM3 中断；此时 PWM 仍保持停止。
5. 启动原有采样、发送任务后退出初始化任务。两项业务任务使用已启用的静态任务分配接口，栈和控制块从现有 4 KB 启动区分配；采样栈 1024 B、发送栈 512 B，避免继续挤占 3 KB RTOS 堆。不增加任务或修改 CubeMX 生成配置。

内部短接校准消除 ADC 自身偏移；无载归零消除当前传感器链路的零点。**若在启动阶段施加载荷，这部分载荷也会被当作零点扣除。**

| 执行位置 | 工作内容 |
| --- | --- |
| `Task_get_torque` | 消费有序命令、执行参数与保存操作；读取 ADS1220，新样本换算为扭矩幅值并更新卡尔曼估计，随后延时 1 tick |
| `TIM2_Vout_pid_PeriodElapsedCallback` | 读取板载 ADC 电压；输出运行时执行原电压 PID，停止时将 PWM 比较值保持为零 |
| `TIM3_Torque_pid_PeriodElapsedCallback` | 已启用且反馈有效时，直接调用 cpkg `PIDMotor::calc()` 计算电压目标，并限制到 0～18 V |
| `Task_host_communication` | 每轮延时 50 tick，优先发送命令应答，没有待发应答时上传测量反馈；DMA 忙时保留应答、跳过反馈 |

ADS1220 的 `Update()` 未就绪时立即返回 `NotReady`；就绪后仍使用同步 HAL SPI 收发。它不等待一次 ADC 转换，不意味着整个驱动都是异步接口。采样任务只在读取成功时发布新测量值。

扭矩 PID 的 `Kp/Ki/Kd` 直接交给 `PIDMotor`，采样周期已融入离散参数，不额外乘除 `dt`。TIM3 的 100 Hz 不等于传感器新数据率：当前驱动在 Normal20 下至少等待 57 ms 后才再次读取，卡尔曼只处理新读数。默认 `Kp=1, Ki=0, Kd=0, Q=0.01, R=0.01, TorqueLimit=3` 仅是起始调试值，不是已实板整定的稳定性保证。

扭矩外环输出的单位是电压，因此输出上限仍为 18 V，`TorqueLimit` 限制的是扭矩目标，不是 PWM 或电压。关闭扭矩环会清空两个 PID 状态并停止 PWM；使能需要有效反馈和显式命令。原 0 V 电压命令在任意模式下均停止输出；扭矩模式下非零电压命令返回 `ModeConflict`。SPI/数值错误使扭矩环关闭，不会在反馈恢复后自行重新使能。

任务与 ISR 之间的独立共享浮点值采用无锁 `std::atomic<float>` 交接；目标、使能和 PID 配置通过短中断临界区与 TIM3 串行化。电压与扭矩不保证来自同一采样时刻。无命令应答时反馈约为 20 Hz；调参期间应答优先，反馈会让位，不是硬实时周期。

## 编译

### 环境

- CMake 3.22 或更新版本、Ninja。
- Arm GNU Toolchain：`arm-none-eabi-gcc`、`arm-none-eabi-g++` 等工具可从 `PATH` 找到。
- C11、C++17；固件关闭 C++ 异常和 RTTI。
- cpkg，用于同步 `wtrproject.toml` 指定的组件与生成其构建文件。
- STM32CubeMX，用于变更 `.ioc` 并重新生成静态硬件配置；仅编译现有代码不要求启动 CubeMX。

### 初始化依赖

在工程根目录执行：

```text
git submodule update --init --recursive
cpkg sync
```

BasicComponents 子模块地址使用 GitHub SSH，首次获取需要相应的 SSH 访问权限。依赖包含 PIDMotor、CRC、StaticArena、UartRxSync、Math::EKF 和 utils；Watchdog、DMABuffer、Math::LinearAlgebra 由组件依赖引入。

`cmake/wtr_modules.cmake` 和组件 `CMakeLists.txt` 由 cpkg 管理，不要手工维护另一套组件链接规则。

### Debug 构建

```text
cmake --preset Debug
cmake --build --preset Debug
```

输出为 `build/Debug/Motor_test.elf` 和 `Motor_test.map`。Release 使用同名 `Release` 配置预设与构建预设。

本工程没有统一烧录脚本；使用与实际硬件匹配的 ST-Link 调试器或 STM32CubeProgrammer 将 ELF 下载到目标芯片。

## 上位机联调

配套 HTML 上位机位于本机独立目录 `D:\omp\stm32dbg\motor test debug`，不属于本固件 Git 仓库。**本次没有修改或测试上位机**；旧电压帧仍可使用，新增按钮和参数面板需按通信文档接入。

1. USB-UART 使用 3.3 V 电平并共地，适配器 RX 接 PC10，TX 接 PC11；不可直接接 RS-232 电平。
2. 上电保持无载，等待校准和首个反馈。串口不能同时被浏览器、串口助手等多个程序占用。
3. 电压调试仍使用原指令。扭矩调试先查询有效反馈、设置目标，再显式使能；可查询/调整 `Kp/Ki/Kd/Q/R/TorqueLimit`。保存前发送关闭扭矩环或 0 V 命令，等待停机应答，再发送保存并检查结果。
4. 检查上位机显示的是实测电压与实测扭矩，不是目标值。扭矩单位由传感器标定决定，不能仅凭界面上的 Nm 标签确认。
5. 结束实验前主动发送 0 V，并按实际设备要求切断功率输出。

通信采用 14 字节二进制帧及 CRC-16/CCITT-FALSE，不是文本命令；完整字段和可直接发送的示例见[通信说明](UserCode/HostCommunication/README.md)。

## 修改参数与排查

| 现象或需求 | 优先检查 |
| --- | --- |
| 调整电压范围、分压或 PID | `app.cpp` 的 `MAX_VOUT`、`Rtop/Rbottom`、`Vref`、`V_cfg`；同步核对上位机允许范围 |
| 扭矩比例或零点异常 | `Vrange`、`TorqueRange`、ADS1220 增益与参考电压；启动时是否有载荷 |
| 上位机收不到数据 | USART3 引脚、电平、115200 8N1、校准是否结束、UART/DMA IRQ 和回调注册开关 |
| 控制命令无效 | CRC、帧类型、保留字段及 0～18 V 范围；坏帧会被丢弃，不修改目标 |
| 反馈值长时间不变 | 查看 `Update()`/HAL 状态和真实采样链路；应答提供 `feedback_valid`，但不携带数据龄期，不能据此替代传感器健康诊断 |
| 添加业务源文件 | 放入 `UserCode` 后重新执行 CMake 配置；源码通过目录扫描加入目标 |

板级配置以 `.ioc` 为准，通过 CubeMX 修改；业务逻辑放在 `UserCode`。避免在 CubeMX 自动生成区直接添加不可再生的代码。

## 扭矩反馈滤波

`UserCode/Torque/TorqueFeedback.hpp` 是 cpkg `Math::EKF` 的一维应用适配：对无载归零后的扭矩取绝对值，再执行随机游走模型的预测和测量修正，不自写卡尔曼算法。只在 ADS1220 成功提供新样本时更新；TIM3 不能反复把缓存当作新测量更新滤波器。

`Q` 表示每个新样本的过程方差，`R` 表示测量方差，均使用扭矩工程单位的平方；它们不是截止频率，也不按 `dt` 换算。提高 Q 倾向于跟随新测量，提高 R 倾向于更平滑。首次有效样本用于初始化估计；修改 Q/R 后用下一个新样本重新初始化。滤波失败不发布非有限数。

`tests/torque/filter.cpp` 使用真实 EKF 和矩阵库验证正负测量的幅值一致性、波动衰减、Q/R 响应差异及非法数据处理。滤波会引入响应滞后，不能用曲线变平代替机械稳定性验证。

## 参数存储

`STM32G431xx_FLASH.ld` 将程序 Flash 限制为 124 KB，末尾 `0x0801F000～0x0801FFFF` 的两个 2 KB 页预留给参数，不放入固件加载段。链接符号 `__torque_parameters_start` 和 `__torque_parameters_size` 分别表示区域起始地址和大小。

`torque::ParameterStorage` 保存六个参数：`Kp/Ki/Kd/Q/R/torque_limit`，不保存运行目标和使能状态。布局版本为 2，旧截止频率布局不兼容。调用方传入上述链接区域；存储实现还会核对区域确为芯片 Flash 的末尾两页。两页交替写入，CRC-16/CCITT-FALSE 校验记录，最后写提交标记，读回比对后返回结果。加载选择最新有效记录，损坏或未完成的新记录不会覆盖上一份有效参数；没有有效记录时返回 `NotFound`，由应用选择明确的默认参数。

保存是同步操作，必须由单个任务独占 Flash。调用方必须先实际停止输出，并禁止其他路径在保存期间重新启动，才能传入 `output_stopped=true`；零目标本身不是输出已停止的证明。传入 false 时不会擦写。擦除、编程、读回和重新加锁失败均有独立状态；保存失败后不能假定 Flash 已重新锁定。存储层要求所有参数有限，PID 增益和 Q 非负，R 和目标上限大于零；这些数值检查不构成机械安全边界。

这些恢复保证针对可读的有效 Flash 记录，不代表电源异常期间的硬件可靠性认证。主机测试未模拟 Flash ECC/NMI、低电压写入或真实器件寿命；全片擦除、读保护变更或烧录器擦除参数页仍会丢失参数。应用只在收到显式保存命令且 PWM 已停止时写入，重启恢复参数但保留零目标和关闭状态。

主机回归需要完整支持 C++17 的编译器（旧版 MinGW GCC 6.3 不满足）；例如在 WSL 的工程目录运行：

```text
g++ -std=c++17 -Wall -Wextra -Werror -Itests/parameters -IUserCode/Parameters -IModules/BasicComponents/libs/utils/crc UserCode/Parameters/ParameterStorage.cpp tests/parameters/storage.cpp -o build/parameter-storage-test
./build/parameter-storage-test
```

## 验证范围

- `tests/ads1220`：校准、配置失败恢复、新旧数据区分、单次/连续转换、监测换算和数值边界。
- `tests/host_communication`：黄金帧、CRC 损坏、目标范围、IT/DMA 接收、错位同步和发送缓冲生命周期。主机编译运行命令见通信模块 README。
- `tests/parameters`：重启后恢复、逐位记录损坏回退、运行中拒绝保存、非法参数与分区、每个写入步骤中断、HAL 错误及保存代数回绕。
- `tests/torque`：真实 EKF 的幅值/滤波行为，以及启动分配器的普通和过对齐对象分配。
- `tests/torque_application`：真实 `Init`、ADS1220、PID、卡尔曼、UART 协议及 Flash 模块，配合模拟 HAL/调度，验证启停、TIM3 离散 PID、在线参数/上限、模式互斥、显式保存和独立进程启动恢复；不代表实板验证。

这些是主机模拟测试，不是实板或功率电路验证。工程没有统一 `ctest` 入口，构建成功也不代表传感器标定、硬件保护及闭环参数已经适用于实际设备。

### 应用集成回归（WSL）

以下运行真实应用代码，只有 HAL 和调度是模拟。两次独立进程用于验证上电读取已保存参数，而不是只检查同一对象的缓存；测试生成的 Flash 镜像在恢复校验后删除。

```text
g++ -std=c++17 -Wall -Wextra -Werror -fno-pie -no-pie -Wl,--defsym=__torque_parameters_start=0x0801F000,--defsym=__torque_parameters_size=4096 -Itests/torque_application -IUserCode -IUserCode/HostCommunication -IModules/BasicComponents/protocol/UartRxSync -IModules/BasicComponents/memory/DMABuffer -IModules/BasicComponents/services/watchdog -IModules/BasicComponents/libs/utils/crc -IModules/BasicComponents/libs/control/pid_motor -IModules/BasicComponents/libs/math/EKF -IModules/BasicComponents/libs/math/LinearAlgebra -IModules/BasicComponents/memory/StaticArena tests/torque_application/application.cpp UserCode/app_control.cpp UserCode/app_tasks.cpp UserCode/ADS1220/ADS1220.cpp UserCode/HostCommunication/HostCommunication.cpp UserCode/Parameters/ParameterStorage.cpp UserCode/arena.cpp Modules/BasicComponents/libs/control/pid_motor/pid_motor.cpp Modules/BasicComponents/services/watchdog/watchdog.cpp -o build/torque-application-test
./build/torque-application-test --exercise build/torque-application.flash
./build/torque-application-test --restore build/torque-application.flash
```

CubeMX 若重新生成链接脚本，必须再次核对程序区 124 KB 和末尾 4 KB 参数预留；不能让程序重新链接进已使用的参数页。

