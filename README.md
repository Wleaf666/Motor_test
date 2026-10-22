# Motor_test

基于 **STM32G431RBT6** 的电机测试固件：通过上位机设置输出电压，使用板载 ADC 进行电压闭环，使用 ADS1220 采集扭矩传感器信号，并通过串口反馈实测电压与扭矩。

**当前只启用电压闭环，扭矩用于测量，不参与闭环控制。**

## 开始前

- 上电后保持传感器无负载，等待约 **7 秒**完成校准和归零；收到首个串口反馈后再发送控制指令。
- 目标电压上电为 **0 V**，允许范围为 **0～18 V**；此范围是软件限制，不是功率电路的安全认证。
- **串口断联不会自动停机**，固件保持最后一次有效目标。发送 0 V 是软件控制命令，不能替代硬件急停或断电保护。
- ADC 换算、分压比例及扭矩标定必须与实际电路一致。更换硬件后不能直接沿用本工程参数。

## 工程组成

| 路径 | 职责 |
| --- | --- |
| `Motor_test.ioc` | CubeMX 外设、时钟、DMA、中断及 RTOS 配置 |
| `Core/` | CubeMX 生成的初始化、IRQ 分发和 HAL/RTOS 配置 |
| `UserCode/app.cpp` | 启动流程、电压 PID、扭矩采样和通信任务 |
| `UserCode/ADS1220/` | 外部 ADC 驱动及使用案例 |
| `UserCode/HostCommunication/` | 上位机协议、CRC 校验和 UART DMA 收发 |
| `UserCode/Parameters/` | 扭矩 PID 与低通截止频率参数的双页 Flash 存储接口 |
| `UserCode/arena.cpp` | 全局 C++ `new` 使用的 4 KB 静态线性分配区，单块内存不回收 |
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
| TIM3 | 定时器更新中断 | 扭矩工程量换算，当前频率 100 Hz，并非扭矩闭环 |
| USART3 | PC10 TX、PC11 RX | 上位机通信，115200、8N1，无流控 |
| USART3 RX DMA | DMA1 Channel2，Circular，Byte | 接收命令帧 |
| USART3 TX DMA | DMA1 Channel3，Normal，Byte | 发送测量反馈 |

频率基于当前 72 MHz 时钟及现有定时器分频。RTOS tick 为 1 kHz；业务中的 `osDelay(1)`、`osDelay(50)` 分别对应约 1 ms、50 ms 的任务延时。

ADS1220 当前使用 AIN0-AIN1 差分输入、128 倍增益、正常模式 20 SPS、外部 REFP0/REFN0 的 5 V 参考、50/60 Hz 抑制。外部参考电压必须真实接入。驱动未管理 CS 引脚，接线与时序限制见其头文件和模块 README。

## 运行流程

启动由 FreeRTOS 的 `Init` 任务组织：

1. 初始化 ADS1220，执行内部短接偏移校准。
2. 启动板载 ADC 校准及循环 DMA。
3. 无负载稳定 3 秒，再以约 60 ms 间隔平均 32 次有效传感器读数，保存业务零点。
4. 初始化主机串口接收，创建 PID，启动 TIM2/TIM3 中断及 PWM。
5. 启动采样任务和反馈任务，初始化任务退出。

内部短接校准消除 ADC 自身偏移；无载归零消除当前传感器链路的零点。**若在启动阶段施加载荷，这部分载荷也会被当作零点扣除。**

| 执行位置 | 工作内容 |
| --- | --- |
| `Task_get_torque` | 每轮取最新合法电压命令，尝试读取 ADS1220，成功后更新零点补偿电压，随后延时 1 tick |
| `TIM2_Vout_pid_PeriodElapsedCallback` | 读取板载 ADC 电压，执行电压 PID 并更新 PWM 比较值 |
| `TIM3_Torque_pid_PeriodElapsedCallback` | 将传感器补偿电压换算为扭矩 |
| `Task_host_communication` | 每轮延时 50 tick 后，通过 TX DMA 上传最新实测电压和扭矩 |

ADS1220 的 `Update()` 未就绪时立即返回 `NotReady`；就绪后仍使用同步 HAL SPI 收发。它不等待一次 ADC 转换，不意味着整个驱动都是异步接口。采样任务只在读取成功时发布新测量值。

任务与 ISR 之间的独立共享浮点值采用无锁 `std::atomic<float>` 交接。电压与扭矩分别更新，**不保证来自同一采样时刻**。反馈约为 20 Hz，受任务调度与 DMA 忙状态影响，不是硬实时周期。

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

BasicComponents 子模块地址使用 GitHub SSH，首次获取需要相应的 SSH 访问权限。依赖包含 PIDMotor、CRC、StaticArena、UartRxSync 和 utils；Watchdog、DMABuffer 由组件依赖引入。

`cmake/wtr_modules.cmake` 和组件 `CMakeLists.txt` 由 cpkg 管理，不要手工维护另一套组件链接规则。

### Debug 构建

```text
cmake --preset Debug
cmake --build --preset Debug
```

输出为 `build/Debug/Motor_test.elf` 和 `Motor_test.map`。Release 使用同名 `Release` 配置预设与构建预设。

本工程没有统一烧录脚本；使用与实际硬件匹配的 ST-Link 调试器或 STM32CubeProgrammer 将 ELF 下载到目标芯片。

## 上位机联调

配套 HTML 上位机位于本机独立目录 `D:\omp\motor test debug`，不属于本固件 Git 仓库。其运行方式以该目录 README 为准。

1. USB-UART 使用 3.3 V 电平并共地，适配器 RX 接 PC10，TX 接 PC11；不可直接接 RS-232 电平。
2. 上电保持无载，等待校准和首个反馈。串口不能同时被浏览器、串口助手等多个程序占用。
3. 发送目标电压，先从低电压验证实测输出，再逐步调整。
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
| 反馈值长时间不变 | 查看 `Update()`/HAL 状态及真实采样链路；当前协议不携带测量有效性或数据龄期 |
| 添加业务源文件 | 放入 `UserCode` 后重新执行 CMake 配置；源码通过目录扫描加入目标 |

板级配置以 `.ioc` 为准，通过 CubeMX 修改；业务逻辑放在 `UserCode`。避免在 CubeMX 自动生成区直接添加不可再生的代码。

## 参数存储

`STM32G431xx_FLASH.ld` 将程序 Flash 限制为 124 KB，末尾 `0x0801F000～0x0801FFFF` 的两个 2 KB 页预留给参数，不放入固件加载段。链接符号 `__torque_parameters_start` 和 `__torque_parameters_size` 分别表示区域起始地址和大小。

`torque::ParameterStorage` 保存 `Kp/Ki/Kd` 和低通截止频率（Hz），不保存运行目标。调用方传入上述链接区域；存储实现还会核对区域确为芯片 Flash 的末尾两页。两页交替写入，CRC-16/CCITT-FALSE 校验记录，最后写提交标记，读回比对后返回结果。加载选择最新有效记录，损坏或未完成的新记录不会覆盖上一份有效参数；没有有效记录时返回 `NotFound`，不伪造默认参数。

保存是同步操作，必须由单个任务独占 Flash。调用方必须先实际停止输出，并禁止其他路径在保存期间重新启动，才能传入 `output_stopped=true`；零目标本身不是输出已停止的证明。传入 false 时不会擦写。擦除、编程、读回和重新加锁失败均有独立状态；保存失败后不能假定 Flash 已重新锁定或输出可以恢复。存储层只检查数值有限、PID 增益非负及截止频率大于零，应用仍须按实际采样率检查截止频率及其他控制边界。

这些恢复保证针对可读的有效 Flash 记录，不代表电源异常期间的硬件可靠性认证。主机测试未模拟 Flash ECC/NMI、低电压写入或真实器件寿命；全片擦除、读保护变更或烧录器擦除参数页仍会丢失参数。当前应用尚未调用该模块，现有串口协议也尚未接入保存命令，不能据此认定上位机保存按钮已可用。

主机回归需要完整支持 C++17 的编译器（旧版 MinGW GCC 6.3 不满足）；例如在 WSL 的工程目录运行：

```text
g++ -std=c++17 -Wall -Wextra -Werror -Itests/parameters -IUserCode/Parameters -IModules/BasicComponents/libs/utils/crc UserCode/Parameters/ParameterStorage.cpp tests/parameters/storage.cpp -o build/parameter-storage-test
./build/parameter-storage-test
```

## 验证范围

- `tests/ads1220`：校准、配置失败恢复、新旧数据区分、单次/连续转换、监测换算和数值边界。
- `tests/host_communication`：黄金帧、CRC 损坏、目标范围、IT/DMA 接收、错位同步和发送缓冲生命周期。主机编译运行命令见通信模块 README。
- `tests/parameters`：重启后恢复、逐位记录损坏回退、运行中拒绝保存、非法参数与分区、每个写入步骤中断、HAL 错误及保存代数回绕。

这些是主机模拟测试，不是实板或功率电路验证。工程没有统一 `ctest` 入口，构建成功也不代表传感器标定、硬件保护及闭环参数已经适用于实际设备。
