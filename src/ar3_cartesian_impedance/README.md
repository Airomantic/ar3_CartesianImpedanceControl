# AR3 实时笛卡尔阻抗

本包以 `src/xCoreSDK-0.7.1.ar_10/librokae-v0.7.1.ar_10/example/rt/cartesian_impedance_control.cpp` 为接口依据，使用 AR 专用 `rokae::ArRobot`、`RtCommand` 和 `RtControllerMode::cartesianImpedance`。原来的 NRT `fcInit/fcStart`、`moveAppend/MoveL` 分段执行方案已从本包移除，不保留非实时运行开关。

编译并链接的仍是厂家原版 AR SDK `lib/Linux/x86_64/libxCoreSDK.a` 与 `libxMateModel.a`；不修改厂家库，不使用旧 `rokae_hardware/sdk/lib`，也不启动旧七关节驱动或独立 RL 工程。厂家的示例包含预先移动到固定关节姿态的演示代码，本包不执行该段动作，接管以现场实测 TCP 为起点。

## 控制链路

```text
戴盟 MessageBridge /target_robot/right_ee/target + /clutch/right
    → Python 遥操桥，100 Hz 最新目标（原有侧装方向映射）
    → /ar3/cartesian_impedance/command
    → C++ 最新目标交换区
    → SDK 1 ms CartesianPosition 回调
    → 实时笛卡尔阻抗
```

每周期消费最新目标，不等待上一段到位、不累积轨迹队列。回调只读取实时反馈、检查边界和超时、生成连续位姿；ROS 发布、服务、配置 RPC、日志由回调外处理。实时状态的测量时间只在收到新数据包时更新，重复读取 SDK 缓存不能制造新状态心跳。控制器反馈为 SDK 1 ms 数据流，ROS 状态发布及录制是降采样。

实时流不等于将任意位姿跳变原样执行。默认保留 40 mm/s 线速度、0.15 rad/s 角速度、加速度、工作范围和跟踪误差约束。Python 将手柄映射目标限制在实测 TCP 附近 40 mm/0.05 rad，再限制指令变化速度；SDK 回调继续限制每周期变化。平移领先按三维距离计算，前后、左右、上下使用相同限值，斜向总领先也不超过 40 mm。这一目标限制不等于实际跟踪误差的停机界限：反馈更新、柔顺位移或外力可能使已发出的目标与实测 TCP 暂时相差更多。Python 的 `max_tracking_error_m=0.05`、`max_tracking_error_rad=0.15` 用于跟踪误差停止检查，与 SDK 节点的 50 mm/0.15 rad 限值一致；SDK 自身的检查始终独立执行。目标领先限值必须不大于跟踪停止限值，越界日志打印平移误差、旋转误差及两者上限。SDK 调度周期为 1 ms，不代表普通 Linux 主机已通过硬实时性能认证。

平移参数需同时配置：Python `max_translation_lead_m=0.04`、`max_tracking_error_m=0.05`、`max_linear_speed_m_s=0.04`；SDK `rt.yaml` 的 `max_translation_step_m=0.05`、`speed_mm_s=40` 和 `max_velocity` 前三项 `0.04`。SDK 参数默认值也保持一致。线加速度仍为 0.1 m/s²；500 N/m 刚度与 40 mm 位移误差对应约 20 N 的位置弹簧项，不代表实际总接触力。力超限、反馈/指令超时及工作范围保护继续生效，松键保持与再次按键重新对齐的行为不变。

K 和期望 wrench 在 `startMove()` 前配置，一个 RT 会话内固定；会话内改变它们会拒绝并停止，完成真正停止后重新接管才可配置新值。正常松键保持和再次按键沿用同一 RT 会话，不重配 K/期望力，也不重置 SDK 的工作范围原点。头文件要求先完成参数配置再开始运动；这些 setter 使用 RPC，不是每周期控制包字段。本实现不在 1 ms 回调中调用 setter，也不声称模型的动态 K/力可以按 1 kHz 更新。遥操默认期望 wrench 为零，位置目标配合 K 产生跟随；实测接触力不回灌为期望力。

SDK 提供 `setCartesianImpedanceDamp`，但当前头文件未说明系数单位、范围和默认值，本版不调用、不猜参数。SDK 未提供虚拟质量 M setter。wire 中 D/M 均保持零，表示未指定、未下发，不表示物理阻尼或质量为零。工具负载质量与阻抗虚拟质量 M 是不同参数。

## 坐标与测量来源

ROS 位姿和目标使用机器人本体 base 坐标。RT `CartesianPosition.pos` 与 `RtSupportedFields::tcpPose_m` 都是 TCP 相对于 base 的行优先 4×4 矩阵；ROS pose6 使用 xyz[m]、RPY[rad]，`R=Rz*Ry*Rx`。实时目标不再使用 NRT 的工件参考系变换。[官方 SDK 实时接口](https://docs.rokae.com/docs/SDK--0-7-x-AR/cpp/cpp_method/)。

本机右臂基座约定为 +X 朝前、+Y 朝上、+Z 朝右。已有戴盟 TARGET 实测约定为 +X 朝前、+Y 朝左、+Z 朝上，因此保持 `target_direction_rpy_deg=[-90,0,0]`：

| 手柄方向 | SOURCE 增量 | base 目标增量 |
| --- | --- | --- |
| 前/后 | +X / -X | +X / -X |
| 上/下 | +Z / -Z | +Y / -Y |
| 右/左 | -Y / +Y | +Z / -Z |

`setFcCoor` 公开支持 world/tool/path，没有列出 base。本节点选择 world 力控轴，并读取 `baseFrame()` 将 base 下的期望 wrench 旋转到 world；刚度按 `R diag(K) Rᵀ` 变换，只有结果仍可由 SDK 的对角参数表达才接受。侧装 90° 轴置换、三轴等刚度可精确表示；不能表达的各向异性刚度会拒绝，不能丢弃非对角项。沿用已有 TCP 偏置；不修改基座/工具/工件标定。world 模式如何使用 frame 参数的原点，公开 SDK 说明不充分，需要现场确认接触效果。

实测六维反馈读取 **`RtSupportedFields::tauExt_inBase` (`tau_ext_base`) 原始六值**，不在实时循环内调用 `getEndTorque`。该字段明确用 base 轴表达，但厂家头文件对六维数组仅笼统标注外部力矩，未逐项说明力/力矩单位、顺序及作用点。当前训练字段沿用 Fx/Fy/Fz/Tx/Ty/Tz 命名，metadata 明确标记该物理映射待厂家或定向施力验证；不宣称是原始腕部传感器，不宣称与旧 `getEndTorque(tool)` 完全等价，不做未经确认的单位换算或力矩平移。

## ROS 合同与采集

所有接口以 `/ar3/cartesian_impedance/` 为前缀。

| 接口 | 类型 | 含义 |
| --- | --- | --- |
| `state` | Float64MultiArray | 实测 TCP6、RT 外部反馈6、配置 K6、D/M 占位各6 |
| `command` | Float64MultiArray | 最新目标 TCP6、会话期望 wrench6、会话 K6、D/M 占位各6 |
| `accepted_command` | Float64MultiArray | 回调最近返回的平滑 TCP6，加已配置的会话 wrench/K；不是原始遥操目标或运动到位确认 |
| `tcp_pose` | PoseStamped | 当前实测 TCP/base |
| `measured_wrench` | WrenchStamped | 六维 RT 字段按现有接口排列；物理映射验证状态见 metadata |
| `ready` | Bool | 实际反馈新鲜且可用，不表示已使能运动 |
| `status`, `metadata` | String，transient local | 会话状态与精确数据来源 |
| `enable` | SetBool | true 请求接管，之后必须收到新有效命令；false 请求停止 |
| `stop` | Trigger | 异步停止；`stopping:` 只表示请求已接收，`stopped:` 才表示循环结束且确认静止 |

30 维数组顺序为 `[pose6,wrench6,K6,D6,M6]`。metadata 顶层包含 `motion_backend="rt"`、`control_mode="cartesianImpedance"`、`session_fixed_K_wrench=true`，并标记 `tau_ext_base` 来源及未验证的分量解释。采集器等待并校验这些字段；旧 NRT metadata 不符合 RT 合同。

LeRobot 保持 31 维 observation.state、19 维 action，实测示教反馈作为力训练标签，实际控制命令另存 `action.cartesian_command`。不录制七关节 position/torque 或夹爪 position；夹爪原始电流、两指六维反馈和深度图保留。RT 采集使用新目录 `datasets/local/ar3_cartesian_impedance_rt` 和 schema v2，不与旧 NRT 数据续采混合。当前执行器不支持在运动中直接应用模型逐帧变化的 K/期望力。

## 工具负载与启停

启动时读取并缓存控制器 `Toolset.load`、TCP 和基座配置；每次会话在 `startMove()` 前，通过 RT 控制器的 `setLoad()` 下发选定负载。现场修改工具或标定后须重启节点，不在运行中静默切换 TCP。当前按用户要求设置 `load_mass_override_enabled=true`、`load_mass_kg=1.60`，仅覆盖质量；`load_override_enabled=false` 保持完整负载覆盖关闭，质心和惯量继续沿用控制器值，不使用 YAML 中的质心和惯量暂估值。当前工具配置是否已正确标定，不能仅由 setter 成功证明。无需另开 `rokae_driver7` 调旧负载服务。

默认 `enable_motion=false`，只能监测；oneClick 显式允许运动与自动上电。首次运动须离合先释放再按下，接管锚定实测 TCP，新目标才触发会话。SDK 已进入 `active:` 且期望 wrench 为零时，正常释放离合会锁定最后下发的目标位姿，继续以 100 Hz 发送同一命令，使 SDK 的 1 kHz 阻抗回调持续保持；不会调用停止、切换模式或重新上电。保持目标不会追随实测位姿漂移。再次按键重新锚定当前实测 TCP，并等待新的手柄目标后恢复跟随；等待期间继续保持，超时仍停止。阻抗保持仍具有原有柔顺性，不等同于机械锁死。

使能或初始化尚未完成、期望 wrench 非零时松键，仍走真正停止流程。跟随期间目标掉线、任何阶段命令掉线或反馈过期、越界、SDK 异常、力超限、显式停止和退出遥操也会结束当前会话；保持且松键期间忽略不再使用的手柄目标（包括设备松键时重置的无效目标），重新按键后必须收到新的有效目标。SDK 反馈和指令看门狗始终有效。完成停止确认后才允许重新接管，启动失败不自动重试运动。现有停止代码没有主动调用下电，但退出实时阻抗会话也不等同于继续伺服保持，不能仅凭 `Idle(..., false)` 保证末端不下垂。

现场曾在 `Clutch released: holding...` 后约 6 ms 出现 `Measured TCP moved beyond tracking limits`，随后机械臂节点记录 `stop requested`。原因是旧遥操桥误用 1 cm/0.05 rad 的目标限制作为跟踪停机界限。保持阶段现使用上述独立跟踪限值，固定目标不会为了躲避误差而追随实测位置；持续漂移达到 SDK 原有限值仍会停止，不会取消保护或自动增大 K。

`setFinished`/`stopMove` 结束的是运动，控制器可能仍处在 `rci/idle`。AR_10 SDK 的 `operationState()` 截去 `/` 后的子状态，统一报告 `rtControlling`，不能直接等它变成 `idle`。因此停止流程在结束 RT 循环后，显式切 `MotionControlMode::Idle`（`autoPwrOffOnRtChange=false`），确认真正的空闲状态，再恢复 RT 状态监测；恢复监测不调用 `startMove`，不自动重放目标。退出 RT 期间不发布无效的 RT 外力读数。

停止服务既用于请求停止，也用于查询完成状态；重复查询不会重启停止流程或延长期限。若退出模式/空闲确认失败，节点保留 `stop failed:`、实际 `operationState` 和原始原因，锁住重新使能。遥操端也停止自动重试刷屏，仍禁止发命令；`stopping:` 不算完成，必须得到 `stopped:` 确认后重新释放、按下离合。

如果旧进程异常退出后留下 `rtControlling(2)`，普通启动仍拒绝接管。只有确认旧进程已退出、没有其他主机或示教器控制源时，才可单次传入 `recover_stale_rt_on_startup:=true`，并同时保持 `enable_motion:=false`、`auto_power_on:=false`。节点先获取本机同一 IP 的进程锁，持续读取关节位置/速度检查静止，再只执行一次 `Idle(preserve power)`，确认空闲后进入监测；不发送运动目标、不自动上电。这些关节读数只用于恢复检查，不发布或录入训练集。SDK 没有控制权归属查询，静止检查和本机锁不能证明其他主机不存在控制源，因此 oneClick 不默认开启此选项。

恢复检查要求至少 1 秒、20 个样本，整段关节位置峰峰值不超过 20 µrad，同时检查瞬时速度及按时间加权的速度均值。现场静止时读到的速度存在跳动，因此不能只用单个速度样本判静止。这些是诊断阈值，不是厂家承诺的传感器精度或安全认证。SDK 的断连接口还可能停止正在执行的运动，所以即使启动拒绝，也不能把本节点当成多控制源之间的隔离或仲裁工具。

oneClick 的 4.1 等待器同时检查新鲜的 30 维 `state`、RT `metadata` 和节点 `status`。遇到 `fault:`、`stop failed:`、监测恢复失败或依赖任务退出，会立即打印原因并退出等待；不会在机械臂节点已经报错后继续等满 180 秒。修改 oneClick 后，已经打开的旧任务终端不会自动换成新脚本，需要重新启动对应任务。

使能及 `startMove` 前先检查实测六通道绝对限值，运行时同样检查，不自动减去接管时的初值。超限日志包含通道编号、带符号原值、限值和完整六值。例如现场悬空读数 `[0.14,-14.58,0.85,6.04,-0.07,-5.0]` 中第 4 通道 `6.04 > 6`，新版在运动前明确拒绝，而不是先进入 active 再触发停止。默认限值未放宽；这些原始通道的物理分量含义仍待验证，不能仅凭该日志判定夹爪碰撞。

当前 **1.60 kg** 来自用户指定，质心和惯量保持控制器现有值。当前六值不是已确认的原始称重数据，不能将 `14.58/9.81` 直接写成负载，或不加验证地叠加到现有质量。官方自动负载辨识还限制正装，不能直接用于这支侧装右臂。[官方工具负载辨识说明](https://docs.rokae.com/docs/xCore/08-%E7%BC%96%E7%A8%8B/%E5%B7%A5%E5%85%B7%E5%88%97%E8%A1%A8/)。

MessageBridge 不保证离合心跳，默认 `clutch_timeout_s=0` 锁存真实按键消息；收到释放后按上述条件进入固定目标保持或真正停止。跟随/重新接管的目标 watchdog、所有阶段的反馈及指令 watchdog 保留。若设备已确认周期发布，可设置正值增加离合 heartbeat watchdog。

## 构建、启动与离线验证

```bash
./jazzy.sh build --base-paths src/ar3_cartesian_impedance --packages-select ar3_cartesian_impedance \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DPython3_EXECUTABLE=/usr/bin/python3
ctest --test-dir build/ar3_cartesian_impedance --output-on-failure
python3 -m unittest discover -s scripts -p 'test_cartesian_teleop*.py'
```

离线节点启动必须显式禁用硬件连接：

```bash
./jazzy.sh ros2 run ar3_cartesian_impedance cartesian_impedance_node --ros-args -p connect_robot:=false
```

真机入口仍是 LeRobot 目录的 `oneClick.sh`，使用安装后的 `config/rt.yaml` 并传入 `robot_ip=192.168.0.160`、`local_ip=192.168.0.100`。修改源码配置后必须构建安装，再重启 impedance 和 teleop。不要同时运行旧关节驱动、旧阻抗进程或其他控制源。实时通信依赖本机绑定地址和 UDP，能 ping 通不等于实时数据链路已就绪。[官方实时网络说明](https://docs.rokae.com/docs/SDK--0-7-x-AR/cpp/实时模式常见问题/cpp_rt_network_communication/)。

遥操启动日志打印实际方向映射、100 Hz 目标频率和速度上限。接管时应看到 RT 初始化和 active 状态，不应再出现 `fcStart`、`first MoveL submitted` 或逐段到达日志。每秒 `direction:` 比较 SOURCE、MAPPED、COMMAND、ACTUAL 的位移，附上 `RT_EXTERNAL_BASE[SDK6]`；可通过 `direction_diagnostics_hz=0` 关闭。离线测试验证数学、线程间数据交换和启停合同，不代表真机实时性、接触稳定性或反馈分量含义已经验证。
