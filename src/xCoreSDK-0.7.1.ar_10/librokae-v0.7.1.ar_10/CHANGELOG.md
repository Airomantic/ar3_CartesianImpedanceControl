# CHANGELOG

## v0.7.1.ar_10 2026-08-05

* 特性
  * 支持实时模式切换，由用户自主选择是否开启自动下电
* 修复
  * CAN通信int16/uint16数据成员的bug

## v0.7.1.ar_9 2026-07-31

* 特性
  * 设置阻尼的接口
  * ar机型的67轴耦合参数从控制器读取，临时更新模型库，支持67轴耦合的逆解判断
  * 增加设置力控限位阈值的接口
  * 增加接口getJointPos，调用模型库CartToJnt
  * _do_loop线程绑定cpu核心、提高线程优先级
* 修改
  * 模型库：删除设置重力方向，直接读取控制器安装方式

## v0.7.1.ar_8 2026-07-22

* Linux下模型库增加getJointPosNearst接口

## v0.7.1.ar_7 2026-07-21

* 增加xPanelRS485Init、setxPanelRS485Overtime、xPanelCANInit接口适配
* 支持AR3、AR10

## v0.7.1.ar_6 2026-06-18

* 兼容性
  * AR机型专用版本: xCore >= v3.2.1.C224.20260618
* 修复
  * 修改CAN帧ID数值范围以支持扩展帧

## v0.7.1.ar_5 2026-05-25

* 模型库版本号 `v3.2.1, date:C224.20260508`
* 兼容性
  * AR机型专用版本: xCore >= v3.2.1.C224.20260522
* 修复
  * 透传指令接收长度与数据错误的问题
  * ARC逆解多线程问题
* 新增
  * 模型库新增ARC基于3轴冗余度的运动学逆解，即当前输⼊elbow为3轴关节⻆度,
* 移除
  * 废弃臂⻆冗余度概念，废弃calcIk_SearchElbow 、getJointPosSearchPsi遍历臂⻆接⼝
  * 不再需要授权

## v0.7.1.ar_4 2026-03-27

* 兼容性
  * AR机型专用版本: xCore >= v3.2.1.C224.20260327
* 修复
  * CAN收发时序问题
* 新增
  * ServoL功能

## v0.7.0.ar_3.a 2026-03-13

* 修改
  * 更新头文件

## v0.7.0.ar_3 2026-03-12

* 新增
  * AR机型类型
  * CAN接口
  * 增加获取科里奥利力
  * 增加获取质量矩阵和雅各比矩阵的微分
* 删除
  * 遍历臂角求逆解接口

## v0.5.1.ar_10 2025-12-26

* 修改
  * 适配 AR5-5-0.7R-W4Cxxx版本

## v0.5.1.ar_9 2025-12-19

* 新增
  * 增加SR5逆解库

## v0.5.1.ar_6 2025-11-21

* 兼容性
  * xCore >= v3.0.1.6.AR1121 v2
* 新增
  * 增加servoj功能
  * 增加实时模式发送命令接口
  * 添加CAN接收发送接口
  * 新增十字交叉手腕模型库
* 修复
  * 可达长度判断计算错误
  * 机型判断的bug
  * 软限位相关问题
  * rd参数长度&删除多余打印

## v0.5.1.ar_5 2025-10-29

* 修改
  * 增加arm版

## v0.5.1.ar_5 2025-10-23

* 修改
  * 适配ar的末端透传长度

## v0.5.1.ar_4 2025-10-14

* 新增
  * 遍历臂角的逆解接口

## v0.5.1.ar_3 2025-10-14

* 修改
  * ar逆解问题

## v0.5.1.ar_2.a 2025-09-29

* 修改
  * 增加g3log和kdl库

## v0.5.1.ar_2 2025-09-25

* 新增
  * 新增ar机型

## v0.5.1.ar_1 2025-09-19

* 新增
  * ar的Linux模型库

## v0.5.1.ar_0 2025-06-05

* 新增
  * 读写Ethercat从站相关接口，BaseEthercat

## v0.7.1 2026-03-03

* 修复
 * 实时模式关节点位跟随功能优化，处理奇异点问题
 * 设置工具工件信息(setToolInfo, setWobjInfo) 时旋转角度和负载信息设置错误

## v0.7.0 2026-01-16

* 兼容性
  * xCore >= v3.2.0
* 新增
  * ServoJ功能: setServoJoint(), stopServoJoint(), sendCommand()
  * 导出控制器备份exportBackup() 和升级 upgrade() 接口
  * 力控指令: 阻抗力限幅, 阻抗速度限幅, 设置关节力控带宽、摩擦力补偿系数
  * 计算全部逆解 calcAllIkSolutions()
  * 关闭工控机 shutdownSystem()
  * 查询控制器日志增加偏移选项
  * 读取DH参数 getRobotCfg_DHparam()
* 优化
  * 隔离内部引入的spdlog, 避免冲突
  * 实时模式运动延迟问题，增大超时时间上限到20ms
* 修复
  * 末端工具指令兼容新旧固件

## v0.6.0 2025-07-08

* 兼容性
  * xCore >= v3.1.0
* 新增
  * 通过SDK操作RL工程文件
    * 传输RL工程importProject()，传输单个工程文件importFile()
    * 删除工程 removeProject()，删除文件removeFiles()
  * 设置工具工件信息setToolInfo(), setWobjInfo()
  * 设置示教器热插拔setTeachPendantMode()
  * 重启工控机rebootSystem()
  * 设置SDK连接/断开回调函数setConnectionHandler()
  * 获取机器人基本信息(robotInfo)增加MAC地址
  * 设置是否自动忽略转弯区函数setAutoIgnoreZone()
  * 事件监控增加控制器日志 Event::logReporter
  * 优化xMate模型库动力学参数获取，修复getTorqueNoFriction()计算问题
  * 模型库支持XMS5机型（注意:需要升级特殊版本的机型文件）
  * 实时模型控制类增加hasMotionError()
* 修复
  * 非实时运动指令缓存上限放宽到1000，以满足高速短轨迹场景
  * 实时模式MoveL的起点和终点的笛卡尔空间距离小于1e-6时，函数会阻塞的问题
  * 重连机器人后，恢复监听事件
  * 修正实时模式错误位的报错信息
* 移除
  * xMate模型库不支持XMC18和XMC20机型
  * 删除模型库计算力矩函数getTorque()和getTorqueWithFriction()

## v0.5.1 2025-04-25

* 兼容性
  * xCore >= v3.0.2
* 新增
  * 运动指令的速度speed和转弯区zone数据类型改为double
  * TCP连接断开增加日志，日志路径为执行目录下logs；正逆解计算增加日志
* 修复
  * 点位跟随功能用的关节限位和控制器设置的限位不一致问题
  * 网络连接断开后，运动事件监听无法恢复的问题，通过connectToRobot或setEventWatcher可再次设置监听
  * 查询工程工具工件崩溃问题
  * 版本号向下兼容
  * moveReset重置指令技术，避免由于网络不稳定引起的计数错误问题

## v0.5.0 2024-12-31

* 兼容性
  * xCore >= v3.0.1
* 新增
  * 支持控制导轨
    * 带导轨的联动运动；
    * 读写导轨参数 setRailParameter(), getRailParameter()
    * 导轨Jog
  * 笛卡尔空间运动指令可达性校验 checkPath()
  * XMS, XMC机型末端485通信相关 setxPanelRS485(), XPRWModbusRTUReg(), XPRWModbusRTUCoil()
  * 实时模式执行回调函数时，读取数据的超时等待时间可设置
  * 运动停留指令MoveWait，运动指令自定义信息
  * RL工程执行状态事件通知
  * 给定工具工件坐标系下的正逆解计算
  * 无末端按键拖动
* 修复
  * 工业六轴机型本机地址设置问题

## v0.4.1.b 2024-09-03

* 修复
  * 进入协作模式后机器人状态错误问题
* 新增
  * 写寄存器数组

## v0.4.1.a 2024-08-16

* 修复
  * 移除Linux平台对std::filesystem的依赖

## v0.4.1 2024-07-02

* 兼容性
  * xCore >= v2.2.1, 部分新增特性需要xCore >= v2.2.2
* 新增
  * 运动指令速度细化，增加关节速度百分比jointSpeed 和空间旋转速度rotSpeed (需要xCore版本v2.2.2)
  * 增加三个实时状态数据
  * 急停复位接口recoverState()
  * SDK日志可通过本地文件配置
* 修复 (需要xCore版本v2.2.2)
  * Jog步长在示教器显示的问题
  * 运动缓存错误码没有传出的问题
  * 碰撞检测负载设置问题

## v0.4.0 2024-04-15

* 兼容性
  * xCore >= v2.2
* 新增
  * 支持CR5轴机型
  * 所有非实时力控指令
  * 力矩传感器标定calibrateForceSensor()
  * SDK执行日志
  * 加速度读写接口getAcceleration(), adjustAcceleration()
  * 末端按键读取 getKeypadState()
  * 设置基坐标系 setBaseFrame()
  * 打开关闭三种奇异规避方式
  * 其它新增: MoveSP指令增加偏移项; 全圆指令增加旋转类型, 等
* 修复
  * 非实时接口多线程阻塞问题
  * 实时模式不能控制多台机器人问题
  * UDP端口设置导致的状态数据接收问题
  * 其它已知问题
* 变更
  * 使用协作CR和SR机型，并且程序中MoveJCommand用到了confData，需要在程序中加 setDefaultConfOpt(true), 让confData生效;
  * 错误码和异常信息语言根据用户PC系统语言设置而定，中文返回中文信息，非中文返回英文信息;
  * 拖动回放replayPath()，由调用完立即开始运动，改为需要moveStart()才开始运动，并且可以和其它运动指令一起下发;
  * 通过setToolset()函数设置的工具工件组，优化为右上角的工具工件显示"toolx", "wobjx"，并且状态监控里看到的位姿会同步更新。

## v0.3.4 2023-11-02

* 兼容性  
  * xCore >= v2.1.0.15 (三位发布号v2.1.0)
  * 增加支持aarch64-linux-gnu (gcc version 7.5.0)
  * xMateModel模型库增加支持Windows-64 Debug版本
* 新增
  * Robot类的默认构造，带IP地址参数的连接机器人接口connectToRobot(remoteIP, localIP)
  * 读取设置软限位接口getSoftLimit(), setSoftLimit()
  * 协作机器人读取末端力矩接口getEndTorque()
  * 碰撞检测触发行为增加柔顺停止(StopLevel::suppleStop)和柔顺度选项
  * xMateCR和xMateSR机型奇异规避相关接口: 奇异规避&平行基座Jog，设置奇异规避模式运动setAvoidSingularity()
  * 非实时运动信息反馈增加点位距离过近的报警信息(EventInfoKey::MoveExecution::Remark)
  * 工具/工件/基坐标系标定接口calibrateFrame()
  * 螺旋线运动指令MoveSPCommand
  * 设置是否严格遵循轴配置数据接口setDefaultConfOpt()
  * 模型库支持所有已知协作机型，增加支持XMS3, XMS4, XMC18, XMC20
* 修复&优化
  * 在工具工件坐标系下Jog机器人，由原来的使用RobotAssist右上角选择的工具工件，改为使用通过setToolset()设置的工具工件坐标系
  * 全圆指令MoveCFCommand参数全圆执行角度(angle)单位由度数改为弧度
  * 删除ForceControlFrameType枚举类，setFcCoor()中坐标系类型参数改为FrameType
  * 修复实时模式运动中发生异常（如急停）不能恢复、只能重新运行的问题
  * 移除模型库对glog的依赖
  * 修复计算逆解接口不回复，或者用时较长的问题
  * 修复因网络异常没有处理造成的实时收发线程可能崩溃的问题
  * 其它已知问题

## v0.3.3 2023-08-23

* 兼容性
  * 增加Linux下不依赖模型库的xCoreSDK.so, 可用于编译动态库
* 修复&优化
  * 通过moveAppend()发送运动指令可能不执行的问题
  * 实时模式轴空间阻抗控制，放宽阻抗系数上限到3000, 300
  * RL工程相关接口没有检查模式的问题

## v0.3.2 2023-07-04

* 兼容性
  * xCore >= v2.0.1
  * xMateModel模型库支持Linux x86_64; 及Windows Release编译类型
* 新增
  * 运动指令MoveCF
  * 设置和打开关闭碰撞检测接口; 碰撞监测功能
* 修复
  * 实时模式急停后恢复运动的问题; 及其它已知问题

## v0.3.1 2023-05-03

* 兼容性
  * xCore >= v2.0.0.7
  * 增加动态库；xMateModel及相关接口除外，仅支持Linux静态库
* 新增&优化
  * 非实时运动指令增加暂停、继续、获取运动指令执行信息功能，新增接口moveStart(), moveAppend(); 及设置事件回调setEventWatcher();
  * 笛卡尔目标点增加偏移选项(Offs/Reltool)
  * 增加全局调整运动速率接口adjustSpeedOnline()

## v0.3.0 2023-03-07

* 兼容性
  * xCore >= v2.0.0.1
* 新增&优化
  * xCore版本匹配检查
  * 工业六轴机型支持实时模式位置控制
  * 支持不开启实时模式的情况下读取状态数据
  * 更新Eigen库
  * 增加设置IO仿真模式和设置DI
  * ”XMate“开头的类改成为”xMate"开头
  * projectPointToMain()改为ppToMain()
* 修复
  * loadProject()加载工程增加工程是否存在的检查

## v0.2.8 2023-02-15


* 新增&优化
  * 合并CartesianPosition & CartesianPose
  * 删除append()&executeCommands()
  * 增加FollowPosition目标跟随
  * xMateModel增加适配CR&SR机型

本文档移除了更早的历史版本的变更记录
