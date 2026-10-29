# AR3R 末端方向校正

入口 `./jazzy.sh teleop` 运行相邻 `remote/.../teleop_bridge` 中的现有遥操脚本。
`target_direction_rpy_deg` 是只读启动参数，单位为度。Python 类的通用默认值
为 `[0.0, 0.0, 0.0]`；本机 YAML 根据用户确认的右臂侧装姿态配置为
`[-90.0, 0.0, 0.0]`。改变时先松开离合并停止遥操节点，再重新启动。

## 本机侧装方向

用户于 2026-09-15 确认：右臂安装在中间支柱侧面，机器人基座 +X 朝现场前方、
+Y 朝上、+Z 朝右。三向补测显示，原始 TARGET 的手柄前、左、上主要分别为
+X、+Y、+Z。因此从手柄方向基准到机器人基座的增量变换为：

```text
[dx_base, dy_base, dz_base] = [dx_target, dz_target, -dy_target]
C = Rx(-90 degrees)
```

| 手柄动作 | TARGET 主分量 | MAPPED / IK 应改变的基座分量 |
|---|---|---|
| 前 / 后 | +X / -X | +X / -X |
| 上 / 下 | +Z / -Z | +Y / -Y |
| 左 / 右 | +Y / -Y | -Z / +Z |

启动入口加载上述本机 YAML，并默认 `dry_run=true`：

```bash
./jazzy.sh teleop
# 等效的显式配置（命令行优先于 YAML）：
./jazzy.sh teleop -p 'target_direction_rpy_deg:=[-90.0, 0.0, 0.0]'
```

此前 `[180, 0, 0]` 只会反转 Y、Z，不能实现侧装所需的轴交换。不附加旧的
`-p` 参数，否则它会覆盖本机 YAML。需要禁用安装方向转换时，显式使用
`-p 'target_direction_rpy_deg:=[0.0, 0.0, 0.0]'`。

ROS 模型中的 world 仍与机器人基座重合，预览画面的 world +Z 不代表现场
向上；现场向上在本机是基座 +Y。方向修正在目标增量处完成，真实关节反馈、
FK、URDF 的关节轴和 TCP 定义保持原有机器人坐标约定。

另开终端运行只读诊断：

```bash
./jazzy.sh teleop-monitor
```

- `TARGET`：收到的原始目标；本次实测增量按手柄前/左/上对应 +X/+Y/+Z。
- `MAPPED`：方向校正后的目标，送入软启动和 IK；对应话题
  `/ar3r_retargeting/mapped_target`。
- `IK`：通过模型正运动学计算的 IK 预览末端。
- `ACTUAL`：用真实关节反馈和同一 URDF 计算的末端；预览中保持不动是正常的。

`dXYZ` 单位 mm，`dRotXYZ` 是旋转向量，单位度。TARGET 保留输入分量，
MAPPED、IK、ACTUAL 才按机器人基座解释。MessageBridge 的 frame_id 标签相同
不代表原始输入已经完成安装变换。每路数据分别以按离合后的第一帧为参考。
组合腕部限位显示来自
《AR3 产品硬件安装手册-20260608》第 4.1.4 节右臂图，当前为诊断显示，
不等同于控制算法已经加入这一组合约束。

## 校正方式

`C = Rz(yaw) Ry(pitch) Rx(roll)`，三个角均取自启动参数。
在每次离合接管时保存实际末端 `(p_r0, R_r0)`，重置输入滤波器，并等待新的
目标作为输入参考 `(p_s0, R_s0)`。后续映射为：

```text
p_mapped = p_r0 + C (p_source - p_s0)
R_mapped = C (R_source R_s0^T) C^T R_r0
```

因此没有把绝对位置绕机器人基座原点翻转，也没有独立取反关节角。
单位旋转走原有绝对目标路径，保持既有默认行为。

已验证：三轴位移/姿态增量的正负、任意参考姿态、首帧无跳变、接管后等待
新目标、重复离合、参数运行中不可修改，以及隔离 ROS 域中使用真实 URDF
和 Pink IK 的预览。未执行真机动作验证。目标源断更和手腕组合限位仍需分别处理。

侧装回归检查（不启动 ROS 节点、不发送硬件指令）：

```bash
.venv-teleop/bin/python scripts/test_teleop_mount_mapping.py
```
