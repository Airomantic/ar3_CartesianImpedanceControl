#!/usr/bin/env bash
# jazzy.sh — 在干净的 ROS2 Jazzy 环境中编译/运行 ar3_ws
#
# 为什么需要它:
#   ~/.bashrc 里 source 了底盘(xiaoyi_chassis_jazzy)、导航(humble_localization_nav2)、
#   fast_ws 等外部工作空间, 它们的路径会累积进 AMENT_PREFIX_PATH, 污染 Jazzy 的
#   rosidl 工具链, 导致 rokae_msgs 编译时段错误(core dump).
#   本脚本用 env -i + --noprofile/--norc 启动一个完全干净的 bash, 只 source
#   /opt/ros/jazzy 和本工作空间 install, 彻底避开污染.
#   继承调用终端 export 的 ROS_DOMAIN_ID（通常来自 ~/.bashrc），未设置时使用 0。
#   域号隔离与工作空间依赖隔离是两件事，保留域号不会引入其它工作空间。
#
# 用法:
#   ./jazzy.sh build [--packages-select rokae_hardware ...]            编译
#   ./jazzy.sh launch rokae_hardware rokae_moveit_launch.py robot_type:=AR5L use_fake_hardware:=true
#   ./jazzy.sh ros2  topic list / control list_controllers / ...        任意 ros2 命令
#   ./jazzy.sh teleop                                                   启动 AR3R 遥操桥（默认只预览）
#   ./jazzy.sh teleop-monitor                                           只读比较目标/IK/实际末端位移
#   ./jazzy.sh shell                                                    进入干净交互式 shell
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DISTRO=jazzy

# 在干净环境中执行一条命令; $1=prelude(source 语句), 其余=要 exec 的命令及参数
# 用位置参数法把命令透传给内层 bash, 避免引号/空格被二次解析
exec_clean() {
  local prelude="$1"; shift
  env -i HOME="$HOME" PATH=/usr/bin:/bin:/usr/local/bin ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
      LANG="${LANG:-C.UTF-8}" TERM="${TERM:-xterm-256color}" \
      DISPLAY="${DISPLAY:-}" XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}" \
    bash --noprofile --norc -c "$prelude"'exec "$@"' _ "$@"
}

# 运行类前置: source ROS + 本工作空间 install
prelude_run=". /opt/ros/$DISTRO/setup.bash; cd '$WS_DIR'; . install/local_setup.bash; "
# 编译类前置: 只 source ROS (不 source 自己的 install, 它正在编译)
# 默认限 make 并行到 -j4: Jazzy 的 rosidl 生成器(empy4)在全核 -j24 满载下偶发段错误
#   (rokae_msgs 生成 external_force.hpp 时 core dump)。降并行即解。
#   若仍崩, 用 MAKEFLAGS=-j2 ./jazzy.sh build 进一步降低。
prelude_build=". /opt/ros/$DISTRO/setup.bash; cd '$WS_DIR'; export MAKEFLAGS='${MAKEFLAGS:--j2}'; " #  Jazzy 的 rosidl 生成器（empy4）在全核 -j24 满载下偶发段错误，设置为4 并行时都报了错

case "${1:-help}" in
  build)
    shift
    exec_clean "$prelude_build" colcon build "$@"
    ;;
  launch)
    shift
    exec_clean "$prelude_run" ros2 launch "$@"
    ;;
  ros2)
    shift
    exec_clean "$prelude_run" ros2 "$@"
    ;;
  impedance-wait-state)
    shift
    exec_clean "$prelude_run" /usr/bin/python3 \
      "$WS_DIR/scripts/ar3_wait_cartesian_state.py" "$@"
    ;;
  impedance-capture-home)
    shift
    exec_clean "$prelude_run" /usr/bin/python3 \
      "$WS_DIR/scripts/ar3_capture_home.py" "$@"
    ;;
  impedance-teleop)
    shift
    exec 9>"$WS_DIR/.venv-teleop/impedance-teleop-domain-${ROS_DOMAIN_ID:-0}.lock"
    flock -n 9 || { echo "笛卡尔阻抗遥操已在当前 ROS 域运行" >&2; exit 1; }
    exec_clean "$prelude_run" /usr/bin/python3 \
      "$WS_DIR/scripts/ar3_cartesian_impedance_teleop.py" --ros-args "$@"
    ;;
  teleop)
    shift
    teleop_dir="$WS_DIR/../remote/pp_ubuntu24.04_V1.1.2.38_202606301404_release_ros2_x86_jazzy/teleop_bridge"
    teleop_python="$WS_DIR/.venv-teleop/bin/python"
    if [[ ! -x "$teleop_python" || ! -f "$teleop_dir/ar3r_cartesian_retargeting.py" || ! -f "$teleop_dir/ar3r_cartesian_retargeting.yaml" ]]; then
      echo "缺少遥操虚拟环境或脚本，请检查 .venv-teleop 和 $teleop_dir" >&2
      exit 1
    fi
    # Keep the lock for the lifetime of this launcher and its child. Separate
    # ROS domains may run independently; direct Python launches bypass this lock.
    exec 9>"$WS_DIR/.venv-teleop/teleop-domain-${ROS_DOMAIN_ID:-0}.lock"
    if ! flock -n 9; then
      echo "当前工作空间的 ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} 已有 jazzy.sh teleop 在运行，请使用原终端。" >&2
      exit 1
    fi
    exec_clean "$prelude_run" "$teleop_python" \
      "$teleop_dir/ar3r_cartesian_retargeting.py" --ros-args \
      --params-file "$teleop_dir/ar3r_cartesian_retargeting.yaml" \
      -p dry_run:=true "$@"
    ;;
  teleop-monitor)
    shift
    exec_clean "$prelude_run" "$WS_DIR/.venv-teleop/bin/python" \
      "$WS_DIR/scripts/teleop_direction_monitor.py" "$@"
    ;;
  shell)
    rc_file="$(mktemp)"
    printf '. /opt/ros/%s/setup.bash\ncd "%s"\n. install/setup.bash\n' "$DISTRO" "$WS_DIR" > "$rc_file"
    env -i HOME="$HOME" PATH=/usr/bin:/bin:/usr/local/bin ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
        LANG="${LANG:-C.UTF-8}" TERM="${TERM:-xterm-256color}" \
        DISPLAY="${DISPLAY:-}" XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}" \
      bash --noprofile --rcfile "$rc_file" -i
    rm -f "$rc_file"
    ;;
  help|*)
    cat <<EOF
用法: $0 <子命令> [参数...]
所有入口继承终端的 ROS_DOMAIN_ID（未设置时为 0），只加载 Jazzy 和本工作空间的依赖。
  build    colcon build              (只 source /opt/ros/jazzy)
  launch   ros2 launch               (source jazzy + 本工作空间 install)
  ros2     任意 ros2 命令
  teleop   启动 AR3R 重定向节点，默认 dry_run=true（只预览，不发送真机指令）
  teleop-monitor  只读显示目标、IK 预览和实际末端的位移及第七轴限位余量
  impedance-teleop  原生实时笛卡尔阻抗遥操（100 Hz 目标，SDK 1 kHz，默认 dry_run=true）
  impedance-wait-state  只读等待实际 RT 状态；SDK fault/stop failed 或依赖退出时立即报错
  impedance-capture-home  只读保存当前静止 TCP 为固定原点；重新定义时显式传 --overwrite
  shell    进入干净的交互式 bash
示例:
  $0 build --packages-select rokae_hardware
  $0 launch rokae_hardware rokae_moveit_launch.py robot_type:=AR5L use_fake_hardware:=true
  $0 ros2 control list_controllers
  $0 teleop
EOF
    exit 1
    ;;
esac
