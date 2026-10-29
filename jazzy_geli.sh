#!/usr/bin/env bash
# jazzy.sh — 在干净的 ROS2 Jazzy 环境中编译/运行 xiaoyi_arm_ws
#
# 为什么需要它:
#   ~/.bashrc 里 source 了底盘(xiaoyi_chassis_jazzy)、导航(humble_localization_nav2)、
#   fast_ws 等外部工作空间, 它们的路径会累积进 AMENT_PREFIX_PATH, 污染 Jazzy 的
#   rosidl 工具链, 导致 rokae_msgs 编译时段错误(core dump).
#   本脚本用 env -i + --noprofile/--norc 启动一个完全干净的 bash, 只 source
#   /opt/ros/jazzy 和本工作空间 install, 彻底避开污染.
#
# 用法:
#   ./jazzy.sh build [--packages-select rokae_hardware ...]            编译
#   ./jazzy.sh launch rokae_hardware rokae_moveit_launch.py robot_type:=AR5L use_fake_hardware:=true
#   ./jazzy.sh ros2  topic list / control list_controllers / ...        任意 ros2 命令
#   ./jazzy.sh shell                                                    进入干净交互式 shell
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DISTRO=jazzy

# 在干净环境中执行一条命令; $1=prelude(source 语句), 其余=要 exec 的命令及参数
# 用位置参数法把命令透传给内层 bash, 避免引号/空格被二次解析
exec_clean() {
  local prelude="$1"; shift
  env -i HOME="$HOME" PATH=/usr/bin:/bin:/usr/local/bin \
      LANG="${LANG:-C.UTF-8}" TERM="${TERM:-xterm-256color}" \
      DISPLAY="${DISPLAY:-}" XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}" \
    bash --noprofile --norc -c "$prelude"'exec "$@"' _ "$@"
}

# 运行类前置: source ROS + 本工作空间 install
prelude_run=". /opt/ros/$DISTRO/setup.bash; cd '$WS_DIR'; . install/setup.bash; "
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
  shell)
    rc_file="$(mktemp)"
    printf '. /opt/ros/%s/setup.bash\ncd "%s"\n. install/setup.bash\n' "$DISTRO" "$WS_DIR" > "$rc_file"
    env -i HOME="$HOME" PATH=/usr/bin:/bin:/usr/local/bin \
        LANG="${LANG:-C.UTF-8}" TERM="${TERM:-xterm-256color}" \
        DISPLAY="${DISPLAY:-}" XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}" \
      bash --noprofile --norc --rcfile "$rc_file" -i
    rm -f "$rc_file"
    ;;
  help|*)
    cat <<EOF
用法: $0 <子命令> [参数...]
  build    colcon build              (只 source /opt/ros/jazzy)
  launch   ros2 launch               (source jazzy + 本工作空间 install)
  ros2     任意 ros2 命令
  shell    进入干净的交互式 bash
示例:
  $0 build --packages-select rokae_hardware
  $0 launch rokae_hardware rokae_moveit_launch.py robot_type:=AR5L use_fake_hardware:=true
  $0 ros2 control list_controllers
EOF
    exit 1
    ;;
esac
