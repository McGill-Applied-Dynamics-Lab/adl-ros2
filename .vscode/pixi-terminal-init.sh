[ -f "$HOME/.bashrc" ] && . "$HOME/.bashrc"

_strip_opt_ros() {
    local var="$1" out=""
    local IFS=":"
    for p in ${!var}; do
        case "$p" in
            /opt/ros/*) ;;
            *) out="${out:+$out:}$p" ;;
        esac
    done
    export "$var=$out"
}
for v in PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH LD_LIBRARY_PATH PATH PKG_CONFIG_PATH; do
    [ -n "${!v}" ] && _strip_opt_ros "$v"
done
unset -f _strip_opt_ros
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION AMENT_CURRENT_PREFIX

eval "$(pixi shell-hook --environment humble --manifest-path "${PIXI_MANIFEST:-$HOME/Documents/adl-ros2/pixi.toml}" 2>/dev/null)"
