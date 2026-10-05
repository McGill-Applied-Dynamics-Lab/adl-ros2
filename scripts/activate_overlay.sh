# Sourced by pixi on environment activation (see [activation] in pixi.toml).
# Keep this POSIX sh: pixi may source it from bash or zsh.
#
# Overlays the colcon workspace (install_<distro>/) on top of the ROS underlay
# so `pixi run python ...` sees packages like arm_client. Skipped silently
# before the first build.

_adl_overlay="${PIXI_PROJECT_ROOT:-.}/install_${ROS_DISTRO:-humble}/setup.sh"
if [ -f "$_adl_overlay" ]; then
  . "$_adl_overlay"
fi
unset _adl_overlay
