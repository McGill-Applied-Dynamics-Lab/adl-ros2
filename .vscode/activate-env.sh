#!/usr/bin/env bash
# Loaded by VSCode's "pixi-humble" terminal profile as the bash rcfile.
# Activates the pixi 'humble' environment and sources the ROS 2 overlay
# in-place (no subshell), so the resulting terminal is ready for ROS 2 work.

# Keep user's normal bashrc behavior (aliases, prompt, etc.)
if [ -f "$HOME/.bashrc" ]; then
    # shellcheck disable=SC1091
    source "$HOME/.bashrc"
fi

# Activate the pixi env for this directory without spawning a subshell.
# `shell-hook` prints the activation commands; we eval them.
if command -v pixi >/dev/null 2>&1; then
    _pixi_hook="$(pixi shell-hook -e humble 2>/dev/null)"
    if [ -n "$_pixi_hook" ]; then
        eval "$_pixi_hook"
    fi
    unset _pixi_hook
fi

# Source the colcon overlay so `import arm_client` etc. works.
if [ -f "install_humble/setup.bash" ]; then
    # shellcheck disable=SC1091
    source install_humble/setup.bash
fi
