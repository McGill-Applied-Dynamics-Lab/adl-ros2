# my_debug_utils.py

import os
import debugpy


def wait_for_debugger(node_name, default_port=5678):
    """
    Wait for the debugger to attach. Only wait if the node_name matches the environment variable DEBUG_NODE.
    """
    if node_name is None:
        raise ValueError("node_name must be provided to wait for debugger.")

    debug_node = os.getenv("DEBUG_NODE", "")

    if node_name != debug_node:
        return  # Not the node we want to debug

    port = int(os.getenv("DEBUG_PORT", default_port))

    print(f"[{node_name}] Waiting for debugger to attach on port {port}...")
    try:
        debugpy.listen(("0.0.0.0", port))
    except Exception as e:
        print(f"[{node_name}] Failed to bind debugger on port {port}: {e}")
        return
    debugpy.wait_for_client()
    print(f"[{node_name}] Debugger attached.")
