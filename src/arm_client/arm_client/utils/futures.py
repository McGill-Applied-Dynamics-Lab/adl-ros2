"""Blocking waits on rclpy futures that do not starve the executor thread."""

import threading

from rclpy.task import Future


def wait_for_future(future: Future, timeout_sec: float | None = None):
    """Block until `future` is done and return its result.

    The future must be completed by an executor spinning in another thread (e.g. `Robot`'s spin
    thread). Waiting on an event releases the GIL; a `while not future.done()` loop holds it and
    caps the spin thread at one callback per GIL switch interval (~170 Hz instead of 1 kHz).

    Args:
        future: The future to wait on.
        timeout_sec: Maximum time to wait; None waits forever.

    Returns:
        The future's result.

    Raises:
        TimeoutError: If the future is not done within `timeout_sec`.
    """
    done = threading.Event()
    future.add_done_callback(lambda _: done.set())
    if not done.wait(timeout_sec):
        raise TimeoutError(f"Future not done after {timeout_sec} s")
    return future.result()
