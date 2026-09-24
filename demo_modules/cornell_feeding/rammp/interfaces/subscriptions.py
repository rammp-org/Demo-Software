"""Create and destroy subscriptions safely under a MultiThreadedExecutor.

On Humble, destroying a subscription while the executor has already queued a
handler for it makes that handler raise ``InvalidHandle`` from its ``take``,
which propagates out of ``executor.spin()`` and kills the node. The safe order
is: stop the executor from creating new handlers for the subscription (its
callback group answers ``can_execute`` with False, so it leaves the wait set),
wait until no handler is in flight, then destroy.
"""

import time

from rclpy.callback_groups import ReentrantCallbackGroup


class PausableCallbackGroup(ReentrantCallbackGroup):
    """Reentrant group whose members drop out of the executor's wait set when paused."""

    def __init__(self):
        super().__init__()
        self.active = True

    def can_execute(self, entity):
        return self.active and super().can_execute(entity)


def destroy_subscriptions(node, group, subscriptions, timeout_sec=2.0):
    """Pause ``group``, wait for its in-flight handlers, then destroy ``subscriptions``."""
    group.active = False
    deadline = time.time() + timeout_sec
    while time.time() < deadline and any(
        getattr(sub, "_executor_event", False) for sub in subscriptions
    ):
        time.sleep(0.005)
    for sub in subscriptions:
        node.destroy_subscription(sub)
