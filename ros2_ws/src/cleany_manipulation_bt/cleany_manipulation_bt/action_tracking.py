"""Track accepted and late-accepted goals until terminal controller results."""
from action_msgs.msg import GoalStatus


class TrackedClient:
    def __init__(self, owner, client):
        self.owner, self.client = owner, client
        self.submissions, self.handles = [], []
        self.close_requested = False
        self.destroyed = False

    def __getattr__(self, name):
        return getattr(self.client, name)

    def send_goal_async(self, goal, **kwargs):
        self.owner.check_execution()
        context = self.owner.execution_context
        future = self.client.send_goal_async(goal, **kwargs)
        self.owner.submissions.append(future)
        self.submissions.append(future)

        def accepted(response):
            if response.cancelled() or response.exception() is not None:
                return
            handle = response.result()
            if handle is not None and handle.accepted:
                terminal = handle.get_result_async()
                self.handles.append((handle, terminal))
                self.owner.handles.append((handle, terminal))
                terminal.add_done_callback(lambda _: self._close_if_terminal())
                if context is not None and context.abort.is_set():
                    handle.cancel_goal_async()
            self._close_if_terminal()

        future.add_done_callback(accepted)
        return future

    def destroy(self):
        # Destroying a client before a late acknowledgement/result arrives
        # would discard the evidence needed by StopAndAssess.
        self.close_requested = True
        self._close_if_terminal()

    def _close_if_terminal(self):
        if self.close_requested and not self.destroyed and quiescent(self.submissions, self.handles):
            self.client.destroy()
            self.destroyed = True

    def reset_completed(self):
        """Discard this client's old bookkeeping only after all results are known."""
        if quiescent(self.submissions, self.handles):
            self.submissions.clear()
            self.handles.clear()


def quiescent(submissions, handles):
    def valid(future):
        return future.done() and not future.cancelled() and future.exception() is None
    terminal = {GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED}
    return (all(valid(f) and f.result() is not None and
                (not f.result().accepted or any(h is f.result() for h, _ in handles))
                for f in submissions)
            and all(valid(f) and f.result() is not None and f.result().status in terminal
                    for _, f in handles))
