"""Track accepted and late-accepted goals until terminal controller results."""
from action_msgs.msg import GoalStatus
from threading import RLock


class TrackedClient:
    def __init__(self, owner, client, *, owns_client=True):
        self.owner, self.client = owner, client
        self.submissions, self.handles = [], []
        self.close_requested = False
        self.destroyed = False
        self.owns_client = owns_client
        self._close_lock = RLock()

    def __getattr__(self, name):
        return getattr(self.client, name)

    def send_goal_async(self, goal, **kwargs):
        if self.close_requested or self.destroyed:
            raise RuntimeError('Action client lease is closed')
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
        with self._close_lock:
            if self.close_requested and not self.destroyed and quiescent(self.submissions, self.handles):
                if self.owns_client:
                    self.client.destroy()
                self.destroyed = True

    def reset_completed(self):
        """Discard this client's old bookkeeping only after all results are known."""
        if quiescent(self.submissions, self.handles):
            self.submissions.clear()
            self.handles.clear()


class MotionClientPool:
    """Reuse DDS transports with a separate, non-overlapping lease per action.

    Pending acknowledgements and terminal results keep a lease occupied.
    Closing a lease never destroys its shared transport; owner shutdown does.
    """

    def __init__(self, owner, factory):
        self._owner, self._factory = owner, factory
        self._clients, self._leases = {}, {}
        self._lock = RLock()
        self._closed = False

    def acquire(self, action_type, name):
        key = (action_type, name)
        with self._lock:
            if self._closed:
                raise RuntimeError('Motion client pool is closed')
            previous = self._leases.get(key)
            if previous is not None:
                previous._close_if_terminal()
                if not previous.destroyed:
                    raise RuntimeError('Previous motion client lease is not terminal')
            reused = key in self._clients
            if not reused:
                self._clients[key] = self._factory(action_type, name)
            lease = TrackedClient(self._owner, self._clients[key], owns_client=False)
            lease.transport_reused = reused
            self._leases[key] = lease
            return lease

    def close(self):
        """Destroy each transport once, only during owner node teardown."""
        with self._lock:
            if self._closed:
                return
            self._closed = True
            for lease in self._leases.values():
                with lease._close_lock:
                    lease.close_requested = lease.destroyed = True
            clients = list(self._clients.values())
            self._clients.clear()
            self._leases.clear()
        for client in clients:
            client.destroy()


def quiescent(submissions, handles):
    def valid(future):
        return future.done() and not future.cancelled() and future.exception() is None
    terminal = {GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED}
    return (all(valid(f) and f.result() is not None and
                (not f.result().accepted or any(h is f.result() for h, _ in handles))
                for f in submissions)
            and all(valid(f) and f.result() is not None and f.result().status in terminal
                    for _, f in handles))
