from functools import wraps


class SerializedInference:
    def __init__(self, adapter, lock):
        self._adapter, self._lock = adapter, lock

    def __getattr__(self, name):
        member = getattr(self._adapter, name)
        if not callable(member):
            return member
        @wraps(member)
        def call(*args, **kwargs):
            with self._lock:
                return member(*args, **kwargs)
        return call
