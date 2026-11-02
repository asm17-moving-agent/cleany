"""Compose existing navigation and a separately stopped desk approach operation."""
from .result import ModuleResult, ResultStatus, FailureCode


class StagedNavigator:
    """Preserve the navigation port contract through staging and fine approach."""
    def __init__(self, navigation, approach):
        self.navigation, self.approach = navigation, approach
        self.operation = self.approach_operation = None
        self.target = None
        self.result = None
        self.cancelled = False

    def ready(self):
        ok, reason = self.navigation.ready()
        return self.approach.ready() if ok else (ok, reason)

    def stopped(self):
        return self.navigation.stopped() and self.approach.stopped()

    def maintain(self):
        self.navigation.maintain()
        self.approach.maintain()

    def start(self, target):
        if not self.ready()[0] or not self.stopped():
            raise RuntimeError('staged navigation unavailable')
        self.operation = self.navigation.start(target)
        self.target, self.approach_operation, self.result, self.cancelled = target, None, None, False
        return self.operation

    def poll(self, operation_id):
        if operation_id != self.operation:
            return None
        if self.result is not None:
            return self.result
        if self.approach_operation is None:
            result = self.navigation.poll(operation_id)
            if result is None:
                return None
            if self.cancelled:
                self.result = ModuleResult(False, ResultStatus.CANCELLED)
                return self.result
            if not result.ok or self.target == 'home' or self.cancelled:
                self.result = result
                return result
            try:
                self.approach_operation = self.approach.start(self.target)
            except RuntimeError as exc:
                self.result = ModuleResult.failed(FailureCode.NAVIGATION_FAIL, message=str(exc))
                return self.result
        result = self.approach.poll(self.approach_operation)
        if result is not None:
            self.result = result
        return result

    def cancel(self, operation_id):
        if operation_id != self.operation:
            return
        self.cancelled = True
        self.navigation.cancel(operation_id)
        if self.approach_operation is not None:
            self.approach.cancel(self.approach_operation)

    def localization_snapshot(self):
        return self.navigation.localization_snapshot()
