"""Health acquisition and freshness only; monitors never change system state."""
from .models import Availability, HealthReport


class HealthMonitoring:
    def __init__(self, sources: dict, started_at: float):
        self.sources = sources
        self.started_at = started_at
        self.reports: dict[str, HealthReport] = {}
        self.good_since: dict[str, float] = {}
        self.bad_since: dict[str, float] = {}
        self.verified: set[str] = set()  # Sources that have reported AVAILABLE at least once.

    def observe(self, report: HealthReport) -> None:
        if report.source not in self.sources:
            raise ValueError(f'unconfigured health source {report.source}')
        previous = self.reports.get(report.source)
        if previous and report.observed_at <= previous.observed_at:
            return
        self.reports[report.source] = report
        if report.status == Availability.AVAILABLE:
            self.verified.add(report.source)
            # A gap must break healthy hysteresis even when both samples are good.
            if previous is None or previous.status != report.status or (
                report.observed_at - previous.observed_at > self.sources[report.source]['stale_s']
            ):
                self.good_since[report.source] = report.observed_at
            self.bad_since.pop(report.source, None)
        else:
            self.good_since.pop(report.source, None)
            self.bad_since.setdefault(report.source, report.observed_at)

    def status(self, source: str, now: float) -> Availability:
        report = self.reports.get(source)
        if report is None or now - report.observed_at > self.sources[source]['stale_s']:
            self.good_since.pop(source, None)
            self.bad_since.setdefault(source, self.started_at if report is None else
                                      report.observed_at + self.sources[source]['stale_s'])
            return Availability.UNKNOWN
        return report.status

    def healthy(self, source: str, now: float, after: float | None = None) -> bool:
        report = self.reports.get(source)
        return (self.status(source, now) == Availability.AVAILABLE and
                now - max(self.good_since[source], after if after is not None else self.good_since[source])
                >= self.sources[source]['clear_s'] and
                (after is None or (report is not None and report.observed_at > after)))
