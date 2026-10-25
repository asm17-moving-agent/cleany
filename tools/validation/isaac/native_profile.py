"""Bounded Carbonite CPU trace of native physics during actual navigation."""
from __future__ import annotations

import json
from pathlib import Path
import time


class NativeProfiler:
    def __init__(self, output: Path, duration_s: float = 3.) -> None:
        import carb.profiler
        import carb.settings
        self.output = output
        self.duration = duration_s
        self.profiler = carb.profiler.acquire_profiler_interface(plugin_name="carb.profiler-cpu.plugin")
        self.monitor = carb.profiler.acquire_profile_monitor_interface()
        self.settings = carb.settings.get_settings()
        self.active = self.done = False
        self.started = None
        self.steps = 0
        self.metadata = {'status': 'waiting_for_motion', 'duration_s': duration_s}
        self.save()

    def save(self) -> None:
        (self.output/'native-profile.json').write_text(json.dumps(self.metadata, indent=2)+'\n')

    def tick(self, requested) -> None:
        if self.done:
            return
        if not self.active and max(abs(float(v)) for v in requested) > .02:
            self.started = time.monotonic()
            self.metadata.update(status='capturing', wall_start=time.time())
            self.settings.set_string('/plugins/carb.profiler-cpu.plugin/filePath', str(self.output/'native-trace.json.gz'))
            self.profiler.set_capture_mask(0xffffffffffffffff)
            self.settings.set_bool('/plugins/carb.profiler-cpu.plugin/saveProfile', True)
            self.active = True
            self.save()
        elif self.active and time.monotonic()-self.started >= self.duration:
            self.close()

    def begin_step(self) -> None:
        if self.active:
            self.profiler.begin(1, 'Cleany::PhysicsStep')

    def end_step(self) -> None:
        if self.active:
            self.profiler.end(1)
            self.monitor.mark_frame_end()
            self.steps += 1

    def close(self) -> None:
        if self.active:
            self.monitor.mark_frame_end()
            events = self.monitor.get_last_profile_events()
            snapshot = {str(tid): list(events.get_profile_events(tid)) for tid in events.get_profile_thread_ids()}
            (self.output/'native-last-frame.json').write_text(json.dumps(snapshot, indent=2)+'\n')
            self.settings.set_bool('/plugins/carb.profiler-cpu.plugin/saveProfile', False)
            self.profiler.set_capture_mask(0)
            self.metadata.update(status='complete', wall_end=time.time(), steps=self.steps,
                                 main_thread_id=events.get_main_thread_id())
            self.active = False
            self.done = True
            self.save()
