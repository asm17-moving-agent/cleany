"""Single-owner session lifecycle, independent of ROS executors and HTTP."""
import copy
import fcntl
import queue
import threading
import time
from concurrent.futures import Future
from pathlib import Path

from .recorder import Recorder
from .recording import validate
from .sources import resolved, validate_sources


class ControlConflict(ValueError):
    pass


class SessionController:
    def __init__(self, hub, config, recorder=None, *, record_on_start=True):
        self.hub, self.config = hub, config
        self.state_changed = lambda controller: None
        self.controls = hub.recording
        self.recorder = recorder or Recorder(config['state_directory'], config['session_limit_bytes'],
            config['total_limit_bytes'], controls=self.controls, open_session=record_on_start)
        self.recorder.controls = self.controls
        self.recorder.restore_settings(config)
        settings = self.controls.view()['settings']
        settings['active'] = record_on_start
        self.controls.configure(settings)
        self.state = 'RECORDING' if record_on_start and not self.recorder.reason else 'ERROR' if self.recorder.reason else 'IDLE'
        self.hub.put('recording_policy', 'settings', settings, critical=True)
        self.previous_status = None
        if not record_on_start:
            self.recorder.close()

    def tick(self):
        if self.state == 'RECORDING':
            self.recorder.append(self.hub.snapshot(self.recorder.last))
            self.recorder.append_raw()
            if self.recorder.reason:
                self.state = 'ERROR'
        self.controls.writer_error = self.recorder.reason
        self.record_status()

    def record_status(self):
        value = self.controls.view(self.recorder.reason)
        status = {key: {field: item for field, item in counter.items() if field != 'received_at'}
                  for key, counter in value['status'].items()}
        payload = {'status': status, 'reason': self.recorder.reason}
        if payload != self.previous_status:
            self.hub.put('recording_status', 'writer', payload)
            self.previous_status = payload

    def view(self):
        value = self.controls.view(self.recorder.reason)
        value.update(boot_id=self.hub.boot_id, session_id=self.recorder.session_id,
            state=self.state, updated_at=time.time(), stored_bytes=self.recorder.session_bytes(),
            resource_interval_s=self.config.get('resource_interval_s', 1.0),
            bt_event_repeat_interval_s=self.config.get('bt_event_repeat_interval_s', 1.0))
        return value

    def command(self, operation, *, settings=None, session_id='', expected_revision=0):
        if expected_revision and expected_revision != self.controls.revision:
            raise ControlConflict('Recording settings changed; refresh before retrying')
        if operation == 'get_config':
            return self.view()
        if operation == 'configure':
            if self.state in ('RECORDING', 'STOPPING'):
                raise ControlConflict('Stop recording before changing settings')
            settings = validate(settings)
            settings['active'] = False
            validate_sources({**resolved(self.config), **settings['sources']})
            with self.controls.transition():
                self.recorder.persist_settings(settings)
                self.controls.configure(settings)
                self.recorder.session_limit = settings['limit_mib'] * 1024**2
                self.hub.put('recording_policy', 'settings', settings, critical=True)
            return self.view()
        if operation == 'start':
            if self.state == 'RECORDING':
                return self.view()
            with self.controls.transition():
                settings = self.controls.view()['settings']
                settings['active'] = True
                self.controls.configure(settings)
                self.recorder.start_session()
                boundary = self.hub.snapshot()['sequence']
                self.hub.put('recording_policy', 'settings', settings, critical=True)
                snapshot = self.hub.snapshot(boundary)
                snapshot['rows'] = [r for r in snapshot['rows'] if r['kind'] not in ('event', 'log', 'result', 'recording_policy') or r['seq'] > boundary]
                self.recorder.append(snapshot)
            self.state = 'ERROR' if self.recorder.reason else 'RECORDING'
            if self.recorder.reason:
                raise ControlConflict(self.recorder.reason)
            return self.view()
        if operation != 'stop':
            raise ValueError('Unsupported recording operation')
        if session_id and session_id != self.recorder.session_id:
            raise ControlConflict('Session changed; refusing to stop another recording')
        if self.state == 'IDLE':
            return self.view()
        self.state = 'STOPPING'
        self.state_changed(self)
        with self.controls.transition():
            self.recorder.append(self.hub.snapshot(self.recorder.last))
            self.recorder.append_raw()
            settings = self.controls.view()['settings']
            settings['active'] = False
            self.controls.configure(settings, preserve_stats=True)
            self.hub.put('recording_policy', 'settings', settings, critical=True)
            self.record_status()
            self.recorder.append(self.hub.snapshot(self.recorder.last))
            self.recorder.close()
        self.state = 'ERROR' if self.recorder.reason else 'IDLE'
        return self.view()

    def close(self):
        self.command('stop')
        self.recorder.close()


class RecordingWorker:
    """Only this thread opens writers; process lock rejects duplicate storage owners."""
    def __init__(self, hub, config, *, record_on_start):
        self.hub, self.config, self.record_on_start = hub, config, record_on_start
        self.stop = threading.Event()
        self.ready = threading.Event()
        self.requests = queue.Queue(maxsize=32)
        self.view_lock = threading.Lock()
        self.latest = None
        self.error = None
        self.thread = threading.Thread(target=self.run, name='cleany-recording', daemon=False)

    def start(self):
        self.thread.start()
        if not self.ready.wait(10):
            raise RuntimeError('Logger startup timed out')
        if self.error:
            self.thread.join()
            raise RuntimeError(self.error)

    def submit(self, operation, **arguments):
        future = Future()
        try:
            self.requests.put_nowait((operation, arguments, future))
        except queue.Full:
            future.set_exception(ControlConflict('Logger command queue is full'))
        return future

    def snapshot(self):
        with self.view_lock:
            return copy.deepcopy(self.latest)

    def publish(self, controller):
        with self.view_lock:
            self.latest = controller.view()

    def run(self):
        controller = None
        lock = None
        try:
            directory = Path(self.config['state_directory']).expanduser()
            directory.mkdir(parents=True, exist_ok=True)
            lock = (directory / '.logger.lock').open('a')
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as exc:
                raise RuntimeError('Another logger owns this recording directory') from exc
            # Never classify an abandoned writer as finalized on restart.
            self.recover_interrupted(directory)
            controller = SessionController(self.hub, self.config, record_on_start=self.record_on_start)
            controller.state_changed = self.publish
            self.publish(controller)
            self.ready.set()
            while not self.stop.is_set():
                try:
                    operation, arguments, future = self.requests.get(timeout=.1)
                except queue.Empty:
                    operation = None
                if operation:
                    try:
                        result = controller.command(operation, **arguments)
                        self.publish(controller)
                        future.set_result(result)
                    except Exception as exc:
                        future.set_exception(exc)
                try:
                    controller.tick()
                except Exception as exc:
                    controller.recorder.stop_recording(f'Recording failed: {exc}')
                    controller.state = 'ERROR'
                self.publish(controller)
        except Exception as exc:
            self.error = str(exc)
            self.ready.set()
        finally:
            try:
                if controller:
                    try:
                        controller.close()
                        self.publish(controller)
                    finally:
                        controller.recorder.close()
            finally:
                if lock:
                    lock.close()
                while not self.requests.empty():
                    _, _, future = self.requests.get_nowait()
                    future.set_exception(RuntimeError('Logger has stopped'))

    @staticmethod
    def recover_interrupted(directory):
        import sqlite3
        for path in directory.glob('*.sqlite3'):
            with sqlite3.connect(f'file:{path}?mode=rw', uri=True) as db:
                if not db.execute("SELECT name FROM sqlite_master WHERE type='table' AND name='metadata'").fetchone():
                    continue
                row = db.execute("SELECT value FROM metadata WHERE key='bag_state'").fetchone()
                if row and row[0] == 'recording':
                    db.execute("UPDATE metadata SET value='incomplete' WHERE key='bag_state'")
                    db.execute("INSERT OR REPLACE INTO metadata VALUES ('stopped_reason','Logger interrupted before finalization')")

    def close(self):
        self.stop.set()
        self.thread.join(timeout=15)
        if self.thread.is_alive():
            raise RuntimeError('Logger worker has not finished closing its files')
