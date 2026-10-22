"""A persistent CPU-facing port with an idle-releasable licensed GPU worker.

The ROS process never imports gsnet or torch. CPU grasp results stay usable after
worker shutdown. The child uses spawn, preserving the container's SDK identity.
"""
from __future__ import annotations

import math
import multiprocessing as mp
from multiprocessing.connection import Connection
from threading import RLock
import time

from cleany_grasping.anygrasp_adapter import AnyGraspPredictor, ModelUnavailableError


def _serve(connection: Connection, options: dict) -> None:
    try:
        predictor = AnyGraspPredictor(**options)
        predictor._load()
        connection.send(('ok', None))
        while True:
            request = connection.recv()
            if request is None:
                break
            points, region_mask = request
            try:
                result = predictor._predict_points(points, region_mask)
                connection.send(('ok', result))
            except Exception as error:
                connection.send(('model_error' if isinstance(error, ModelUnavailableError)
                                 else 'error', str(error)))
    except (EOFError, BrokenPipeError):
        pass
    except Exception as error:
        try:
            connection.send(('model_error', str(error)))
        except (EOFError, BrokenPipeError):
            pass
    finally:
        connection.close()


class AnyGraspWorkerPredictor:
    def __init__(self, *, idle_timeout_sec: float = 120.0,
                 request_timeout_sec: float = 120.0, **options) -> None:
        if (not math.isfinite(idle_timeout_sec) or idle_timeout_sec < 0
                or not math.isfinite(request_timeout_sec) or request_timeout_sec <= 0):
            raise ValueError('Worker idle timeout must be nonnegative and request timeout positive')
        # Validate options without loading the licensed SDK or CUDA runtime.
        self._sampling = AnyGraspPredictor(**options)
        self._options = options
        self._idle_timeout = idle_timeout_sec
        self._request_timeout = request_timeout_sec
        self._lock = RLock()
        self._process = None
        self._connection = None
        self._last_used = time.monotonic()

    @property
    def worker_pid(self) -> int | None:
        return self._process.pid if self._process is not None else None

    def _receive(self):
        try:
            if not self._connection.poll(self._request_timeout):
                raise ModelUnavailableError('AnyGrasp GPU worker timed out')
            status, result = self._connection.recv()
        except (EOFError, BrokenPipeError, OSError) as error:
            self.close()
            raise ModelUnavailableError('AnyGrasp GPU worker exited') from error
        if status != 'ok':
            self.close()
            raise ModelUnavailableError(f'AnyGrasp worker failed: {result}')
        return result

    def prepare(self) -> None:
        with self._lock:
            if self._process is not None and self._process.is_alive():
                return
            self.close()
            context = mp.get_context('spawn')
            self._connection, child = context.Pipe()
            self._process = context.Process(target=_serve, args=(child, self._options), daemon=True)
            try:
                self._process.start()
                child.close()
                self._receive()
            except BaseException:
                child.close()
                self.close()
                raise
            self._last_used = time.monotonic()

    def _predict(self, method, target, context, bounds):
        with self._lock:
            self.prepare()
            try:
                points, region_mask = self._sampling._prepare_input(target, context, bounds)
                if method == 'predict' and self._sampling.should_retry_full_resolution(context):
                    points, region_mask = self._sampling._adaptive_sample(points, region_mask)
                # SDK inference uses XYZ and steering only. Sample on the CPU
                # before IPC; never copy unused colors or an omitted full cloud.
                self._connection.send((points, region_mask))
                return self._receive()
            except (EOFError, BrokenPipeError, OSError) as error:
                self.close()
                raise ModelUnavailableError('AnyGrasp GPU worker transport failed') from error
            except BaseException:
                self.close()
                raise
            finally:
                self._last_used = time.monotonic()

    def predict(self, target_cloud, context_cloud, workspace_bounds):
        return self._predict('predict', target_cloud, context_cloud, workspace_bounds)

    def predict_full_resolution(self, target_cloud, context_cloud, workspace_bounds):
        return self._predict('predict_full_resolution', target_cloud, context_cloud, workspace_bounds)

    def should_retry_full_resolution(self, context_cloud) -> bool:
        return self._sampling.should_retry_full_resolution(context_cloud)

    def release_if_idle(self) -> bool:
        with self._lock:
            if (self._process is not None and self._idle_timeout > 0
                    and time.monotonic()-self._last_used >= self._idle_timeout):
                self.close()
                return True
            return False

    def close(self) -> None:
        with self._lock:
            process, connection = self._process, self._connection
            self._process = self._connection = None
            if connection is not None:
                try:
                    connection.send(None)
                except (OSError, BrokenPipeError):
                    pass
                connection.close()
            if process is not None:
                if process.pid is None:
                    process.close()
                    return
                process.join(timeout=2.0)
                if process.is_alive():
                    process.terminate()
                    process.join(timeout=2.0)
                if process.is_alive():
                    process.kill()
                    process.join(timeout=2.0)
                process.close()
