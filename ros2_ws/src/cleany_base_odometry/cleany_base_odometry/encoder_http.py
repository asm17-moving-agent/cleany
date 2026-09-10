"""Read-only client for the ESP32 motor POC's existing status endpoint."""

from __future__ import annotations

from dataclasses import dataclass
from http.client import HTTPConnection, HTTPException
import json
from math import isfinite
import re
from time import monotonic


MAX_RESPONSE_BYTES = 2048


class EncoderReadError(Exception):
    """An HTTP response did not provide a usable encoder sample."""


@dataclass(frozen=True)
class EncoderSample:
    """Signed int32 counts in firmware order: FL, FR, RR, RL."""

    ticks: tuple[int, int, int, int]
    round_trip_time_sec: float
    boot_id: str = ''
    sample_seq: int = 0
    sample_time_us: int = 0


def parse_encoder_counts(payload: bytes) -> tuple[int, int, int, int]:
    return parse_encoder_sample(payload).ticks


def parse_encoder_sample(payload: bytes, round_trip_time_sec: float = 0.0) -> EncoderSample:
    if len(payload) > MAX_RESPONSE_BYTES:
        raise EncoderReadError('Status response exceeds size limit')
    try:
        document = json.loads(payload)
    except (ValueError, UnicodeError, RecursionError) as error:
        raise EncoderReadError('Status response is not valid JSON') from error
    counts = document.get('encoders') if isinstance(document, dict) else None
    if (
        not isinstance(counts, list)
        or len(counts) != 4
        or any(type(count) is not int for count in counts)
        or any(not -(2**31) <= count < 2**31 for count in counts)
    ):
        raise EncoderReadError('Expected four signed int32 encoder counts')
    ticks = (counts[0], counts[1], counts[2], counts[3])
    metadata = ('protocol_version', 'boot_id', 'sample_seq', 'sample_time_us')
    if not any(key in document for key in metadata):
        return EncoderSample(ticks, round_trip_time_sec)
    version = document.get('protocol_version')
    boot_id = document.get('boot_id')
    sequence = document.get('sample_seq')
    sample_time = document.get('sample_time_us')
    if (
        type(version) is not int or version != 1
        or not isinstance(boot_id, str) or re.fullmatch(r'[0-9a-f]{16}', boot_id) is None
        or type(sequence) is not int or not 0 <= sequence < 2**32
        or type(sample_time) is not int or not 0 <= sample_time < 2**63
    ):
        raise EncoderReadError('Invalid or incomplete MCU snapshot metadata')
    return EncoderSample(ticks, round_trip_time_sec, boot_id, sequence, sample_time)


class EncoderHttpClient:
    """One outstanding GET at a time; reconnect after any failed response.

    No proxy discovery, redirects, command endpoints, or firmware writes.
    Call read/close from the same thread.
    """

    def __init__(
        self, host: str, port: int = 80, timeout_sec: float = 0.2,
    ) -> None:
        if not host or any(char in host for char in '/?#@'):
            raise ValueError('host must be a hostname or IP, without a URL path')
        if not 1 <= port <= 65535:
            raise ValueError('port must be in 1..65535')
        if not isfinite(timeout_sec) or timeout_sec <= 0.0:
            raise ValueError('timeout_sec must be positive and finite')
        self._timeout_sec = timeout_sec
        self._connection = HTTPConnection(host, port, timeout=timeout_sec)

    def read(self) -> EncoderSample:
        started = monotonic()
        try:
            self._connection.request(
                'GET', '/api/status', headers={'Accept': 'application/json'},
            )
            response = self._connection.getresponse()
            if response.status != 200:
                raise EncoderReadError(f'Status endpoint returned HTTP {response.status}')
            payload = response.read(MAX_RESPONSE_BYTES + 1)
            elapsed = monotonic() - started
            # A socket timeout limits individual blocking operations; also reject
            # a response whose complete round trip exceeded the sample budget.
            if elapsed > self._timeout_sec:
                raise EncoderReadError('Status response exceeded latency limit')
            return parse_encoder_sample(payload, elapsed)
        except (OSError, HTTPException, EncoderReadError) as error:
            self.close()
            if isinstance(error, EncoderReadError):
                raise
            raise EncoderReadError(f'Status request failed: {error}') from error

    def close(self) -> None:
        self._connection.close()
