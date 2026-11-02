import {
  useEffect,
  useRef,
  useState,
  type PointerEvent as ReactPointerEvent,
} from "react";
import { createPortal } from "react-dom";
import { Play, Pause } from "lucide-react";
import { Button } from "@/components/ui/button";
import "./recording-timeline.css";

export type TimelineSession = {
  id: string;
  start?: number;
  end?: number;
  segments?: { start: number; end: number }[];
};
export function recordingSegments(sessions: TimelineSession[]) {
  return sessions
    .flatMap((s) =>
      (
        s.segments ??
        (s.start != null && s.end != null
          ? [{ start: s.start, end: s.end }]
          : [])
      )
        .filter(
          (r) =>
            Number.isFinite(r.start) &&
            Number.isFinite(r.end) &&
            r.end >= r.start,
        )
        .map((r) => ({ ...r, id: s.id })),
    )
    .sort((a, b) => a.start - b.start || a.id.localeCompare(b.id));
}
export function sessionAt(sessions: TimelineSession[], at: number) {
  return (
    recordingSegments(sessions)
      .filter((s) => s.start <= at && at <= s.end)
      .at(-1)?.id ?? ""
  );
}
const clock = (at: number) =>
  new Date(at * 1000).toLocaleTimeString("ko-KR", {
    hour: "2-digit",
    minute: "2-digit",
    second: "2-digit",
    hour12: false,
  });
const minuteClock = (at: number) =>
  new Date(at * 1000).toLocaleTimeString("ko-KR", {
    hour: "2-digit",
    minute: "2-digit",
    hour12: false,
  });
const localDate = (at: number) => {
  const d = new Date(at * 1000);
  return `${d.getFullYear()}-${String(d.getMonth() + 1).padStart(2, "0")}-${String(d.getDate()).padStart(2, "0")}T${String(d.getHours()).padStart(2, "0")}:${String(d.getMinutes()).padStart(2, "0")}:${String(d.getSeconds()).padStart(2, "0")}`;
};
export function RecordingTimeline({
  sessions,
  controlsHost,
  focusAt,
  now,
  at,
  historical,
  playing,
  speed,
  onSeek,
  onLive,
  onPlaying,
  onSpeed,
}: {
  sessions: TimelineSession[];
  controlsHost?: HTMLElement | null;
  focusAt?: number | null;
  now: number;
  at: number;
  historical: boolean;
  playing: boolean;
  speed: number;
  onSeek: (at: number) => void;
  onLive: () => void;
  onPlaying: (playing: boolean) => void;
  onSpeed: (speed: number) => void;
}) {
  const [end, setEnd] = useState<number | null>(null);
  const [span, setSpan] = useState(3600);
  const [jump, setJump] = useState(localDate(now).slice(0, 10));
  useEffect(() => {
    if (focusAt != null) {
      setEnd(Math.min(now, focusAt + span / 2));
      setJump(localDate(focusAt).slice(0, 10));
    }
  }, [focusAt]);
  const showDate = (value: string) => {
    if (!value) return;
    const start = new Date(`${value}T00:00:00`);
    if (!Number.isFinite(start.getTime()) || start.getTime() / 1000 > now)
      return;
    const next = new Date(start);
    next.setDate(next.getDate() + 1);
    setJump(value);
    const limit = Math.min(now, next.getTime() / 1000);
    setSpan(Math.max(1, limit - start.getTime() / 1000));
    setEnd(limit);
  };
  const track = useRef<HTMLDivElement>(null);
  const gesture = useRef<{
    x: number;
    end: number;
    seek: boolean;
    width: number;
  } | null>(null);
  const [panning, setPanning] = useState(false);
  const [shiftHeld, setShiftHeld] = useState(false);
  useEffect(() => {
    const key = (e: KeyboardEvent) => setShiftHeld(e.shiftKey);
    const blur = () => setShiftHeld(false);
    window.addEventListener("keydown", key);
    window.addEventListener("keyup", key);
    window.addEventListener("blur", blur);
    return () => {
      window.removeEventListener("keydown", key);
      window.removeEventListener("keyup", key);
      window.removeEventListener("blur", blur);
    };
  }, []);
  const right = Math.min(now, end ?? now),
    left = right - span,
    pointer = historical ? at : now;
  const segments = recordingSegments(sessions);
  const percent = (t: number) => (100 * (t - left)) / span;
  const point = (x: number) => {
    const r = track.current!.getBoundingClientRect();
    return Math.max(
      left,
      Math.min(right, left + ((x - r.left) / r.width) * span),
    );
  };
  const fineSpan = 60;
  type FineWindow = { center: number; cursor: number; anchor: number };
  const [fine, setFine] = useState<FineWindow | null>(null);
  const fineRef = useRef<FineWindow | null>(null);
  const fineRatio = useRef<number | null>(null);
  const fineDragging = useRef(false);
  const callbacks = useRef({ onSeek, now, left, right, span });
  callbacks.current = { onSeek, now, left, right, span };
  const updateFine = (value: FineWindow | null) => {
    fineRef.current = value;
    setFine(value);
  };
  const finePoint = (center: number, ratio: number) =>
    Math.min(
      callbacks.current.now,
      Math.round((center + (ratio - 0.5) * fineSpan) * 10) / 10,
    );
  const hoverOverview = (x: number) => {
    if (gesture.current || fineDragging.current) return;
    const r = track.current!.getBoundingClientRect();
    const t = Math.min(now, point(x));
    fineRatio.current = null;
    updateFine({
      center: Math.min(t, now - fineSpan / 2),
      cursor: t,
      anchor: (100 * (x - r.left)) / r.width,
    });
  };
  const closeFine = () => {
    fineRatio.current = null;
    if (!fineDragging.current) updateFine(null);
  };
  useEffect(() => {
    if (!fine) return;
    let frame = 0;
    let previous = performance.now();
    const scroll = (stamp: number) => {
      const elapsed = Math.min((stamp - previous) / 1000, 0.1);
      previous = stamp;
      const ratio = fineRatio.current;
      const current = fineRef.current;
      if (current && ratio != null) {
        // Speed increases smoothly toward either edge, including during a drag.
        const edge =
          ratio < 0.15
            ? (ratio - 0.15) / 0.15
            : ratio > 0.85
              ? (ratio - 0.85) / 0.15
              : 0;
        if (edge) {
          const center = Math.min(
            callbacks.current.now - fineSpan / 2,
            current.center + edge * fineSpan * 0.35 * elapsed,
          );
          if (center !== current.center) {
            const cursor = finePoint(center, ratio);
            updateFine({ ...current, center, cursor });
            if (fineDragging.current) callbacks.current.onSeek(cursor);
            if (
              center < callbacks.current.left ||
              center > callbacks.current.right
            )
              setEnd(
                Math.min(
                  callbacks.current.now,
                  center + callbacks.current.span / 2,
                ),
              );
          }
        }
      }
      frame = requestAnimationFrame(scroll);
    };
    frame = requestAnimationFrame(scroll);
    return () => cancelAnimationFrame(frame);
  }, [fine !== null]);
  const fineClock = (t: number) =>
    `${clock(Math.round(t * 10) / 10)}.${String(Math.round(t * 10) % 10)}`;
  const beginGesture = (
    e: ReactPointerEvent<HTMLDivElement>,
    seek: boolean,
  ) => {
    if (e.button !== 0) return;
    e.preventDefault();
    gesture.current = {
      x: e.clientX,
      end: right,
      seek,
      width: e.currentTarget.getBoundingClientRect().width,
    };
    e.currentTarget.setPointerCapture(e.pointerId);
    setPanning(!seek);
    if (!seek) {
      closeFine();
      return;
    }
    onPlaying(false);
    const t = point(e.clientX);
    const target = (e.target as HTMLElement).closest<HTMLElement>(
      "[data-start]",
    );
    onSeek(
      target?.dataset.start
        ? Math.min(
            now,
            Math.max(+target.dataset.start, Math.min(+target.dataset.end!, t)),
          )
        : t,
    );
  };
  const moveGesture = (e: ReactPointerEvent<HTMLDivElement>) => {
    const g = gesture.current;
    if (!g) return;
    if (g.seek) onSeek(point(e.clientX));
    else setEnd(Math.min(now, g.end - ((e.clientX - g.x) / g.width) * span));
  };
  const finishGesture = (e: ReactPointerEvent<HTMLDivElement>) => {
    gesture.current = null;
    setPanning(false);
    if (e.currentTarget.hasPointerCapture(e.pointerId))
      e.currentTarget.releasePointerCapture(e.pointerId);
  };
  const controls = (
    <div className="timeline-toolbar">
      <div className="timeline-primary-controls">
        <input
          aria-label="날짜 선택"
          type="date"
          max={localDate(now).slice(0, 10)}
          value={end === null ? localDate(now).slice(0, 10) : jump}
          onChange={(e) => showDate(e.target.value)}
        />
        <select
          aria-label="재생 배속"
          value={speed}
          onChange={(e) => onSpeed(+e.target.value)}
        >
          {[0.5, 1, 2, 4].map((v) => (
            <option key={v} value={v}>
              {v}×
            </option>
          ))}
        </select>
        <Button
          variant="outline"
          onClick={() => {
            setEnd(null);
            setSpan(3600);
            setJump(localDate(now).slice(0, 10));
            onLive();
          }}
        >
          라이브
        </Button>
      </div>
      <Button
        variant="outline"
        size="icon-sm"
        aria-label={playing ? "일시정지" : "재생"}
        title={playing ? "일시정지" : "재생"}
        disabled={!historical || (!sessionAt(sessions, at) && !playing)}
        onClick={() => onPlaying(!playing)}
      >
        {playing ? <Pause size={15} /> : <Play size={15} />}
      </Button>
    </div>
  );
  return (
    <section className="recording-timeline" aria-label="녹화 시간 탐색">
      {controlsHost ? createPortal(controls, controlsHost) : controls}
      <div className="timeline-surface" onPointerLeave={closeFine}>
        {fine && (
          <div
            className="timeline-fine"
            data-testid="timeline-fine"
            style={{
              left: `clamp(min(280px, 50%), ${fine.anchor}%, calc(100% - min(280px, 50%)))`,
            }}
          >
            <div className="timeline-fine-heading">
              <strong>정밀 탐색, 1분</strong>
              <span>{fineClock(fine.cursor)}</span>
            </div>
            <div
              className="timeline-track timeline-fine-track"
              data-testid="timeline-fine-track"
              data-start={fine.center - fineSpan / 2}
              data-end={fine.center + fineSpan / 2}
              role="slider"
              tabIndex={0}
              aria-label="정밀 재생 시각"
              aria-valuemin={fine.center - fineSpan / 2}
              aria-valuemax={fine.center + fineSpan / 2}
              aria-valuenow={fine.cursor}
              aria-valuetext={fineClock(fine.cursor)}
              onPointerMove={(e) => {
                const r = e.currentTarget.getBoundingClientRect();
                const ratio = Math.max(
                  0,
                  Math.min(1, (e.clientX - r.left) / r.width),
                );
                fineRatio.current = ratio;
                const current = fineRef.current!;
                const cursor = finePoint(current.center, ratio);
                updateFine({ ...current, cursor });
                if (fineDragging.current) onSeek(cursor);
              }}
              onPointerLeave={() => {
                fineRatio.current = null;
              }}
              onPointerDown={(e) => {
                if (e.button !== 0) return;
                fineDragging.current = true;
                e.currentTarget.setPointerCapture(e.pointerId);
                onPlaying(false);
                const r = e.currentTarget.getBoundingClientRect();
                const ratio = Math.max(
                  0,
                  Math.min(1, (e.clientX - r.left) / r.width),
                );
                fineRatio.current = ratio;
                const current = fineRef.current!;
                const cursor = finePoint(current.center, ratio);
                updateFine({ ...current, cursor });
                onSeek(cursor);
              }}
              onPointerUp={(e) => {
                fineDragging.current = false;
                if (e.currentTarget.hasPointerCapture(e.pointerId))
                  e.currentTarget.releasePointerCapture(e.pointerId);
                const r = e.currentTarget.getBoundingClientRect();
                if (
                  e.clientX < r.left ||
                  e.clientX > r.right ||
                  e.clientY < r.top ||
                  e.clientY > r.bottom
                )
                  closeFine();
              }}
              onPointerCancel={() => {
                fineDragging.current = false;
                closeFine();
              }}
              onKeyDown={(e) => {
                if (e.key === "Escape") {
                  fineDragging.current = false;
                  updateFine(null);
                  track.current
                    ?.querySelector<HTMLElement>('[role="slider"]')
                    ?.focus();
                }
                if (e.key === "ArrowLeft" || e.key === "ArrowRight") {
                  e.preventDefault();
                  onPlaying(false);
                  fineRatio.current = null;
                  const cursor = Math.min(
                    now,
                    (historical ? at : fine.cursor) +
                      (e.key === "ArrowLeft" ? -1 : 1) * (e.shiftKey ? 1 : 0.1),
                  );
                  updateFine({
                    ...fine,
                    cursor,
                    center: Math.min(now - fineSpan / 2, cursor),
                  });
                  onSeek(Math.min(now, Math.round(cursor * 10) / 10));
                }
              }}
            >
              {segments
                .filter(
                  (s) =>
                    s.end >= fine.center - fineSpan / 2 &&
                    s.start <= fine.center + fineSpan / 2,
                )
                .map((s, i) => {
                  const start = Math.max(
                    0,
                    (100 * (s.start - fine.center + fineSpan / 2)) / fineSpan,
                  );
                  const finish = Math.min(
                    100,
                    (100 * (s.end - fine.center + fineSpan / 2)) / fineSpan,
                  );
                  return (
                    <div
                      key={`${s.id}:${i}`}
                      className="timeline-segment"
                      style={{
                        left: `${start}%`,
                        width: `${Math.max(0.15, finish - start)}%`,
                      }}
                    />
                  );
                })}
              {pointer >= fine.center - fineSpan / 2 &&
                pointer <= fine.center + fineSpan / 2 && (
                  <div
                    className="timeline-pointer"
                    style={{
                      left: `${(100 * (pointer - fine.center + fineSpan / 2)) / fineSpan}%`,
                    }}
                  />
                )}
              <div
                className="timeline-pointer timeline-hover-pointer"
                style={{
                  left: `${(100 * (fine.cursor - fine.center + fineSpan / 2)) / fineSpan}%`,
                }}
              />
            </div>
            <div className="timeline-fine-help">
              클릭, 드래그로 선택, 양끝에서 시간 이동
            </div>
          </div>
        )}
        <div
          ref={track}
          className={`timeline-track${shiftHeld || panning ? " timeline-pan-mode" : ""}`}
          data-testid="timeline-track"
          onPointerEnter={(e) => {
            if (e.pointerType === "mouse" && !e.shiftKey)
              hoverOverview(e.clientX);
          }}
          onPointerDown={(e) => beginGesture(e, !e.shiftKey)}
          onPointerMove={(e) => {
            if (gesture.current) moveGesture(e);
            else if (e.pointerType === "mouse" && !e.shiftKey)
              hoverOverview(e.clientX);
            else closeFine();
          }}
          onPointerUp={finishGesture}
          onPointerCancel={finishGesture}
          onLostPointerCapture={() => {
            gesture.current = null;
            setPanning(false);
          }}
        >
          {segments
            .filter((s) => s.end >= left && s.start <= right)
            .map((s, i) => (
              <div
                key={`${s.id}:${i}`}
                data-seek
                data-start={s.start}
                data-end={s.end}
                className="timeline-segment"
                title={`${clock(s.start)}–${clock(s.end)}`}
                style={{
                  left: `${Math.max(0, percent(s.start))}%`,
                  width: `${Math.max(0.15, Math.min(100, percent(s.end)) - Math.max(0, percent(s.start)))}%`,
                }}
              />
            ))}
          {fine && (
            <div
              aria-hidden="true"
              className="timeline-pointer timeline-hover-pointer"
              style={{ left: `${fine.anchor}%` }}
            />
          )}
          {pointer >= left && pointer <= right && (
            <div
              data-seek
              className="timeline-pointer"
              role="slider"
              tabIndex={0}
              aria-label="재생 포인터"
              aria-valuemin={left}
              aria-valuemax={right}
              aria-valuenow={pointer}
              aria-valuetext={clock(pointer)}
              style={{ left: `${percent(pointer)}%` }}
              onKeyDown={(e) => {
                if (e.key === "Home" || e.key === "End") {
                  e.preventDefault();
                  const segment = segments.find(
                    (s) =>
                      s.id === sessionAt(sessions, pointer) &&
                      s.start <= pointer &&
                      pointer <= s.end,
                  );
                  if (segment) {
                    onPlaying(false);
                    onSeek(
                      Math.min(
                        now,
                        e.key === "Home" ? segment.start : segment.end,
                      ),
                    );
                  }
                }
                if (e.key === "ArrowLeft" || e.key === "ArrowRight") {
                  e.preventDefault();
                  onPlaying(false);
                  onSeek(
                    Math.max(
                      left,
                      Math.min(
                        right,
                        pointer +
                          (e.key === "ArrowLeft" ? -1 : 1) *
                            (e.shiftKey ? 10 : 1),
                      ),
                    ),
                  );
                }
              }}
            />
          )}
        </div>
      </div>
      <div
        className="timeline-ticks timeline-range-ticks"
        data-testid="timeline-range-ticks"
        role="group"
        aria-label="시간 범위 이동"
        title="드래그하여 시간 범위 이동"
        onPointerDown={(e) => beginGesture(e, false)}
        onPointerMove={moveGesture}
        onPointerUp={finishGesture}
        onPointerCancel={finishGesture}
        onLostPointerCapture={() => {
          gesture.current = null;
          setPanning(false);
        }}
      >
        {Array.from({ length: 7 }, (_, i) => (
          <span key={i}>{minuteClock(left + (span * i) / 6)}</span>
        ))}
      </div>
      {historical && !sessionAt(sessions, at) && (
        <span role="status">해당 시각에 기록 없음</span>
      )}
    </section>
  );
}
