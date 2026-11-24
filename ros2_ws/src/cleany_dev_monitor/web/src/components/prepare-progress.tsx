import { CheckCircle2, Circle, LoaderCircle, OctagonPause } from "lucide-react";
import type { Data } from "@/lib/monitor";

const steps = [
  ["preflight", "모터 통신·토크 점검"],
  ["control", "팔 제어기·MoveIt"],
  ["planning", "깊이 장면·파지 계획"],
  ["gpu", "GPU IK 예열"],
  ["feedback", "관절 피드백·토크 준비"],
  ["executor", "실행기 연결"],
] as const;
const phases: Record<string, string> = {
  GPU_INITIALIZATION: "GPU 초기화",
  GPU_GEOMETRY: "충돌 형상 로드",
  IK_SOLVER_SETUP: "IK 솔버 구성",
  IK_WARMUP: "IK 초기화",
  GPU_IK: "IK 계산",
  GPU_FK_ALIGNMENT: "FK 정렬 확인",
};
const states: Record<string, string> = {
  PENDING: "접수",
  RUNNING: "진행 중",
  SUCCEEDED: "완료",
  FAILED: "실패",
  INTERRUPTED: "중단",
  UNKNOWN: "결과 미확인",
  ACKNOWLEDGED: "기록 확인",
};

export function PrepareProgress({
  job,
  observedAt,
}: {
  job: Data;
  observedAt: number;
}) {
  const active = ["PENDING", "RUNNING"].includes(job.state);
  const elapsed = Math.max(
    0,
    Math.round((job.finished_at ?? observedAt / 1000) - job.created_at),
  );
  const showElapsed = job.finished_at || (active && observedAt);
  const evidence = job.prepare_progress || {};
  return (
    <section className="prepare-progress" aria-label="로봇 준비 진행 상황">
      <div className="prepare-title">
        <h3>
          {active ? "로봇 준비" : "최근 로봇 준비"} · {states[job.state]}
        </h3>
        {showElapsed ? (
          <span>
            {Math.floor(elapsed / 60)}분 {elapsed % 60}초
          </span>
        ) : null}
      </div>
      {evidence.reuse && <p>기동 중인 런타임을 다시 확인했습니다.</p>}
      <ol>
        {steps.map(([id, label]) => {
          const item = evidence[id];
          const done = item?.state === "done";
          const working = item?.state === "active" && active;
          const ended = item?.state === "active" && !active;
          const Icon = done
            ? CheckCircle2
            : working
              ? LoaderCircle
              : ended
                ? OctagonPause
                : Circle;
          const detail =
            id === "gpu" && item?.phase
              ? [
                  item.arm === "left"
                    ? "왼팔"
                    : item.arm === "right"
                      ? "오른팔"
                      : "",
                  phases[item.phase] || item.phase,
                ]
                  .filter(Boolean)
                  .join(" · ")
              : item?.waiting_for;
          const status = done
            ? "완료"
            : working
              ? "진행 중"
              : ended
                ? "완료 미확인"
                : active
                  ? "대기"
                  : "기록 없음";
          return (
            <li
              key={id}
              data-state={done ? "done" : working ? "working" : "other"}
            >
              <Icon
                size={15}
                aria-hidden="true"
                className={working ? "operator-spin" : ""}
              />
              <span>{label}</span>
              <small>
                {status}
                {Number.isFinite(item?.seconds)
                  ? " · " + item.seconds.toFixed(1) + "초"
                  : done ? " · 시간 미기록" : ""}
              </small>
              {working && detail && <p>{detail}</p>}
            </li>
          );
        })}
      </ol>
      {Number.isFinite(evidence.gpu?.seconds) &&
        Number.isFinite(evidence.planning?.seconds) && (
          <p>파지 계획 준비 시간에 GPU IK 예열이 포함됩니다.</p>
        )}
      {job.message && <p role="status">{job.message}</p>}
    </section>
  );
}
