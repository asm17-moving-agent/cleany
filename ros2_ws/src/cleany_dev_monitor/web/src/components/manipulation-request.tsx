import { Panel } from "@/components/panel";
import { type Data } from "@/lib/monitor";
import { CircleDot, Clock3, Radio, TriangleAlert } from "lucide-react";

const phases = [
  ["RUNTIME_PREPARING", "실행 환경 준비"],
  ["SERVER_PREPARING", "조작 서버 준비"],
  ["CHECKING_EXECUTION", "실행 요청 준비 확인"],
  ["INSPECTING_SCENE", "초기 장면 인식"],
  ["SELECTING_TARGET", "수거 대상 선택"],
  ["SUBMITTING_GOAL", "조작 요청 접수"],
  ["EXECUTING", "실물 조작"],
];

export function ManipulationRequest({
  request,
  fresh,
}: {
  request: Data;
  fresh: boolean;
}) {
  const active = request.state === "ACTIVE" && fresh;
  const label =
    request.state === "FINISHED"
      ? "실행 명령 종료"
      : request.state === "FAILED"
        ? "실행 요청 실패"
        : active
          ? phases.find(([id]) => id === request.phase)?.[1] || request.phase
          : "요청 진행 확인 불가";
  const history: Data[] = request.history || [];
  const phaseLabel = phases.find(([id]) => id === request.phase)?.[1];
  const reported = phases.filter(([id]) =>
    history.some((item) => item.phase === id),
  ).length;
  const tone = request.state === "FAILED" ? "bad" : active ? "info" : "neutral";
  const StatusIcon =
    request.state === "FAILED" ? TriangleAlert : active ? Radio : Clock3;
  return (
    <section
      aria-label="실행 요청 진행"
      className="manip-request"
      data-tone={tone}
    >
      <Panel
        title="실행 요청 진행"
        aside={<span className="manip-panel-caption">준비 및 요청 이력</span>}
      >
        <div className="manip-request-status" role="status">
          <StatusIcon size={24} aria-hidden="true" />
          <div>
            <p>
              <strong>{label}</strong>
            </p>
            {phaseLabel && !active && (
              <span>마지막 보고 단계: {phaseLabel}</span>
            )}
          </div>
        </div>
        <details
          className="manip-request-history"
          open={request.state === "ACTIVE"}
          key={request.execution_id}
        >
          <summary>
            준비 단계 이력{" "}
            <span>
              {reported} / {phases.length}단계 보고
            </span>
          </summary>
          <ol aria-label="실행 요청 단계" className="manip-request-phases">
            {phases.map(([id, name], index) => {
              const visited = history.some((item) => item.phase === id);
              const current = active && request.phase === id;
              return (
                <li
                  key={id}
                  aria-current={current ? "step" : undefined}
                  data-state={
                    current ? "active" : visited ? "reported" : "waiting"
                  }
                >
                  <span className="manip-request-index">
                    {String(index + 1).padStart(2, "0")}
                  </span>
                  <strong>{name}</strong>
                  <span className="manip-request-phase-state">
                    <CircleDot size={13} aria-hidden="true" />
                    {current ? "진행 중" : visited ? "보고됨" : "대기"}
                  </span>
                </li>
              );
            })}
          </ol>
        </details>
        <p className="manip-note">
          명령 준비와 초기 인식 진행입니다. 실제 조작 단계와 결과는 아래 서버
          기록에서 확인합니다.
        </p>
        <details className="manip-metadata">
          <summary>요청 진단 정보</summary>
          <code>{request.execution_id}</code>
          <p>{request.message}</p>
        </details>
      </Panel>
    </section>
  );
}
