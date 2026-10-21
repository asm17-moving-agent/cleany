import { Panel } from "@/components/panel";
import { type Data } from "@/lib/monitor";

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
  return (
    <section aria-label="실행 요청 진행">
      <Panel title="실행 요청 진행">
        <p>
          <strong>{label}</strong>
        </p>
        <ol aria-label="실행 요청 단계" className="manip-request-phases">
          {phases.map(([id, name]) => {
            const visited = history.some((item) => item.phase === id);
            const current = active && request.phase === id;
            return (
              <li key={id} aria-current={current ? "step" : undefined}>
                <span>{name}</span>
                <span>{current ? "진행 중" : visited ? "보고됨" : "대기"}</span>
              </li>
            );
          })}
        </ol>
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
