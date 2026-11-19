import { useEffect, useRef, useState } from "react";
import {
  Activity,
  ArrowRight,
  ArrowUpRight,
  Check,
  CheckCircle2,
  ChevronUp,
  Circle,
  Clock3,
  Hand,
  History,
  Radio,
  TriangleAlert,
  X,
} from "lucide-react";
import { Button } from "./ui/button";
import { type Data, type Row } from "@/lib/monitor";
import {
  currentExecution,
  manipulationObservation,
  missionExecutions,
  resultLabels,
  objectLabels,
  placementLabels,
  stageIcons,
  stageLabel,
  stages,
  normalSteps,
  stepLabel,
} from "@/lib/manipulation";
import "./execute-one-detail.css";

type Props = {
  rows: Row[];
  timeAt: number;
  replay: boolean;
  connected: boolean;
  close: () => void;
  openDashboard: () => void;
};
const age = (now: number, received: number) => Math.max(0, now - received);
const stateLabel: Record<string, string> = {
  completed: "완료",
  active: "진행 중",
  failed: "실패",
  last: "마지막 관측",
  pending: "미관측",
};

export function ExecuteOneDetail({
  rows,
  timeAt,
  replay,
  connected,
  close,
  openDashboard,
}: Props) {
  const panel = useRef<HTMLElement>(null);
  const [selected, setSelected] = useState("");
  const observation = manipulationObservation(rows, timeAt, replay, connected);
  const { runtime, runtimeFresh, serverPresent } = observation;
  const { mission, matching, automatic, execution } =
    missionExecutions(observation);
  const run =
    matching.find((item) => item.record.execution_id === selected) || automatic;
  const record: Data = run?.record || {};
  const current = currentExecution(run, observation);
  const terminal = !!record.has_result;
  const completed = new Set<string>(
    Array.isArray(record.completed_substages) ? record.completed_substages : [],
  );
  const done = normalSteps.filter(([id]) => completed.has(id)).length;
  const total = normalSteps.length;
  const end =
    Number(record.updated_at_ns || 0) +
    (current && run ? age(timeAt, run.received_at) * 1e9 : 0);
  const elapsed =
    run &&
    !run.clock_regressed &&
    record.accepted_at_ns > 0 &&
    end >= record.accepted_at_ns
      ? (end - record.accepted_at_ns) / 1e9
      : null;
  const noAction = ["done", "skip", "human_review"].includes(
    runtime.proposal?.action,
  );
  const failure =
    terminal && ["FAILED", "FATAL", "BLOCKED"].includes(record.status);
  const tone = failure
    ? "failed"
    : terminal
      ? record.status === "SUCCESS"
        ? "completed"
        : "last"
      : current
        ? "active"
        : "last";
  const stageIndex = stages.findIndex(([id]) => id === record.stage);
  const Icon = terminal
    ? record.status === "SUCCESS"
      ? CheckCircle2
      : record.status === "CANCELED"
        ? Hand
        : TriangleAlert
    : stageIcons[record.stage] || Activity;
  const observationLabel = terminal
    ? "서버 최종 결과"
    : current
      ? replay
        ? "기록 시점 진행"
        : "실시간 관측"
      : "상태 미확인 · 마지막 관측";
  const phaseState = (id: string, steps: [string, string][]) =>
    record.failed_stage === id
      ? "failed"
      : current && record.stage === id
        ? "active"
        : steps.every(([step]) => completed.has(step))
          ? "completed"
          : !terminal && record.stage === id
            ? "last"
            : "pending";
  const substepState = (id: string) =>
    record.failed_substage === id
      ? "failed"
      : completed.has(id)
        ? "completed"
        : !terminal && record.substage === id
          ? current
            ? "active"
            : "last"
          : "pending";

  // Only opening the panel moves focus/scroll. Telemetry must never pull the
  // reader away from another phase or reset their position.
  useEffect(() => {
    panel.current?.focus({ preventScroll: true });
    panel.current?.scrollIntoView({ block: "start", behavior: "instant" });
  }, []);

  return (
    <section
      ref={panel}
      id="execute-one-detail"
      tabIndex={-1}
      className="execute-detail"
      data-tone={tone}
      aria-label="ExecuteOne 조작 상세"
    >
      <header className="execute-detail-header">
        <div className="execute-detail-heading">
          <span className="execute-detail-icon">
            <Hand size={21} aria-hidden="true" />
          </span>
          <div>
            <p className="execute-breadcrumb">
              청소 BT <ArrowRight size={12} aria-hidden="true" />{" "}
              <strong>ExecuteOne</strong>
            </p>
            <h2>한 물체 조작</h2>
          </div>
          <span className="execute-owner">Action 서버 진행</span>
        </div>
        <div className="execute-detail-actions">
          <Button variant="ghost" size="sm" onClick={openDashboard}>
            전체 조작 화면 <ArrowUpRight size={14} />
          </Button>
          <Button
            variant="outline"
            size="sm"
            onClick={close}
            aria-label="조작 상세 접기"
          >
            <ChevronUp size={16} />
            접기
          </Button>
        </div>
      </header>
      <div className="execute-detail-body">
        {noAction && (
          <p className="execute-context-note">
            <History size={15} aria-hidden="true" />
            현재 제안 {runtime.proposal.action} · 이번 주기는 조작 Action 없이
            통과합니다.{run && " 아래는 이 미션에서 관측한 이전 실행입니다."}
          </p>
        )}
        {matching.length > 1 && (
          <label className="execute-picker">
            <span>실행 기록</span>
            <select
              value={
                matching.some((item) => item.record.execution_id === selected)
                  ? selected
                  : ""
              }
              aria-label="ExecuteOne 실행 선택"
              onChange={(event) => setSelected(event.target.value)}
            >
              <option value="">현재 / 최근 실행 자동 선택</option>
              {matching.map(({ record: item }) => (
                <option key={item.execution_id} value={item.execution_id}>
                  물체 {item.object_id} ·{" "}
                  {item.has_result
                    ? resultLabels[item.status] || item.status
                    : "진행 기록"}{" "}
                  · {item.execution_id.slice(0, 8)}
                </option>
              ))}
            </select>
          </label>
        )}
        {!run ? (
          <div className="execute-wait" role="status">
            <span className="execute-wait-icon">
              <Radio size={28} aria-hidden="true" />
            </span>
            <div>
              <h3>
                {noAction
                  ? "이번 주기의 조작 실행 없음"
                  : "연결된 조작 기록을 기다리는 중"}
              </h3>
              <p>
                {noAction
                  ? "다음 재관측과 완료 검사를 이어갑니다."
                  : "이 미션과 execution ID가 일치하는 서버 기록이 수신되면 세부 진행을 표시합니다."}
              </p>
              {execution &&
                runtime.manipulation_feedback?.execution_id === execution && (
                  <p className="execute-feedback-only">
                    마지막 Feedback ·{" "}
                    {stageLabel(runtime.manipulation_feedback.stage)}
                    {runtime.manipulation_feedback.substage &&
                      ` / ${stepLabel(runtime.manipulation_feedback.substage)}`}
                  </p>
                )}
            </div>
          </div>
        ) : (
          <>
            <div className="execute-current" data-tone={tone}>
              <div className="execute-current-main">
                <div className="execute-observation">
                  <span className="execute-state-tag" data-state={tone}>
                    {current ? (
                      <Radio size={13} />
                    ) : failure ? (
                      <TriangleAlert size={13} />
                    ) : terminal ? (
                      record.status === "SUCCESS" ? (
                        <CheckCircle2 size={13} />
                      ) : (
                        <History size={13} />
                      )
                    ) : (
                      <Clock3 size={13} />
                    )}
                    {observationLabel}
                  </span>
                  <span className="execute-source">
                    {record.execution_profile === "mock"
                      ? "Mock"
                      : record.execution_profile || "출처 미확인"}
                  </span>
                  {selected && run !== automatic && (
                    <span className="execute-history-label">
                      선택한 이전 실행
                    </span>
                  )}
                </div>
                <div className="execute-current-title">
                  <span className="execute-current-icon">
                    <Icon size={29} aria-hidden="true" />
                  </span>
                  <div>
                    <small>
                      {terminal
                        ? "Action 결과"
                        : current
                          ? "현재 세부 동작"
                          : "마지막 세부 동작"}
                    </small>
                    <h3>
                      {terminal
                        ? resultLabels[record.status] ||
                          record.status ||
                          "결과 미확인"
                        : stepLabel(record.substage) ||
                          stageLabel(record.stage)}
                    </h3>
                    <p>
                      {terminal ? (
                        <code>{record.status}</code>
                      ) : (
                        <>
                          <span>
                            {stageIndex >= 0
                              ? `${String(stageIndex + 1).padStart(2, "0")} / 09 · `
                              : ""}
                            {stageLabel(record.stage)}
                          </span>
                          {record.substage && <code>{record.substage}</code>}
                        </>
                      )}
                    </p>
                  </div>
                </div>
              </div>
              <div className="execute-metrics">
                <div>
                  <span>확인된 동작</span>
                  <strong>
                    {done}
                    <small> / {total}</small>
                  </strong>
                </div>
                <div>
                  <span>{current ? "관측 경과" : "기록된 시간"}</span>
                  <strong>
                    {elapsed === null ? "—" : elapsed.toFixed(1)}
                    <small> 초</small>
                  </strong>
                </div>
              </div>
              <div
                className="execute-progress"
                role="progressbar"
                aria-label="ExecuteOne 서버 확인 동작"
                aria-valuemin={0}
                aria-valuemax={total}
                aria-valuenow={done}
              >
                <span style={{ width: `${(done / total) * 100}%` }} />
              </div>
            </div>
            <div className="execute-target">
              <span>
                <strong>물체 {record.object_id ?? "—"}</strong>
                <ArrowRight size={15} aria-hidden="true" />
                <span>{record.destination_id || "목적지 미확인"}</span>
              </span>
              <span className="execute-target-evidence">
                <span>
                  선택 팔{" "}
                  <strong>
                    {record.selected_arm === "left"
                      ? "왼팔"
                      : record.selected_arm === "right"
                        ? "오른팔"
                        : record.selected_arm || "미확인"}
                  </strong>
                </span>
                <span
                  className="execute-object-state"
                  data-risk={
                    record.object_state === "HELD" ||
                    record.object_state === "UNKNOWN"
                  }
                  title={`서버 보고: ${record.object_state || "없음"}`}
                >
                  물체 {objectLabels[record.object_state] || "미확인"}
                </span>
                {terminal && (
                  <span>
                    팔 복귀{" "}
                    <strong>
                      {record.arm_recovered === true ? "확인" : "미확인"}
                    </strong>{" "}
                    ·{" "}
                    <strong title="안착 상태">
                      {placementLabels[record.placement_state] ||
                        "안착 정보 없음"}
                    </strong>
                  </span>
                )}
              </span>
            </div>
            {(failure || record.failed_stage || record.cancel_mode) && (
              <div
                className="execute-alert"
                data-tone={failure ? "failed" : "last"}
                role="status"
              >
                <TriangleAlert size={19} aria-hidden="true" />
                <div>
                  <strong>
                    {failure
                      ? `${resultLabels[record.status] || record.status} · ${record.error_code || "오류 코드 없음"}`
                      : record.cancel_mode
                        ? `취소 방식 · ${record.cancel_mode}`
                        : "실패 위치 보고"}
                  </strong>
                  <p>
                    {record.failed_stage &&
                      `${stageLabel(record.failed_stage)}${record.failed_substage ? ` / ${stepLabel(record.failed_substage)}` : ""}`}
                    {record.message && ` · ${record.message}`}
                  </p>
                </div>
              </div>
            )}
            {stageIndex < 0 && !terminal && (
              <p className="execute-context-note">
                <Activity size={15} />
                {stageLabel(record.stage)}
                {record.substage && ` · ${stepLabel(record.substage)}`}
              </p>
            )}
            <div className="execute-phases-heading">
              <h3>조작 단계</h3>
              <p>
                <span className="execute-key active" />
                진행 <span className="execute-key completed" />
                완료 <span className="execute-key failed" />
                실패 <span className="execute-key last" />
                마지막 관측
              </p>
            </div>
            <ol className="execute-phases" aria-label="ExecuteOne 조작 단계">
              {stages.map(([id, label, steps], index) => {
                const state = phaseState(id, steps);
                const PhaseIcon = stageIcons[id];
                return (
                  <li
                    key={id}
                    data-stage={id}
                    data-state={state}
                    aria-current={state === "active" ? "step" : undefined}
                  >
                    <div className="execute-phase-heading">
                      <span className="execute-phase-icon">
                        {state === "completed" ? (
                          <Check size={19} />
                        ) : state === "failed" ? (
                          <X size={19} />
                        ) : (
                          <PhaseIcon size={19} />
                        )}
                      </span>
                      <div>
                        <span className="execute-phase-index">
                          STEP {String(index + 1).padStart(2, "0")}
                        </span>
                        <h4>{label}</h4>
                      </div>
                      <span className="execute-phase-state">
                        {stateLabel[state]}
                      </span>
                    </div>
                    <ul>
                      {steps.map(([step, label]) => {
                        const state = substepState(step);
                        return (
                          <li
                            key={step}
                            data-state={state}
                            data-substage={step}
                            aria-current={
                              state === "active" ? "step" : undefined
                            }
                            title={step}
                          >
                            {state === "completed" ? (
                              <CheckCircle2 size={15} />
                            ) : state === "failed" ? (
                              <X size={15} />
                            ) : state === "active" ? (
                              <span className="execute-active-dot" />
                            ) : (
                              <Circle size={14} />
                            )}
                            <span>{label}</span>
                            {state === "active" && <b>진행 중</b>}
                            {state === "failed" && <b>실패</b>}
                            {state === "last" && <b>마지막 관측</b>}
                          </li>
                        );
                      })}
                    </ul>
                  </li>
                );
              })}
            </ol>
            <p className="execute-observation-note">
              <Clock3 size={14} />
              {replay
                ? "기록 시점의 관측"
                : current
                  ? "현재 미션과 조작 서버를 연결해 관측 중"
                  : "마지막 실행 기록"}{" "}
              · 수신 {age(timeAt, run.received_at).toFixed(1)}초 전
              {!terminal &&
                !current &&
                ` · ${!runtimeFresh ? "Runtime 수신 중단" : !serverPresent ? "서버 연결 미확인" : "현재 실행과 연결되지 않음"}`}
              {run.partial && " · 일부 구간 미관측"}
              {run.clock_regressed && " · 서버 시각 역행: 시간 미표시"}
            </p>
          </>
        )}
        <footer className="execute-detail-footer">
          <p>
            Action 서버가 보고한 진행입니다. 청소 BT의 실행·완료 판정은 Mission
            Manager가 최종 결과를 검증해 처리합니다.
          </p>
          <details>
            <summary>실행 ID와 원본 기록</summary>
            <pre>
              {JSON.stringify(
                {
                  mission_id: mission || null,
                  expected_execution_id: execution || null,
                  bt_status:
                    runtime.bt_nodes?.find(
                      (node: Data) => node.id === "ExecuteOne",
                    )?.status || null,
                  record: run?.record || null,
                },
                null,
                2,
              )}
            </pre>
          </details>
        </footer>
      </div>
    </section>
  );
}
