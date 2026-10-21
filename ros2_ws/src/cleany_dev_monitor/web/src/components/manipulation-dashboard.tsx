import { useState, type ReactNode } from "react";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Tabs, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { Panel } from "@/components/panel";
import { ManipulationRequest } from "@/components/manipulation-request";
import { OperatorControls } from "@/components/operator-controls";
import { type Row, time } from "@/lib/monitor";
import {
  Activity,
  ArrowRight,
  Box,
  Check,
  CheckCircle2,
  CircleDot,
  Clock3,
  FileClock,
  Hand,
  Layers3,
  ListChecks,
  PackageCheck,
  Radio,
  ShieldCheck,
  Timer,
  Trash2,
  TriangleAlert,
  X,
} from "lucide-react";
import {
  stages,
  stageLabel,
  stageIcons,
  resultLabels,
  objectLabels,
  placementLabels,
  extra,
  manipulationObservation,
  currentExecution,
  missionExecutions,
  normalSteps,
  normalStepIds,
  stepLabel,
  type Run,
} from "@/lib/manipulation";
import "./manipulation.css";

type Props = {
  rows: Row[];
  timeAt: number;
  replay: boolean;
  connected: boolean;
  openTopic: (topic: string) => void;
  seek: (at: number) => void;
};

function StateValue({
  value,
  labels,
}: {
  value?: string;
  labels: Record<string, string>;
}) {
  return (
    <span className="manip-state-value">
      <span>{value ? labels[value] || value : "미확인"}</span>
      {value && <code>{value}</code>}
    </span>
  );
}
function StageIcon({ stage, size = 20 }: { stage: string; size?: number }) {
  const Icon = stageIcons[stage] || Activity;
  return <Icon size={size} strokeWidth={1.8} aria-hidden="true" />;
}
function ProgressRing({
  completed,
  total,
}: {
  completed: number;
  total: number;
}) {
  const percent = Math.round((completed / total) * 100);
  return (
    <div
      className="manip-progress-ring"
      role="progressbar"
      aria-label="서버가 확인한 동작 진행률"
      aria-valuenow={completed}
      aria-valuemin={0}
      aria-valuemax={total}
    >
      <svg viewBox="0 0 100 100" aria-hidden="true">
        <circle className="ring-track" cx="50" cy="50" r="42" />
        <circle
          className="ring-value"
          cx="50"
          cy="50"
          r="42"
          pathLength="100"
          strokeDasharray={`${percent} 100`}
        />
      </svg>
      <strong>
        {percent}
        <span>%</span>
      </strong>
    </div>
  );
}
const seconds = (value: number | null) =>
  value === null ? "—" : `${value.toFixed(1)}초`;
const confirmed = (value: unknown) =>
  value === true ? "확인됨" : value === false ? "미확인" : "정보 없음";
const tone = (status: string) =>
  status === "SUCCESS"
    ? "good"
    : ["FAILED", "FATAL"].includes(status)
      ? "bad"
      : "warn";
const age = (at: number, received: number) => Math.max(0, at - received);
function Tag({ children, kind = "" }: { children: ReactNode; kind?: string }) {
  return (
    <Badge variant="secondary" className={`badge ${kind}`}>
      {children}
    </Badge>
  );
}
function Facts({ items }: { items: [string, ReactNode][] }) {
  return (
    <dl className="facts manip-facts">
      {items.map(([label, value]) => (
        <div key={label}>
          <dt>{label}</dt>
          <dd>{value}</dd>
        </div>
      ))}
    </dl>
  );
}
function elapsed(run: Run, end: number): number | null {
  const start = run.record.accepted_at_ns;
  return !run.clock_regressed && start > 0 && end >= start
    ? (end - start) / 1e9
    : null;
}

export function ManipulationDashboard({
  rows,
  timeAt,
  replay,
  connected,
  openTopic,
  seek,
}: Props) {
  const [tab, setTab] = useState("overview");
  const [selected, setSelected] = useState("");
  const observation = manipulationObservation(rows, timeAt, replay, connected);
  const { runtime, runs, topic, runtimeFresh, summary } = observation;
  const { mission, automatic } = missionExecutions(observation);
  const pendingRequest =
    observation.serverMode &&
    observation.request?.execution_id &&
    (observation.request.execution_id === runs[0]?.record.execution_id ||
      observation.request.started_at_ns >
        (runs[0]?.record.accepted_at_ns || 0));
  const selectedRun =
    runs.find((run) => run.record.execution_id === selected) ||
    (pendingRequest
      ? runs.find(
          (run) =>
            run.record.execution_id === observation.request!.execution_id,
        )
      : mission
        ? automatic
        : runs[0]);
  const record = selectedRun?.record || {};
  const current = currentExecution(selectedRun, observation);
  // Extrapolate only a correlated live observation; freeze stale/finished evidence.
  const end =
    Number(record.updated_at_ns || 0) +
    (current && selectedRun ? age(timeAt, selectedRun.received_at) * 1e9 : 0);
  const completed: string[] = Array.isArray(record.completed_substages)
    ? record.completed_substages
    : [];
  const totalSteps = normalSteps.length;
  const doneCount = new Set(completed.filter((id) => normalStepIds.has(id)))
    .size;
  const unknownSteps = completed.filter((id) => !normalStepIds.has(id));
  const gate = !runtimeFresh
    ? "미확인"
    : runtime.safe_to_drive === true
      ? "조작 기준 주행 조건 충족"
      : "조작 기준 주행 차단";
  const gateReason = !runtimeFresh
    ? observation.serverMode
      ? "실물 조작 서버를 직접 관측하고 있습니다. 주행 판단은 Mission Runtime 연결 후 확인할 수 있습니다."
      : "Runtime 상태가 오래되었거나 수신되지 않았습니다."
    : runtime.drive_block_reason ||
      runtime.reason ||
      "Runtime이 보고한 주행 조건을 표시합니다.";
  const tabs = {
    overview: "조작 개요",
    stages: "단계 진행",
    timeline: "실행 타임라인",
    safety: "취소·주행 조건",
    compare: "실행 비교",
  };
  const totals = new Map<string, number>();
  for (const interval of selectedRun?.intervals || []) {
    const stop = interval.end_ns ?? (current ? end : record.updated_at_ns);
    if (
      selectedRun &&
      !selectedRun.clock_regressed &&
      stop >= interval.start_ns
    )
      totals.set(
        interval.stage,
        (totals.get(interval.stage) || 0) + (stop - interval.start_ns) / 1e9,
      );
  }
  const observationLabel = record.has_result
    ? "완료된 실행"
    : current
      ? record.cancel_mode
        ? "취소 처리 중"
        : replay
          ? "기록 시점 진행"
          : "진행 중"
      : "진행 상태 미확인";
  const visualTone = record.has_result
    ? tone(record.status)
    : current
      ? "info"
      : "neutral";
  const activeStepLabel = stepLabel(record.substage);
  const stageState = (id: string, steps: [string, string][]) =>
    record.failed_stage === id
      ? "failed"
      : current && record.stage === id
        ? "active"
        : steps.every(([node]) => completed.includes(node))
          ? "completed"
          : "pending";

  const targetPanel = (
    <Panel
      title="대상과 실행 결과"
      className="manip-target-panel"
      aside={<PackageCheck size={18} aria-hidden="true" />}
    >
      <div className="manip-object-route">
        <div>
          <span className="manip-route-icon">
            <Box size={22} aria-hidden="true" />
          </span>
          <small>수거 대상</small>
          <strong>물체 {record.object_id ?? "—"}</strong>
        </div>
        <ArrowRight size={20} aria-hidden="true" />
        <div>
          <span className="manip-route-icon">
            <Trash2 size={22} aria-hidden="true" />
          </span>
          <small>목적지</small>
          <strong>{record.destination_id || "—"}</strong>
        </div>
      </div>
      <Facts
        items={[
          [
            "선택한 팔",
            record.selected_arm === "left"
              ? "왼팔"
              : record.selected_arm === "right"
                ? "오른팔"
                : record.selected_arm || "미결정",
          ],
          [
            "안착 상태",
            <StateValue
              value={record.placement_state}
              labels={placementLabels}
            />,
          ],
          [
            "최종 결과",
            record.has_result ? (
              <Tag kind={tone(record.status)}>{record.status || "미확인"}</Tag>
            ) : (
              "최종 결과 대기"
            ),
          ],
          [
            "물체 상태",
            <StateValue value={record.object_state} labels={objectLabels} />,
          ],
        ]}
      />
      <div className="manip-proof-strip" aria-label="선택한 실행의 확인 결과">
        <span data-confirmed={record.arm_recovered === true}>
          <ShieldCheck size={15} aria-hidden="true" />팔 복귀{" "}
          {confirmed(record.arm_recovered)}
        </span>
        <span data-confirmed={record.stop_confirmed === true}>
          <CheckCircle2 size={15} aria-hidden="true" />
          정지 {confirmed(record.stop_confirmed)}
        </span>
      </div>
      {record.error_code && record.error_code !== "NONE" && (
        <div className="manip-failure-note">
          <TriangleAlert size={17} aria-hidden="true" />
          <span>
            <strong>{record.error_code}</strong>
            {record.failed_stage && <> · {stageLabel(record.failed_stage)}</>}
            {record.failed_substage && <code>{record.failed_substage}</code>}
          </span>
        </div>
      )}
      {record.cancel_mode && (
        <p className="manip-note">
          취소 방식 <Tag kind="warn">{record.cancel_mode}</Tag>
        </p>
      )}
      <details className="manip-metadata">
        <summary>실행 정보와 메시지</summary>
        <Facts
          items={[
            ["미션", <code>{record.mission_id || "—"}</code>],
            ["실행 ID", <code>{record.execution_id || "—"}</code>],
            ["관측 snapshot", record.snapshot_id || "—"],
            ["서버 메시지", record.message || "—"],
          ]}
        />
      </details>
    </Panel>
  );
  const safetyPanel = (
    <Panel
      title="현재 주행 조건"
      className="manip-safety-panel"
      aside={<ShieldCheck size={18} aria-hidden="true" />}
    >
      <section className="manip-gate" aria-label="현재 Runtime 주행 판단">
        <span className="manip-gate-icon">
          {runtimeFresh && runtime.safe_to_drive === true ? (
            <ShieldCheck size={22} aria-hidden="true" />
          ) : (
            <Radio size={22} aria-hidden="true" />
          )}
        </span>
        <div>
          <Tag
            kind={
              runtimeFresh && runtime.safe_to_drive === true ? "good" : "warn"
            }
          >
            {gate}
          </Tag>
          <p>{gateReason}</p>
        </div>
      </section>
      <Facts
        items={[
          [
            "현재 주행 모듈 준비",
            runtimeFresh
              ? runtime.modules?.navigation?.ready === true
                ? "준비됨"
                : runtime.modules?.navigation?.ready === false
                  ? "미준비"
                  : "정보 없음"
              : "미확인",
          ],
          [
            "현재 베이스 정지",
            runtimeFresh
              ? confirmed(runtime.modules?.navigation?.stopped)
              : "미확인",
          ],
          [
            "현재 미션 수락 준비",
            runtimeFresh
              ? runtime.ready === true
                ? "준비됨"
                : runtime.ready === false
                  ? runtime.reason || "미준비"
                  : "정보 없음"
              : "미확인",
          ],
          [
            "사람 확인 필요",
            record.human_confirmation_required === true
              ? "서버가 확인을 요청함"
              : "보고 없음",
          ],
        ]}
      />
      <p className="manip-note">
        현재 Runtime의 보고입니다. 과거 실행의 성공이 현재 주행을 허용하지는
        않습니다.
      </p>
      <details className="manip-metadata">
        <summary>Runtime 진단 정보</summary>
        <Facts
          items={[
            [
              "현재 Runtime 상태",
              runtimeFresh ? runtime.state || "—" : "미확인",
            ],
            [
              "현재 주행 출처",
              runtimeFresh
                ? runtime.execution_profile?.navigation || "미확인"
                : "미확인",
            ],
          ]}
        />
      </details>
    </Panel>
  );

  const executionPicker = runs.length > 0 && (
    <label className="manip-execution-picker">
      <span>실행 기록</span>
      <select
        aria-label="조작 실행 선택"
        value={
          runs.some((run) => run.record.execution_id === selected)
            ? selected
            : ""
        }
        onChange={(event) => setSelected(event.target.value)}
      >
        <option value="">현재 / 최근 실행 자동 선택</option>
        {runs.map(({ record: item }) => (
          <option key={item.execution_id} value={item.execution_id}>
            물체 {item.object_id} ·{" "}
            {item.has_result
              ? resultLabels[item.status] || item.status
              : item.record_state}{" "}
            · {item.execution_id.slice(0, 12)}
          </option>
        ))}
      </select>
    </label>
  );

  return (
    <div className="manipulation-dashboard">
      <OperatorControls
        enabled={
          rows.find((row) => row.kind === "config")?.data.control_enabled ===
          true
        }
        connected={connected}
        replay={replay}
      />
      {observation.serverMode && observation.request && (
        <ManipulationRequest
          request={observation.request}
          fresh={observation.requestFresh}
        />
      )}
      <div className="manip-toolbar">
        <Tabs value={tab} onValueChange={setTab}>
          <TabsList className="manip-tabs">
            {Object.entries(tabs).map(([id, name]) => (
              <TabsTrigger key={id} value={id}>
                {id === "overview" ? (
                  <Layers3 size={15} aria-hidden="true" />
                ) : id === "stages" ? (
                  <ListChecks size={15} aria-hidden="true" />
                ) : id === "timeline" ? (
                  <Clock3 size={15} aria-hidden="true" />
                ) : id === "safety" ? (
                  <ShieldCheck size={15} aria-hidden="true" />
                ) : (
                  <FileClock size={15} aria-hidden="true" />
                )}
                {name}
              </TabsTrigger>
            ))}
          </TabsList>
        </Tabs>
        <Button variant="outline" size="sm" onClick={() => openTopic(topic)}>
          실행 토픽 보기
        </Button>
      </div>
      {!selectedRun ? (
        <>
          <Panel title="조작 실행 기록 대기">
            <p className="manip-note">
              {observation.serverMode
                ? "실물 조작을 실행하면 서버가 보고하는 단계·시간·결과를 표시합니다."
                : "현재 미션과 execution ID가 일치하는 서버 기록이 수신되면 단계·시간·결과를 표시합니다. 이전 실행은 기록에서 선택할 수 있습니다."}
            </p>
            <code>{topic}</code>
            {executionPicker}
          </Panel>
          {safetyPanel}
        </>
      ) : (
        <>
          <section
            className="manip-summary"
            data-tone={visualTone}
            aria-label="선택 실행 요약"
          >
            <div className="manip-hero-main">
              <div className="manip-hero-meta">
                <Tag
                  kind={
                    current
                      ? "info"
                      : record.has_result
                        ? tone(record.status)
                        : "warn"
                  }
                >
                  {observationLabel}
                </Tag>
                <span className="manip-source-label">
                  {record.execution_profile === "mock"
                    ? "Mock"
                    : record.execution_profile || "출처 미확인"}
                </span>
                <span>물체 {record.object_id ?? "—"}</span>
              </div>
              <div className="manip-hero-heading">
                <span className="manip-hero-icon">
                  {record.has_result ? (
                    record.status === "SUCCESS" ? (
                      <CheckCircle2 size={28} aria-hidden="true" />
                    ) : record.status === "CANCELED" ? (
                      <Hand size={28} aria-hidden="true" />
                    ) : (
                      <TriangleAlert size={28} aria-hidden="true" />
                    )
                  ) : (
                    <StageIcon stage={record.stage} size={28} />
                  )}
                </span>
                <div>
                  <small>
                    {record.has_result
                      ? "마지막 실행 결과"
                      : current
                        ? replay
                          ? "기록 시점의 동작"
                          : "현재 동작"
                        : "마지막 관측 동작"}
                  </small>
                  <h2>
                    {record.has_result
                      ? resultLabels[record.status] || "결과 미확인"
                      : stageLabel(record.stage)}
                  </h2>
                  <p className="manip-current-step">
                    {activeStepLabel && <span>{activeStepLabel}</span>}
                    <code>
                      {record.substage || record.status || record.stage || "—"}
                    </code>
                  </p>
                </div>
              </div>
              {executionPicker}
            </div>
            <div className="manip-hero-metrics">
              <div className="manip-completion">
                <ProgressRing completed={doneCount} total={totalSteps} />
                <div>
                  <small>확인된 동작</small>
                  <strong>
                    {doneCount}
                    <span> / {totalSteps}</span>
                  </strong>
                  <span>
                    {doneCount}/{totalSteps}개 동작 완료
                  </span>
                </div>
              </div>
              <div className="manip-time-metric">
                <span className="manip-metric-icon">
                  <Timer size={19} aria-hidden="true" />
                </span>
                <div>
                  <small>
                    {current ? "관측 경과 시간" : "기록된 경과 시간"}
                  </small>
                  <strong>{seconds(elapsed(selectedRun, end))}</strong>
                </div>
              </div>
            </div>
          </section>
          <p
            className={`manip-note manip-observation ${selectedRun.partial || selectedRun.clock_regressed ? "is-warning" : ""}`}
          >
            {replay ? (
              <FileClock size={14} aria-hidden="true" />
            ) : current ? (
              <Radio size={14} aria-hidden="true" />
            ) : (
              <Clock3 size={14} aria-hidden="true" />
            )}
            {replay
              ? "기록 시점의 관측"
              : current
                ? "서버와 현재 미션을 연결해 관측 중"
                : "마지막 실행 기록"}{" "}
            · 마지막 기록 수신 {age(timeAt, selectedRun.received_at).toFixed(1)}
            초 전
            {selectedRun.partial
              ? " · 일부 구간 미관측: 세부 시간은 불완전합니다."
              : ""}
            {selectedRun.clock_regressed
              ? " · 서버 시각 역행으로 소요 시간을 표시하지 않습니다."
              : ""}
          </p>
          {tab === "overview" && (
            <Panel
              title="실행 흐름"
              className="manip-flow-panel"
              aside={
                <span className="manip-panel-caption">서버가 보고한 9단계</span>
              }
            >
              <ol className="manip-stage-rail" aria-label="전체 조작 단계">
                {stages.map(([id, label, steps], index) => {
                  const state = stageState(id, steps);
                  return (
                    <li
                      key={id}
                      data-state={state}
                      aria-current={state === "active" ? "step" : undefined}
                    >
                      <span className="manip-rail-node">
                        {state === "completed" ? (
                          <Check size={17} aria-hidden="true" />
                        ) : state === "failed" ? (
                          <X size={17} aria-hidden="true" />
                        ) : (
                          <StageIcon stage={id} size={17} />
                        )}
                      </span>
                      <span className="manip-rail-index">
                        {String(index + 1).padStart(2, "0")}
                      </span>
                      <strong>{label}</strong>
                    </li>
                  );
                })}
              </ol>
            </Panel>
          )}
          {(tab === "overview" || tab === "safety") && (
            <div className="manip-two">
              {targetPanel}
              {safetyPanel}
            </div>
          )}
          {tab === "stages" && (
            <Panel
              title="서버 단계 진행"
              aside={
                <Tag>
                  {doneCount} / {totalSteps} 동작 확인
                </Tag>
              }
            >
              <p className="manip-note">
                청소 BT의 ExecuteOne 내부에서 관측한 Action 단계입니다. 완료
                표시는 서버의 completed_substages를 사용합니다.
              </p>
              <ol className="manip-stages">
                {stages.map(([id, label, steps], index) => {
                  const finished = steps.every(([node]) =>
                    completed.includes(node),
                  );
                  const failed = record.failed_stage === id;
                  const active = current && record.stage === id;
                  return (
                    <li
                      key={id}
                      className={
                        failed
                          ? "failed"
                          : active
                            ? "active"
                            : finished
                              ? "completed"
                              : ""
                      }
                    >
                      <div className="manip-stage-title">
                        <span className="manip-stage-icon">
                          <StageIcon stage={id} />
                        </span>
                        <span className="manip-stage-name">
                          <small>
                            STEP {String(index + 1).padStart(2, "0")}
                          </small>
                          <strong>{label}</strong>
                        </span>
                        <Tag
                          kind={
                            failed
                              ? "bad"
                              : active
                                ? "info"
                                : finished
                                  ? "good"
                                  : ""
                          }
                        >
                          {failed
                            ? "실패"
                            : active
                              ? "진행 중"
                              : finished
                                ? "완료"
                                : record.stage === id
                                  ? "마지막 관측"
                                  : "미관측"}
                        </Tag>
                      </div>
                      <div className="manip-stage-track" aria-hidden="true">
                        <i
                          style={{
                            width: `${(steps.filter(([node]) => completed.includes(node)).length / steps.length) * 100}%`,
                          }}
                        />
                      </div>
                      <ul>
                        {steps.map(([node, name]) => (
                          <li key={node}>
                            <span className="manip-step-name">
                              {record.failed_substage === node ? (
                                <X size={14} aria-hidden="true" />
                              ) : completed.includes(node) ? (
                                <CheckCircle2 size={14} aria-hidden="true" />
                              ) : (
                                <CircleDot size={14} aria-hidden="true" />
                              )}
                              {name}
                            </span>
                            <code>{node}</code>
                            <Tag
                              kind={
                                record.failed_substage === node
                                  ? "bad"
                                  : completed.includes(node)
                                    ? "good"
                                    : current && record.substage === node
                                      ? "info"
                                      : ""
                              }
                            >
                              {record.failed_substage === node
                                ? "실패"
                                : completed.includes(node)
                                  ? "완료"
                                  : current && record.substage === node
                                    ? "진행 중"
                                    : "미관측"}
                            </Tag>
                          </li>
                        ))}
                      </ul>
                    </li>
                  );
                })}
              </ol>
              {extra[record.stage] && (
                <p className="manip-note">
                  추가 단계: {stageLabel(record.stage)} ·{" "}
                  {record.substage || "—"}
                </p>
              )}
              {!!unknownSteps.length && (
                <p className="manip-note">
                  추가 완료 동작: {unknownSteps.join(", ")}
                </p>
              )}
            </Panel>
          )}
          {tab === "timeline" && (
            <Panel title="단계별 관측 시간">
              <p className="manip-note">
                서버 실행 시각 기준입니다. 진행 중 시간은 관측 경과값이며,
                기록을 클릭하면 해당 수신 시점으로 이동합니다.
              </p>
              <div className="manip-bars" aria-label="단계별 소요 시간">
                {[...totals].map(([stage, duration]) => (
                  <div key={stage}>
                    <span>{stageLabel(stage)}</span>
                    <div>
                      <i
                        style={{
                          width: `${Math.max(1, (duration / Math.max(1, ...totals.values())) * 100)}%`,
                        }}
                      />
                    </div>
                    <strong>{seconds(duration)}</strong>
                  </div>
                ))}
              </div>
              <div className="manip-table">
                <table>
                  <thead>
                    <tr>
                      <th>관측 시각</th>
                      <th>단계</th>
                      <th>세부 동작</th>
                      <th>관측 구간</th>
                      <th>기록</th>
                    </tr>
                  </thead>
                  <tbody>
                    {selectedRun.intervals.map((interval, index) => {
                      const stop =
                        interval.end_ns ??
                        (current ? end : record.updated_at_ns);
                      const duration =
                        !selectedRun.clock_regressed &&
                        stop >= interval.start_ns
                          ? (stop - interval.start_ns) / 1e9
                          : null;
                      return (
                        <tr key={index}>
                          <td>{time(interval.received_at)}</td>
                          <td>{stageLabel(interval.stage)}</td>
                          <td>
                            <code>{interval.substage || "—"}</code>
                          </td>
                          <td>
                            {seconds(duration)}
                            {interval.partial
                              ? " · 일부 미관측"
                              : interval.end_ns === null
                                ? current
                                  ? " · 진행 중"
                                  : " · 마지막 관측까지"
                                : ""}
                          </td>
                          <td>
                            <Button
                              variant="ghost"
                              size="sm"
                              aria-label={`${interval.substage || interval.stage} 관측 시점 보기`}
                              onClick={() => seek(interval.received_at)}
                            >
                              시점 보기
                            </Button>
                          </td>
                        </tr>
                      );
                    })}
                  </tbody>
                </table>
              </div>
              {!selectedRun.intervals.length && (
                <p className="manip-note">단계별 시간 기록 없음</p>
              )}
            </Panel>
          )}
          {tab === "compare" && (
            <Panel
              title="실행 기록 비교"
              aside={<Tag>최근 {runs.length}개</Tag>}
            >
              <p className="manip-note">
                현재 Monitor가 관측한 실행입니다.{" "}
                {summary?.data.evicted
                  ? `${summary.data.evicted}개의 이전 실행은 메모리 목록에서 제외되었습니다.`
                  : ""}
              </p>
              <div className="manip-table">
                <table>
                  <thead>
                    <tr>
                      <th>물체</th>
                      <th>실행 ID</th>
                      <th>출처</th>
                      <th>결과</th>
                      <th>기록된 시간</th>
                      <th>물체 상태</th>
                      <th>팔 복귀 / 정지</th>
                      <th>오류</th>
                    </tr>
                  </thead>
                  <tbody>
                    {runs.map((run) => (
                      <tr
                        key={run.record.execution_id}
                        className={run === selectedRun ? "manip-selected" : ""}
                      >
                        <td>{run.record.object_id}</td>
                        <td>
                          <Button
                            variant="ghost"
                            size="sm"
                            onClick={() => setSelected(run.record.execution_id)}
                            title={run.record.execution_id}
                          >
                            {run.record.execution_id.slice(0, 12)}
                          </Button>
                        </td>
                        <td>{run.record.execution_profile || "미확인"}</td>
                        <td>
                          <Tag
                            kind={
                              run.record.has_result
                                ? tone(run.record.status)
                                : "warn"
                            }
                          >
                            {run.record.has_result
                              ? run.record.status
                              : "최종 결과 없음"}
                          </Tag>
                        </td>
                        <td>
                          {seconds(elapsed(run, run.record.updated_at_ns))}
                          {run.partial && <small>일부 구간 미관측</small>}
                        </td>
                        <td>{run.record.object_state || "미확인"}</td>
                        <td>
                          {confirmed(run.record.arm_recovered)} /{" "}
                          {confirmed(run.record.stop_confirmed)}
                        </td>
                        <td>{run.record.error_code || "—"}</td>
                      </tr>
                    ))}
                  </tbody>
                </table>
              </div>
            </Panel>
          )}
          <details>
            <summary>선택 실행의 서버 원본 기록</summary>
            <pre>{JSON.stringify(record, null, 2)}</pre>
          </details>
        </>
      )}
    </div>
  );
}
