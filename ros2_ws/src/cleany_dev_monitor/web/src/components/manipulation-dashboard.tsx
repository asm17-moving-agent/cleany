import { useMemo, useState, type ReactNode } from "react";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Tabs, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { Panel } from "@/components/panel";
import { type Data, type Row, time } from "@/lib/monitor";
import "./manipulation.css";

type Interval = {
  stage: string;
  substage: string;
  start_ns: number;
  end_ns: number | null;
  partial: boolean;
  received_at: number;
};
type Run = {
  record: Data;
  received_at: number;
  intervals: Interval[];
  partial: boolean;
  clock_regressed: boolean;
};
type Props = {
  rows: Row[];
  timeAt: number;
  replay: boolean;
  connected: boolean;
  openTopic: (topic: string) => void;
  seek: (at: number) => void;
};

// Labels describe the Action protocol, rather than inventing an executor BT.
const stages: [string, string, [string, string][]][] = [
  ["VALIDATING", "실행 준비 확인", [["ValidateGoal", "모델과 실행 준비 확인"]]],
  [
    "PREPARING_TARGET",
    "대상 확인·파지 계획",
    [
      ["PrepareTarget", "대상 관측 확인"],
      ["ReconstructTarget", "선택 물체 3D 복원"],
      ["GenerateGrasp", "파지 후보 생성"],
      ["SelectArmAndPath", "팔과 경로 결정"],
    ],
  ],
  [
    "APPROACHING",
    "물체 접근",
    [
      ["MoveToPregrasp", "잡기 전 위치 이동"],
      ["ApproachObject", "물체 접근"],
    ],
  ],
  [
    "GRASPING",
    "잡기·파지 확인",
    [
      ["GraspObject", "그리퍼 닫기"],
      ["ConfirmGrasp", "파지 확인"],
    ],
  ],
  [
    "LIFTING",
    "들어 올리기",
    [
      ["LiftObject", "물체 들어 올리기"],
      ["ConfirmHeld", "보유 상태 확인"],
    ],
  ],
  ["TRANSPORTING", "수거함 운반", [["CarryObject", "수거함 이동"]]],
  [
    "PLACING",
    "놓기·이탈 확인",
    [
      ["CheckPlacementTarget", "놓을 위치 확인"],
      ["OpenGripperAtDestination", "그리퍼 열기"],
      ["ConfirmRelease", "물체 이탈 확인"],
    ],
  ],
  ["RETURNING_ARM", "팔 복귀", [["ReturnArm", "팔 복귀"]]],
  [
    "VERIFYING_PLACEMENT",
    "안착 확인",
    [["VerifyPlacedObject", "수거함 내부 확인"]],
  ],
];
const extra: Record<string, string> = {
  STOPPING: "정지 확인",
  FINALIZING: "결과 확정",
  RELEASING_IN_PLACE: "현재 위치에서 놓기",
  RECOVERING_ARM: "취소 후 팔 복귀",
};
const stageLabel = (id: string) =>
  stages.find(([key]) => key === id)?.[1] || extra[id] || id || "—";
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
    <dl className="facts">
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
  const summary = rows.find(
    (row) => row.kind === "manipulation" && row.key === "executions",
  );
  const runs: Run[] = useMemo(() => summary?.data.executions || [], [summary]);
  const runtimeRow = rows.find((row) => row.kind === "runtime");
  const runtime = runtimeRow?.data || {};
  const config = rows.find((row) => row.kind === "config")?.data || {};
  const graphRow = rows.find((row) => row.kind === "graph");
  const eventTopic =
    config.manipulation_events_topic || "mock/manipulation/execution_events";
  const topic = eventTopic.startsWith("/")
    ? eventTopic
    : "/" +
      [config.runtime_namespace?.replace(/^\/+|\/+$/g, ""), eventTopic]
        .filter(Boolean)
        .join("/");
  const endpoint = graphRow?.data.topics?.find(
    (item: Data) => item.name === topic,
  );
  const runtimeFresh =
    !!runtimeRow &&
    age(timeAt, runtimeRow.received_at) < 3 &&
    (replay || connected);
  const serverPresent =
    !!graphRow &&
    age(timeAt, graphRow.received_at) < 3 &&
    !!endpoint?.publishers?.length &&
    (replay || connected);
  const selectedRun =
    runs.find((run) => run.record.execution_id === selected) ||
    runs.find(
      (run) =>
        run.record.execution_id === runtime.manipulation_feedback?.execution_id,
    ) ||
    runs[0];
  const record = selectedRun?.record || {};
  const current =
    !!selectedRun &&
    !record.has_result &&
    record.record_state === "ACTIVE" &&
    runtimeFresh &&
    serverPresent &&
    runtime.active_request?.mission_id === record.mission_id &&
    runtime.manipulation_feedback?.execution_id === record.execution_id &&
    ["WORKING", "CANCELLING"].includes(runtime.state);
  // Extrapolate only a correlated live observation; freeze stale/finished evidence.
  const end =
    Number(record.updated_at_ns || 0) +
    (current ? age(timeAt, selectedRun.received_at) * 1e9 : 0);
  const completed: string[] = Array.isArray(record.completed_substages)
    ? record.completed_substages
    : [];
  const totalSteps = stages.reduce(
    (count, [, , steps]) => count + steps.length,
    0,
  );
  const normalIds = stages.flatMap(([, , steps]) => steps.map(([id]) => id));
  const doneCount = completed.filter((id) => normalIds.includes(id)).length;
  const unknownSteps = completed.filter((id) => !normalIds.includes(id));
  const gate = !runtimeFresh
    ? "미확인"
    : runtime.safe_to_drive === true
      ? "조작 기준 주행 조건 충족"
      : "조작 기준 주행 차단";
  const gateReason = !runtimeFresh
    ? "Runtime 상태가 오래되었거나 수신되지 않았습니다."
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
    if (!selectedRun.clock_regressed && stop >= interval.start_ns)
      totals.set(
        interval.stage,
        (totals.get(interval.stage) || 0) + (stop - interval.start_ns) / 1e9,
      );
  }

  const targetPanel = (
    <Panel title="대상과 실행 결과">
      <Facts
        items={[
          ["미션", record.mission_id || "—"],
          ["실행 ID", <code>{record.execution_id || "—"}</code>],
          ["물체 ID", record.object_id ?? "—"],
          ["관측 snapshot", record.snapshot_id || "—"],
          ["목적지", record.destination_id || "—"],
          ["선택한 팔", record.selected_arm || "미결정"],
          ["조작 출처", record.execution_profile || "미확인"],
          ["안착 상태", record.placement_state || "미확인"],
          [
            "최종 결과",
            record.has_result ? (
              <Tag kind={tone(record.status)}>{record.status || "미확인"}</Tag>
            ) : (
              "최종 결과 대기"
            ),
          ],
          [
            "오류",
            record.error_code && record.error_code !== "NONE"
              ? record.error_code
              : "—",
          ],
          [
            "실패 단계",
            record.failed_stage ? stageLabel(record.failed_stage) : "—",
          ],
          ["실패 세부 동작", record.failed_substage || "—"],
        ]}
      />
      {record.message && <p className="manip-note">{record.message}</p>}
    </Panel>
  );
  const safetyPanel = (
    <Panel title="취소·주행 차단 근거">
      <section className="manip-gate" aria-label="현재 Runtime 주행 판단">
        <Tag
          kind={
            runtimeFresh && runtime.safe_to_drive === true ? "good" : "warn"
          }
        >
          {gate}
        </Tag>
        <p>{gateReason}</p>
      </section>
      <Facts
        items={[
          ["현재 Runtime 상태", runtimeFresh ? runtime.state || "—" : "미확인"],
          [
            "현재 주행 출처",
            runtimeFresh
              ? runtime.execution_profile?.navigation || "미확인"
              : "미확인",
          ],
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
          ["선택 실행의 물체 상태", record.object_state || "정보 없음"],
          ["선택 실행의 팔 복귀", confirmed(record.arm_recovered)],
          ["선택 실행의 정지", confirmed(record.stop_confirmed)],
          ["취소 방식", record.cancel_mode || "요청 없음"],
          [
            "사람 확인 필요",
            record.human_confirmation_required === true
              ? "서버가 확인을 요청함"
              : "보고 없음",
          ],
        ]}
      />
      <p className="manip-note">
        조작 기준 주행 조건과 Nav2 준비는 현재 Runtime의 서로 다른 보고입니다.
        선택한 과거 실행의 성공이나 정지 결과로 현재 주행을 허용하지 않습니다.
      </p>
    </Panel>
  );

  return (
    <div className="manipulation-dashboard">
      <div className="manip-toolbar">
        <Tabs value={tab} onValueChange={setTab}>
          <TabsList className="manip-tabs">
            {Object.entries(tabs).map(([id, name]) => (
              <TabsTrigger key={id} value={id}>
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
        <Panel title="조작 실행 기록 대기">
          <p className="manip-note">
            서버의 실행 기록이 수신되면 단계·시간·결과를 표시합니다.
          </p>
          <code>{topic}</code>
        </Panel>
      ) : (
        <>
          <div className="manip-summary">
            <label>
              선택 실행
              <select
                aria-label="조작 실행 선택"
                value={record.execution_id}
                onChange={(event) => setSelected(event.target.value)}
              >
                {runs.map((run) => (
                  <option
                    key={run.record.execution_id}
                    value={run.record.execution_id}
                  >
                    물체 {run.record.object_id} · {run.record.execution_id} ·{" "}
                    {run.record.has_result
                      ? run.record.status
                      : run.record.record_state}
                  </option>
                ))}
              </select>
            </label>
            <div>
              <small>실행 관측</small>
              <Tag
                kind={
                  current
                    ? "info"
                    : record.has_result
                      ? tone(record.status)
                      : "warn"
                }
              >
                {record.has_result
                  ? "완료된 실행"
                  : current
                    ? record.cancel_mode
                      ? "취소 처리 중"
                      : replay
                        ? "기록 시점 진행"
                        : "진행 중"
                    : "진행 상태 미확인"}
              </Tag>
            </div>
            <div>
              <small>출처</small>
              <strong>
                {record.execution_profile === "mock"
                  ? "Mock"
                  : record.execution_profile || "미확인"}
              </strong>
            </div>
            <div>
              <small>현재 / 마지막 단계</small>
              <strong>{stageLabel(record.stage)}</strong>
              <code>{record.substage || "—"}</code>
            </div>
            <div>
              <small>{current ? "관측 경과 시간" : "기록된 경과 시간"}</small>
              <strong>{seconds(elapsed(selectedRun, end))}</strong>
            </div>
          </div>
          <p className="manip-note">
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
          {(tab === "overview" || tab === "safety") && (
            <div className="manip-two">
              {targetPanel}
              {safetyPanel}
            </div>
          )}
          {(tab === "overview" || tab === "stages") && (
            <Panel
              title="서버 단계 진행"
              aside={
                <Tag>
                  {doneCount}/{totalSteps}개 동작 완료
                </Tag>
              }
            >
              <p className="manip-note">
                청소 BT의 ExecuteOne 내부에서 관측한 Action 단계입니다. 완료
                표시는 서버의 completed_substages를 사용합니다.
              </p>
              <ol className="manip-stages">
                {stages.map(([id, label, steps]) => {
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
                        <strong>{label}</strong>
                        <code>{id}</code>
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
                      <ul>
                        {steps.map(([node, name]) => (
                          <li key={node}>
                            <span>{name}</span>
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
