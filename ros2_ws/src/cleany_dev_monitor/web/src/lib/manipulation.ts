import { type Data, type Row } from "./monitor";
import {
  Box,
  CheckCircle2,
  Crosshair,
  Hand,
  PackageCheck,
  ScanSearch,
  ShieldCheck,
  Trash2,
  Undo2,
  type LucideIcon,
} from "lucide-react";

export type Interval = {
  stage: string;
  substage: string;
  start_ns: number;
  end_ns: number | null;
  partial: boolean;
  received_at: number;
};
export type Run = {
  record: Data;
  received_at: number;
  intervals: Interval[];
  partial: boolean;
  clock_regressed: boolean;
};
// Labels describe the Action protocol, rather than inventing an executor BT.
export const stages: [string, string, [string, string][]][] = [
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
export const normalSteps = stages.flatMap(([, , steps]) => steps);
export const normalStepIds = new Set(normalSteps.map(([id]) => id));
const stepLabels = new Map(normalSteps);
export const stepLabel = (id: string) => stepLabels.get(id) || id;
export const extra: Record<string, string> = {
  STOPPING: "정지 확인",
  FINALIZING: "결과 확정",
  RELEASING_IN_PLACE: "현재 위치에서 놓기",
  RECOVERING_ARM: "취소 후 팔 복귀",
};
export const stageLabel = (id: string) =>
  stages.find(([key]) => key === id)?.[1] || extra[id] || id || "—";
export const stageIcons: Record<string, LucideIcon> = {
  VALIDATING: ShieldCheck,
  PREPARING_TARGET: ScanSearch,
  APPROACHING: Crosshair,
  GRASPING: Hand,
  LIFTING: Box,
  TRANSPORTING: Trash2,
  PLACING: PackageCheck,
  RETURNING_ARM: Undo2,
  VERIFYING_PLACEMENT: CheckCircle2,
};
export const resultLabels: Record<string, string> = {
  SUCCESS: "수거 완료",
  CANCELED: "수거 취소",
  BLOCKED: "실행 차단",
  FAILED: "조작 실패",
  FATAL: "안전 오류",
};

export function manipulationObservation(
  rows: Row[],
  timeAt: number,
  replay: boolean,
  connected: boolean,
) {
  const runtimeRow = rows.find((row) => row.kind === "runtime");
  const runtime = runtimeRow?.data || {};
  const graphRow = rows.find((row) => row.kind === "graph");
  const config = rows.find((row) => row.kind === "config")?.data || {};
  const eventTopic =
    config.manipulation_events_topic || "mock/manipulation/execution_events";
  const topic = eventTopic.startsWith("/")
    ? eventTopic
    : "/" +
      [config.runtime_namespace?.replace(/^\/+|\/+$/g, ""), eventTopic]
        .filter(Boolean)
        .join("/");
  const fresh = (row: Row | undefined) =>
    !!row && Math.max(0, timeAt - row.received_at) < 3 && (replay || connected);
  const endpoint = graphRow?.data.topics?.find(
    (item: Data) => item.name === topic,
  );
  const summary = rows.find(
    (row) => row.kind === "manipulation" && row.key === "executions",
  );
  const runs: Run[] = summary?.data.executions || [];
  return {
    runtime,
    summary,
    runs,
    topic,
    runtimeFresh: fresh(runtimeRow),
    serverPresent: fresh(graphRow) && !!endpoint?.publishers?.length,
  };
}

export function currentExecution(
  run: Run | undefined,
  observation: ReturnType<typeof manipulationObservation>,
): boolean {
  const { runtime, runtimeFresh, serverPresent } = observation;
  const record = run?.record;
  return (
    !!record &&
    !record.has_result &&
    record.record_state === "ACTIVE" &&
    runtimeFresh &&
    serverPresent &&
    runtime.active_request?.mission_id === record.mission_id &&
    runtime.manipulation_feedback?.execution_id === record.execution_id &&
    (!runtime.proposal?.execution_id ||
      runtime.proposal.execution_id === record.execution_id) &&
    (runtime.state === "CANCELLING" ||
      (runtime.state === "WORKING" && runtime.bt_stage === "ExecuteOne"))
  );
}

export function missionExecutions(
  observation: ReturnType<typeof manipulationObservation>,
) {
  const { runtime, runs } = observation;
  const mission =
    runtime.active_request?.mission_id ||
    runtime.last_result?.request?.mission_id;
  const matching = mission
    ? runs.filter((run) => run.record.mission_id === mission)
    : [];
  const execution =
    runtime.proposal?.execution_id ||
    runtime.manipulation_feedback?.execution_id ||
    runtime.manipulation?.execution_id;
  // A goal may be journaled before its first server event. Never substitute an
  // earlier object's result while waiting for that exact execution.
  const automatic = execution
    ? matching.find((run) => run.record.execution_id === execution)
    : [...matching].sort(
        (a, b) => b.record.updated_at_ns - a.record.updated_at_ns,
      )[0];
  return { mission, matching, automatic, execution };
}

export const objectLabels: Record<string, string> = {
  NOT_TOUCHED: "접촉 전",
  HELD: "보유 중",
  LEFT_GRIPPER: "그리퍼 이탈",
  UNKNOWN: "미확인",
};
export const placementLabels: Record<string, string> = {
  CONFIRMED: "안착 확인",
  NOT_CHECKED: "검증 전",
  NOT_CONFIRMED: "안착 미확인",
  UNKNOWN: "미확인",
};
