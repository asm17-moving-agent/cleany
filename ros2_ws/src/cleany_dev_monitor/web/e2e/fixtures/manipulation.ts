import { type Page } from "@playwright/test";

export const beforeCarry = [
  "ValidateGoal",
  "PrepareTarget",
  "ReconstructTarget",
  "GenerateGrasp",
  "SelectArmAndPath",
  "MoveToPregrasp",
  "ApproachObject",
  "GraspObject",
  "ConfirmGrasp",
  "LiftObject",
  "ConfirmHeld",
];
const allSteps = [
  ...beforeCarry,
  "CarryObject",
  "CheckPlacementTarget",
  "OpenGripperAtDestination",
  "ConfirmRelease",
  "ReturnArm",
  "VerifyPlacedObject",
];
const topic = "/mock/manipulation/execution_events";

// Keep protocol fixtures independent of production rendering definitions.
export function cleaningNodes() {
  return [
    ["ObserveBefore", "CleanDesk", "OperationLeaf", "SUCCESS"],
    ["PlanNext", "ActionCheckpoint", "OperationLeaf", "SUCCESS"],
    ["ValidateProposal", "ActionCheckpoint", "CheckLeaf", "SUCCESS"],
    ["ExecuteOne", "ActionCheckpoint", "OperationLeaf", "RUNNING"],
    ["Reobserve", "ActionCheckpoint", "OperationLeaf", "INVALID"],
    ["VerifyCompletion", "ActionCheckpoint", "CheckLeaf", "INVALID"],
    ["ActionCheckpoint", "UntilVerified", "Sequence", "RUNNING"],
    ["UntilVerified", "CleanDesk", "UntilVerified", "RUNNING"],
    ["CleanDesk", "", "Sequence", "RUNNING"],
  ].map(([id, parent, kind, status]) => ({
    id,
    parent,
    kind,
    status,
    visited: status === "RUNNING",
  }));
}

export function run(
  finished = false,
  id = "execution-1",
  received = Date.now() / 1000 - 8,
) {
  return {
    received_at: received,
    partial: false,
    clock_regressed: false,
    record: {
      execution_id: id,
      mission_id: "mission-1",
      task_id: "task-1",
      object_id: 1,
      snapshot_id: "snapshot-1",
      destination_id: "mock_trash_bin",
      selected_arm: "left",
      execution_profile: "mock",
      stage: finished ? "FINALIZING" : "TRANSPORTING",
      substage: finished ? "" : "CarryObject",
      completed_substages: finished ? allSteps : beforeCarry,
      has_result: finished,
      record_state: finished ? "FINISHED" : "ACTIVE",
      status: finished ? "SUCCESS" : "",
      object_state: finished ? "LEFT_GRIPPER" : "HELD",
      placement_state: finished ? "CONFIRMED" : "NOT_CHECKED",
      arm_recovered: finished,
      stop_confirmed: finished,
      failed_stage: "",
      failed_substage: "",
      error_code: "NONE",
      cancel_mode: "",
      accepted_at_ns: 100e9,
      updated_at_ns: finished ? 140e9 : 120e9,
      revision: finished ? 30 : 15,
    },
    intervals: [
      {
        stage: "GRASPING",
        substage: "GraspObject",
        start_ns: 110e9,
        end_ns: 113e9,
        partial: false,
        received_at: received - 10,
      },
      {
        stage: "TRANSPORTING",
        substage: "CarryObject",
        start_ns: 120e9,
        end_ns: finished ? 128e9 : null,
        partial: false,
        received_at: received,
      },
    ],
  };
}
export function rows(
  runs = [run()],
  at = Date.now() / 1000,
  staleRuntime = false,
) {
  return [
    {
      seq: 1,
      kind: "config",
      key: "monitor",
      received_at: at,
      data: { manipulation_events_topic: topic },
    },
    {
      seq: 2,
      kind: "graph",
      key: "ros",
      received_at: at,
      data: { topics: [{ name: topic, publishers: [{}] }] },
    },
    {
      seq: 3,
      kind: "runtime",
      key: "runtime",
      received_at: at - (staleRuntime ? 20 : 0),
      data: {
        state: "WORKING",
        active_request: { mission_id: "mission-1" },
        bt_stage: "ExecuteOne",
        safe_to_drive: staleRuntime,
        drive_block_reason: "물체 HELD: 팔 복귀가 확인되지 않았습니다.",
        manipulation_feedback: {
          execution_id: "execution-1",
          stage: "TRANSPORTING",
          substage: "CarryObject",
        },
      },
    },
    {
      seq: 4,
      kind: "manipulation",
      key: "executions",
      received_at: at,
      data: { executions: runs, evicted: 0 },
    },
  ];
}
export async function transport(page: Page, initial = rows()) {
  const commands: unknown[] = [];
  let send = (_rows: ReturnType<typeof rows>) => {};
  // Replay mounts RecordingControls too; keep these fixtures independent of a
  // local Monitor's live recording settings and the Vite API proxy.
  await page.route("**/api/recording/settings", (route) =>
    route.fulfill({
      json: {
        settings: {
          active: false,
          preset: "custom",
          limit_mib: 100,
          enabled: {},
          hz: {},
          topics: {},
          sources: {},
          source_hz: {},
          source_enabled: {},
        },
        status: {},
        reason: "",
      },
    }),
  );
  await page.route("**/api/recordings", (route) =>
    route.fulfill({ json: { sessions: [], active: "", reason: "" } }),
  );
  await page.routeWebSocket("**/ws", (ws) => {
    let first = true;
    send = (values) => {
      ws.send(
        JSON.stringify({ initial: first, boot_id: "fixture", rows: values }),
      );
      first = false;
    };
    send(initial);
    ws.onMessage((message) => commands.push(JSON.parse(String(message))));
  });
  return {
    commands,
    update: (values: ReturnType<typeof rows>) => send(values),
  };
}
