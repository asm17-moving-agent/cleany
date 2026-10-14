import { test, expect, type Page } from "@playwright/test";

const beforeCarry = [
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

function run(
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
function rows(runs = [run()], at = Date.now() / 1000, staleRuntime = false) {
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
async function transport(page: Page, initial = rows()) {
  const commands: unknown[] = [];
  let send = (_rows: ReturnType<typeof rows>) => {};
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

test("active manipulation shows reported substages, timing and current drive gates", async ({
  page,
}) => {
  const transportFixture = await transport(page);
  await page.goto("/#manipulation");
  await expect(
    page.getByRole("heading", { name: "매니퓰레이션", exact: true }),
  ).toBeVisible();
  await expect(page.locator(".manip-summary")).toContainText("진행 중");
  await expect(page.locator(".manip-summary")).toContainText("CarryObject");
  await expect(
    page.getByText("11/17개 동작 완료", { exact: true }),
  ).toBeVisible();
  await expect(
    page.getByRole("progressbar", { name: "서버가 확인한 동작 진행률" }),
  ).toHaveAttribute("aria-valuenow", "11");
  await expect(
    page
      .getByRole("list", { name: "전체 조작 단계" })
      .locator('[aria-current="step"]'),
  ).toContainText("수거함 운반");
  await expect(
    page.getByRole("region", { name: "현재 Runtime 주행 판단" }),
  ).toContainText("주행 차단");
  if (process.env.MANIPULATION_VISUAL_DIR) {
    await page.screenshot({
      path: process.env.MANIPULATION_VISUAL_DIR + "/active-light.png",
      fullPage: true,
      animations: "disabled",
    });
    await page.getByRole("button", { name: "다크 모드로 전환" }).click();
    transportFixture.update(rows());
    await page.screenshot({
      path: process.env.MANIPULATION_VISUAL_DIR + "/active-dark.png",
      fullPage: true,
      animations: "disabled",
    });
    await page.getByRole("button", { name: "라이트 모드로 전환" }).click();
    transportFixture.update(rows());
  }
  await page.getByRole("tab", { name: "실행 타임라인", exact: true }).click();
  await expect(page.getByRole("table")).toContainText("GraspObject");
  await expect(page.getByRole("table")).toContainText("3.0초");
  await expect(page.getByRole("table")).toContainText("CarryObject");
  transportFixture.update(rows([run(true)], Date.now() / 1000, true));
  await page.getByRole("tab", { name: "조작 개요", exact: true }).click();
  await expect(page.locator(".manip-summary")).toContainText("완료된 실행");
  await expect(page.locator(".manip-summary")).toContainText("40.0초");
  await expect(
    page.getByRole("progressbar", { name: "서버가 확인한 동작 진행률" }),
  ).toHaveAttribute("aria-valuenow", "17");
  await expect(
    page.getByRole("region", { name: "현재 Runtime 주행 판단" }),
  ).toContainText("미확인");
  await expect(
    page.getByText("조작 기준 주행 조건 충족", { exact: true }),
  ).toHaveCount(0);
  expect(transportFixture.commands).toEqual([]);
});

test("cancellation and incomplete observations survive execution comparison on narrow screens", async ({
  page,
}) => {
  const cancelled = run(true);
  Object.assign(cancelled.record, {
    status: "CANCELED",
    cancel_mode: "CHECKPOINT",
    object_state: "HELD",
    arm_recovered: false,
    error_code: "CANCELED",
    completed_substages: beforeCarry,
  });
  const failed = run(true, "execution-2");
  Object.assign(failed.record, {
    object_id: 2,
    status: "FAILED",
    error_code: "GRASP_FAILED",
    failed_stage: "GRASPING",
    failed_substage: "GraspObject",
    completed_substages: beforeCarry.slice(0, 7),
    object_state: "UNKNOWN",
    placement_state: "NOT_CHECKED",
    arm_recovered: false,
    updated_at_ns: 113e9,
  });
  failed.intervals = failed.intervals.slice(0, 1);
  failed.partial = true;
  await transport(page, rows([cancelled, failed]));
  await page.goto("/#manipulation");
  await page.getByRole("tab", { name: "취소·주행 조건", exact: true }).click();
  await expect(page.getByText("CHECKPOINT", { exact: true })).toBeVisible();
  await expect(page.getByText("HELD", { exact: true })).toBeVisible();
  await page.getByRole("tab", { name: "실행 비교", exact: true }).click();
  await expect(page.getByRole("table")).toContainText("CANCELED");
  await expect(page.getByRole("table")).toContainText("GRASP_FAILED");
  await page
    .getByRole("combobox", { name: "조작 실행 선택" })
    .selectOption("execution-2");
  await expect(
    page.getByText(/일부 구간 미관측: 세부 시간은 불완전합니다/),
  ).toBeVisible();
  await page.getByRole("tab", { name: "단계 진행", exact: true }).click();
  await expect(page.locator(".manip-stages > li.failed")).toContainText(
    "GraspObject",
  );
  await page.getByRole("button", { name: "사이드바 접기" }).click();
  await page.setViewportSize({ width: 620, height: 900 });
  await expect(page.locator(".manip-stages > li.failed")).toBeVisible();
  expect(
    await page.evaluate(
      () => document.documentElement.scrollWidth <= innerWidth,
    ),
  ).toBe(true);
  if (process.env.MANIPULATION_VISUAL_DIR) {
    await page.screenshot({
      path: process.env.MANIPULATION_VISUAL_DIR + "/failed-mobile.png",
      fullPage: true,
      animations: "disabled",
    });
  }
});

test("manipulation replay uses the selected past snapshot and never a live success", async ({
  page,
}) => {
  await transport(page, rows([run(true)]));
  await page.route("**/api/recordings", (route) =>
    route.fulfill({
      json: {
        sessions: [
          {
            id: "session",
            start: 1000,
            runtime_start: 1000,
            end: 1010,
            runtime_count: 1,
            count: 2,
            bytes: 500,
            metadata: {},
          },
        ],
      },
    }),
  );
  await page.route("**/api/recordings/*/replay?*", (route) =>
    route.fulfill({
      json: {
        rows: rows([run(false, "execution-1", 1000)], 1000),
        metadata: {},
      },
    }),
  );
  await page.goto("/#manipulation");
  await expect(page.locator(".manip-summary")).toContainText("완료된 실행");
  await page.getByRole("button", { name: "기록 & 재생", exact: true }).click();
  await page.getByRole("button", { name: "재생", exact: true }).click();
  await page.getByRole("button", { name: "매니퓰레이션", exact: true }).click();
  await expect(page.locator(".manip-summary")).toContainText("기록 시점 진행");
  await expect(page.locator(".manip-summary")).toContainText("CarryObject");
  await expect(page.getByText("최종 결과 대기", { exact: true })).toBeVisible();
  await expect(page.getByText("SUCCESS", { exact: true })).toHaveCount(0);
});
