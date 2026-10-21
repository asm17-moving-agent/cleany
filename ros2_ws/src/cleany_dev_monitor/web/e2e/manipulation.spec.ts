import { test, expect } from "@playwright/test";
import { run, rows, transport, beforeCarry } from "./fixtures/manipulation";
import { type Data } from "../src/lib/monitor";

test("pre-Action preparation is visible and does not borrow a previous object execution", async ({
  page,
}) => {
  const now = Date.now() / 1000;
  const previous = run(true, "previous-object", now);
  const values = rows([previous], now).filter((row) => row.kind !== "runtime");
  Object.assign(values.find((row) => row.kind === "config")!.data, {
    manipulation_observation_mode: "server",
  });
  const request = {
    execution_id: "new-object",
    started_at_ns: now * 1e9,
    phase: "RUNTIME_PREPARING",
    state: "ACTIVE",
    owner_alive: true,
    history: [{ phase: "RUNTIME_PREPARING", at_ns: now * 1e9 }],
    message: "Preparing services",
  };
  const requestRow = {
    seq: 5,
    kind: "manipulation_request",
    key: "current",
    received_at: now,
    data: request,
  };
  const fixture = await transport(page, [...values, requestRow] as ReturnType<
    typeof rows
  >);
  await page.goto("/#manipulation");
  const panel = page.getByRole("region", { name: "실행 요청 진행" });
  await expect(panel.locator("p strong")).toHaveText("실행 환경 준비");
  await expect(page.locator(".manip-summary")).toHaveCount(0);
  request.phase = "INSPECTING_SCENE";
  request.history.push(
    { phase: "CHECKING_EXECUTION", at_ns: now * 1e9 + 1 },
    { phase: "INSPECTING_SCENE", at_ns: now * 1e9 + 2 },
  );
  fixture.update([...values, requestRow] as ReturnType<typeof rows>);
  await expect(panel.locator("p strong")).toHaveText("초기 장면 인식");
  await expect(panel.locator('[aria-current="step"]')).toContainText(
    "초기 장면 인식",
  );
  await expect(panel.getByText("보고됨", { exact: true })).toHaveCount(2);
  const current = run(false, "new-object", Date.now() / 1000);
  current.record.execution_profile = "jetson";
  (values.find((row) => row.kind === "manipulation")!.data as Data).executions =
    [current, previous];
  request.phase = "EXECUTING";
  request.history.push({ phase: "EXECUTING", at_ns: Date.now() * 1e6 });
  fixture.update([...values, requestRow] as ReturnType<typeof rows>);
  await expect(page.locator(".manip-summary")).toContainText("진행 중");
  await expect(panel.locator("p strong")).toHaveText("실물 조작");
  request.owner_alive = false;
  fixture.update([...values, requestRow] as ReturnType<typeof rows>);
  await expect(panel.locator("p strong")).toHaveText("요청 진행 확인 불가");
  expect(fixture.commands).toEqual([]);
});

test("Jetson server progress needs fresh evidence without a Mission Runtime", async ({
  page,
}) => {
  const at = Date.now() / 1000;
  const execution = run(false, "physical-execution", at);
  execution.record.execution_profile = "jetson";
  const values = rows([execution], at).filter((row) => row.kind !== "runtime");
  const topic = "/manipulation/execution_events";
  Object.assign(values.find((row) => row.kind === "config")!.data, {
    manipulation_events_topic: topic,
    manipulation_observation_mode: "server",
  });
  (values.find((row) => row.kind === "graph")!.data as Data).topics[0].name =
    topic;
  const fixture = await transport(page, values);
  await page.goto("/#manipulation");
  await expect(page.locator(".manip-summary")).toContainText("진행 중");
  await expect(page.locator(".manip-summary")).toContainText("jetson");
  await expect(
    page.getByRole("region", { name: "현재 Runtime 주행 판단" }),
  ).toContainText("미확인");
  const stale = structuredClone(values);
  const staleRun = (
    stale.find((row) => row.kind === "manipulation")!.data as Data
  ).executions[0];
  staleRun.received_at = at - 30;
  fixture.update(stale);
  await expect(page.locator(".manip-summary")).toContainText(
    "진행 상태 미확인",
  );
  await expect(
    page.locator('.manip-stage-rail [aria-current="step"]'),
  ).toHaveCount(0);
  // A heartbeat refreshes liveness; the original progress timestamp stays frozen.
  staleRun.last_seen_at = Date.now() / 1000;
  fixture.update(stale);
  await expect(page.locator(".manip-summary")).toContainText("진행 중");
  const missing = structuredClone(values);
  (
    missing.find((row) => row.kind === "graph")!.data as Data
  ).topics[0].publishers = [];
  fixture.update(missing);
  await expect(page.locator(".manip-summary")).toContainText(
    "진행 상태 미확인",
  );
  const mock = structuredClone(values);
  (
    mock.find((row) => row.kind === "manipulation")!.data as Data
  ).executions[0].record.execution_profile = "mock";
  fixture.update(mock);
  await expect(page.locator(".manip-summary")).toContainText(
    "진행 상태 미확인",
  );
  expect(fixture.commands).toEqual([]);
});

test("goal handover waits for the exact execution instead of showing old progress", async ({
  page,
}) => {
  const fixture = await transport(page);
  await page.goto("/#manipulation");
  await expect(page.locator(".manip-summary")).toContainText("진행 중");
  const waiting = rows();
  (waiting.find((row) => row.kind === "runtime")!.data as Data).proposal = {
    execution_id: "execution-next",
  };
  fixture.update(waiting);
  await expect(
    page.getByText("조작 실행 기록 대기", { exact: true }),
  ).toBeVisible();
  await expect(page.locator(".manip-summary")).toHaveCount(0);
  await expect(
    page.getByRole("region", { name: "현재 Runtime 주행 판단" }),
  ).toContainText("주행 차단");
  const picker = page.getByRole("combobox", { name: "조작 실행 선택" });
  await picker.selectOption("execution-1");
  await expect(page.locator(".manip-summary")).toContainText(
    "진행 상태 미확인",
  );
  await expect(
    page.locator('.manip-stage-rail [aria-current="step"]'),
  ).toHaveCount(0);
  await picker.selectOption("");
  const next = run(false, "execution-next");
  next.record.object_id = 2;
  const received = rows([run(), next]);
  const runtime = received.find((row) => row.kind === "runtime")!.data as Data;
  runtime.proposal = { execution_id: "execution-next" };
  runtime.manipulation_feedback.execution_id = "execution-next";
  fixture.update(received);
  await expect(page.locator(".manip-summary")).toContainText("물체 2");
  await expect(page.locator(".manip-summary")).toContainText("진행 중");
  const newMission = rows([run(true)]);
  const newRuntime = newMission.find((row) => row.kind === "runtime")!
    .data as Data;
  newRuntime.active_request.mission_id = "mission-next";
  newRuntime.manipulation_feedback = {};
  fixture.update(newMission);
  await expect(
    page.getByText("조작 실행 기록 대기", { exact: true }),
  ).toBeVisible();
  await expect(page.locator(".manip-summary")).toHaveCount(0);
  expect(fixture.commands).toEqual([]);
});

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
  await page.getByRole("button", { name: "기록", exact: true }).click();
  await page
    .locator("summary")
    .filter({ hasText: "기록 관리, 내보내기" })
    .click();
  await page
    .getByRole("button", { name: "시각으로 이동", exact: true })
    .click();
  await page.getByRole("button", { name: "매니퓰레이션", exact: true }).click();
  await expect(page.locator(".manip-summary")).toContainText("기록 시점 진행");
  await expect(page.locator(".manip-summary")).toContainText("CarryObject");
  await expect(page.getByText("최종 결과 대기", { exact: true })).toBeVisible();
  await expect(page.getByText("SUCCESS", { exact: true })).toHaveCount(0);
});
