import { test, expect, type Page } from "@playwright/test";
import { run, rows, transport, cleaningNodes } from "./fixtures/manipulation";
import { type Data } from "../src/lib/monitor";

function cleaningRows(runs = [run()], stale = false) {
  const values = rows(runs, Date.now() / 1000, stale);
  const runtime = values.find((row) => row.kind === "runtime")!.data as Data;
  Object.assign(runtime, {
    boot_id: "cleaning-fixture",
    event_sequence: 0,
    proposal: {
      action: "collect_trash",
      execution_id: "execution-1",
      object_id: "trash-1",
    },
    bt_nodes: cleaningNodes(),
  });
  return values;
}
async function open(page: Page) {
  await page.goto("/");
  await page.getByRole("tab", { name: "BT", exact: true }).click();
  await page.locator('.react-flow__node[data-id="ExecuteOne"]').click();
  const detail = page.getByRole("region", { name: "ExecuteOne 조작 상세" });
  await expect(detail).toBeVisible();
  return detail;
}
async function screenshot(page: Page, name: string) {
  if (process.env.EXECUTE_ONE_VISUAL_DIR)
    await page
      .getByRole("region", { name: "ExecuteOne 조작 상세" })
      .screenshot({
        path: `${process.env.EXECUTE_ONE_VISUAL_DIR}/${name}.png`,
        animations: "disabled",
      });
}

test("ExecuteOne expands a readable Action view while the actual BT stays intact", async ({
  page,
}) => {
  const errors: string[] = [];
  page.on("pageerror", (error) => errors.push(error.message));
  const fixture = await transport(page, cleaningRows());
  const detail = await open(page);
  await expect(detail).toBeFocused();
  await expect(page.locator(".execution-detail")).toHaveCount(0);
  await expect(page.locator(".react-flow__node")).toHaveCount(9);
  await expect(page.locator(".react-flow__edge")).toHaveCount(8);
  await expect(
    page.locator('.react-flow__node[data-id="ExecuteOne"]'),
  ).toHaveAttribute("aria-expanded", "true");
  await expect(
    detail.getByRole("heading", { name: "수거함 이동", exact: true }),
  ).toBeVisible();
  await expect(detail.getByRole("progressbar")).toHaveAttribute(
    "aria-valuenow",
    "11",
  );
  await expect(
    detail.locator('.execute-phases > [aria-current="step"]'),
  ).toHaveAttribute("data-stage", "TRANSPORTING");
  await expect(
    detail.locator('.execute-phases [data-substage="CarryObject"]'),
  ).toContainText("진행 중");
  expect((await detail.boundingBox())!.width).toBeGreaterThan(950);
  await screenshot(page, "active-light");
  await page.getByRole("button", { name: "다크 모드로 전환" }).click();
  fixture.update(cleaningRows());
  await expect(page.locator("html")).toHaveClass(/dark/);
  await screenshot(page, "active-dark");
  const panelPosition = (await detail.boundingBox())!.y;
  const placing = run();
  Object.assign(placing.record, {
    stage: "PLACING",
    substage: "CheckPlacementTarget",
    completed_substages: [...placing.record.completed_substages, "CarryObject"],
  });
  const update = cleaningRows([placing]);
  const rt = update.find((row) => row.kind === "runtime")!.data as Data;
  rt.manipulation_feedback.stage = "PLACING";
  rt.manipulation_feedback.substage = "CheckPlacementTarget";
  fixture.update(update);
  await expect(
    detail.locator('.execute-phases > [aria-current="step"]'),
  ).toHaveAttribute("data-stage", "PLACING");
  expect(
    Math.abs((await detail.boundingBox())!.y - panelPosition),
  ).toBeLessThan(2);
  await detail.getByRole("button", { name: "조작 상세 접기" }).click();
  await expect(detail).toHaveCount(0);
  await expect(
    page.locator('.react-flow__node[data-id="ExecuteOne"]'),
  ).toBeFocused();
  expect(fixture.commands).toEqual([]);
  expect(errors).toEqual([]);
});

test("only the current mission and exact execution are displayed, including goal handover", async ({
  page,
}) => {
  const other = run(true, "other-mission");
  other.record.mission_id = "mission-other";
  const previous = run(true, "previous-execution");
  previous.record.object_id = 2;
  const fixture = await transport(page, cleaningRows([other, previous, run()]));
  const detail = await open(page);
  await expect(
    detail
      .getByRole("combobox", { name: "ExecuteOne 실행 선택" })
      .locator("option"),
  ).toHaveCount(3);
  await detail.getByRole("combobox").selectOption("previous-execution");
  await expect(detail).toContainText("선택한 이전 실행");
  await expect(
    detail.getByRole("heading", { name: "수거 완료", exact: true }),
  ).toBeVisible();
  await expect(
    detail.locator('.execute-phases > [aria-current="step"]'),
  ).toHaveCount(0);
  await detail.getByRole("combobox").selectOption("");
  await expect(
    detail.getByRole("heading", { name: "수거함 이동", exact: true }),
  ).toBeVisible();
  const waiting = cleaningRows([other, previous, run()]);
  const runtime = waiting.find((row) => row.kind === "runtime")!.data as Data;
  runtime.proposal.execution_id = "execution-next";
  fixture.update(waiting);
  await expect(detail).toContainText("연결된 조작 기록을 기다리는 중");
  await expect(detail.getByRole("progressbar")).toHaveCount(0);
  await expect(
    detail.getByRole("heading", { name: "수거 완료", exact: true }),
  ).toHaveCount(0);
  const next = run(false, "execution-next");
  next.record.object_id = 2;
  const received = cleaningRows([other, previous, next]);
  const rt = received.find((row) => row.kind === "runtime")!.data as Data;
  rt.proposal.execution_id = "execution-next";
  rt.manipulation_feedback.execution_id = "execution-next";
  fixture.update(received);
  await expect(detail.locator(".execute-target")).toContainText("물체 2");
  await expect(detail).toContainText("실시간 관측");
  expect(fixture.commands).toEqual([]);
});

test("failed substages remain visible despite BT SUCCESS, and mobile has no overflow", async ({
  page,
}) => {
  const failed = run(true);
  Object.assign(failed.record, {
    status: "FAILED",
    failed_stage: "GRASPING",
    failed_substage: "GraspObject",
    error_code: "GRASP_FAILED",
    completed_substages: ["ValidateGoal"],
    object_state: "UNKNOWN",
  });
  const values = cleaningRows([failed]);
  (values.find((row) => row.kind === "runtime")!.data as Data).bt_nodes.find(
    (node: Data) => node.id === "ExecuteOne",
  ).status = "SUCCESS";
  const fixture = await transport(page, values);
  const detail = await open(page);
  await expect(
    detail.getByRole("heading", { name: "조작 실패", exact: true }),
  ).toBeVisible();
  await expect(
    detail.locator('.execute-phases > [data-stage="GRASPING"]'),
  ).toHaveAttribute("data-state", "failed");
  await expect(detail.locator('[data-substage="GraspObject"]')).toHaveAttribute(
    "data-state",
    "failed",
  );
  await expect(detail).toContainText("GRASP_FAILED");
  await expect(detail.getByRole("progressbar")).toHaveAttribute(
    "aria-valuenow",
    "1",
  );
  await screenshot(page, "failed");
  await page.setViewportSize({ width: 390, height: 844 });
  await expect(
    detail.getByRole("button", { name: "조작 상세 접기" }),
  ).toBeVisible();
  expect(
    await page.evaluate(
      () => document.documentElement.scrollWidth <= window.innerWidth,
    ),
  ).toBe(true);
  const header = (await detail
    .locator(".execute-detail-header")
    .boundingBox())!;
  const actions = (await detail
    .locator(".execute-detail-actions")
    .boundingBox())!;
  const hero = (await detail.locator(".execute-current").boundingBox())!;
  expect(actions.y + actions.height).toBeLessThanOrEqual(
    header.y + header.height,
  );
  expect(hero.y).toBeGreaterThanOrEqual(header.y + header.height);
  await screenshot(page, "failed-mobile");
  expect(fixture.commands).toEqual([]);
});

test("stale and cancelled observations never animate as current execution", async ({
  page,
}) => {
  const fixture = await transport(page, cleaningRows([run()], true));
  const detail = await open(page);
  await expect(detail).toContainText("상태 미확인 · 마지막 관측");
  await expect(
    detail.locator('.execute-phases > [aria-current="step"]'),
  ).toHaveCount(0);
  await expect(detail.locator(".execute-active-dot")).toHaveCount(0);
  const clock = await detail.locator(".execute-metrics").innerText();
  await page.waitForTimeout(700);
  expect(await detail.locator(".execute-metrics").innerText()).toBe(clock);
  const canceled = run(true);
  Object.assign(canceled.record, {
    status: "CANCELED",
    cancel_mode: "CHECKPOINT",
    object_state: "HELD",
    stage: "STOPPING",
    completed_substages: ["ValidateGoal"],
  });
  fixture.update(cleaningRows([canceled]));
  await expect(
    detail.getByRole("heading", { name: "수거 취소", exact: true }),
  ).toBeVisible();
  await expect(detail).toContainText("CHECKPOINT");
  await expect(detail.locator(".execute-target-evidence")).toContainText(
    "보유 중",
  );
  await expect(
    detail.locator('.execute-phases > [aria-current="step"]'),
  ).toHaveCount(0);
  expect(fixture.commands).toEqual([]);
});

test("done and missing records do not borrow another mission's success", async ({
  page,
}) => {
  const other = run(true, "other-mission");
  other.record.mission_id = "mission-other";
  const values = cleaningRows([other]);
  const runtime = values.find((row) => row.kind === "runtime")!.data as Data;
  runtime.proposal = { action: "done", execution_id: "" };
  runtime.manipulation_feedback = {};
  const fixture = await transport(page, values);
  const detail = await open(page);
  await expect(detail).toContainText("이번 주기는 조작 Action 없이 통과합니다");
  await expect(detail).toContainText("이번 주기의 조작 실행 없음");
  await expect(detail.getByRole("progressbar")).toHaveCount(0);
  await expect(detail).not.toContainText("수거 완료");
  expect(fixture.commands).toEqual([]);
});

test("replay detail uses only the selected past execution record", async ({
  page,
}) => {
  const fixture = await transport(page, cleaningRows([run(true)]));
  const past = cleaningRows([run(false, "execution-1", 1000)]);
  for (const row of past) row.received_at = 1000;
  await page.route("**/api/recordings", (route) =>
    route.fulfill({
      json: {
        sessions: [
          {
            id: "execution-replay",
            start: 1000,
            runtime_start: 1000,
            runtime_count: 1,
            end: 1001,
            count: 4,
            bytes: 100,
            metadata: {},
          },
        ],
      },
    }),
  );
  await page.route("**/api/recordings/*/replay?*", (route) =>
    route.fulfill({ json: { rows: past, metadata: {} } }),
  );
  await page.goto("/");
  await page.getByRole("button", { name: "기록", exact: true }).click();
  await page
    .locator("summary")
    .filter({ hasText: "기록 관리, 내보내기" })
    .click();
  await page
    .getByRole("button", { name: "시각으로 이동", exact: true })
    .click();
  await page.getByRole("button", { name: "실행", exact: true }).click();
  await page.getByRole("tab", { name: "BT", exact: true }).click();
  await page
    .getByRole("button", { name: "조작 단계 보기", exact: true })
    .press("Enter");
  const detail = page.getByRole("region", { name: "ExecuteOne 조작 상세" });
  await expect(detail).toContainText("기록 시점 진행");
  await expect(detail.getByRole("progressbar")).toHaveAttribute(
    "aria-valuenow",
    "11",
  );
  await expect(detail).not.toContainText("수거 완료");
  fixture.update(cleaningRows([run(true)]));
  await expect(detail.getByRole("progressbar")).toHaveAttribute(
    "aria-valuenow",
    "11",
  );
  expect(fixture.commands).toEqual([]);
});
