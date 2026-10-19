import { test, expect } from "@playwright/test";
import { type Data, type Row } from "../src/lib/monitor";
import { applyRuntimeEvents } from "../src/lib/runtime-observation";
import { cleaningNodes } from "./fixtures/manipulation";

// Actual Mission cleaning-tree topology, supplied through a UI transport fixture.
// This does not execute ROS actions or certify physical manipulation.
function snapshot(): Row {
  return {
    seq: 100,
    kind: "runtime",
    key: "runtime",
    received_at: Date.now() / 1000,
    ros_time: null,
    data: {
      boot_id: "bt-fixture",
      event_sequence: 50,
      state: "WORKING",
      active_request: { mission_id: "mission-1" },
      bt_stage: "ExecuteOne",
      bt_nodes: cleaningNodes(),
      safe_to_drive: false,
      ready: false,
    },
  };
}

function event(sequence: number, kind: string, data: Data): Row {
  return {
    seq: 100 + sequence,
    kind: "event",
    key: "runtime_events",
    received_at: Date.now() / 1000,
    ros_time: null,
    data: {
      boot_id: "bt-fixture",
      mission_id: "mission-1",
      sequence,
      kind,
      data,
    },
  };
}
const tick = (sequence: number, node_id: string, status: string) =>
  event(sequence, "bt_tick", { node_id, status });
const state = (rows: Row[]) =>
  applyRuntimeEvents(rows).find((row) => row.kind === "runtime")!.data;

test("Runtime sequence handles cross-topic delivery order without reverting a snapshot", () => {
  const base = snapshot();
  const older = tick(49, "ExecuteOne", "SUCCESS");
  const newer = tick(51, "Reobserve", "RUNNING");
  // The older event arrived after the snapshot, the newer one before it.
  newer.seq = 99;
  const data = state([newer, base, older]);
  expect(data.bt_stage).toBe("Reobserve");
  expect(
    data.bt_nodes.find((node: Data) => node.id === "ExecuteOne").status,
  ).toBe("RUNNING");
  expect(base.data.bt_stage).toBe("ExecuteOne");
  expect(
    base.data.bt_nodes.find((node: Data) => node.id === "Reobserve").status,
  ).toBe("INVALID");
});

test("BT visitation belongs to one tick, and composites do not overwrite the leaf stage", () => {
  const data = state([
    snapshot(),
    tick(51, "ExecuteOne", "SUCCESS"),
    tick(52, "Reobserve", "RUNNING"),
    tick(53, "ActionCheckpoint", "RUNNING"),
    tick(54, "UntilVerified", "RUNNING"),
    tick(55, "CleanDesk", "RUNNING"),
    tick(56, "Reobserve", "SUCCESS"),
    tick(57, "VerifyCompletion", "SUCCESS"),
    tick(58, "ActionCheckpoint", "SUCCESS"),
    tick(59, "UntilVerified", "SUCCESS"),
    tick(60, "CleanDesk", "SUCCESS"),
  ]);
  expect(data.bt_stage).toBe("VerifyCompletion");
  expect(
    data.bt_nodes
      .filter((node: Data) => node.visited)
      .map((node: Data) => node.id),
  ).toEqual([
    "Reobserve",
    "VerifyCompletion",
    "ActionCheckpoint",
    "UntilVerified",
    "CleanDesk",
  ]);
});

test("old boot and duplicate events cannot alter current state or refresh safety evidence", () => {
  const base = snapshot();
  const oldBoot = tick(99, "CleanDesk", "SUCCESS");
  oldBoot.data.boot_id = "previous-boot";
  const rows = applyRuntimeEvents([
    base,
    tick(51, "ExecuteOne", "FAILURE"),
    tick(51, "ExecuteOne", "SUCCESS"),
    oldBoot,
  ]);
  const observation = rows.find((row) => row.kind === "runtime")!;
  expect(observation.received_at).toBe(base.received_at);
  expect(observation.data.safe_to_drive).toBe(false);
  expect(observation.data.ready).toBe(false);
  expect(
    observation.data.bt_nodes.find((node: Data) => node.id === "ExecuteOne")
      .status,
  ).toBe("FAILURE");
  expect(
    observation.data.bt_nodes.find((node: Data) => node.id === "CleanDesk")
      .status,
  ).toBe("RUNNING");
});

test("a mission result does not synthesize BT success; a new mission drops the old tree", () => {
  const base = snapshot();
  base.data.manipulation_feedback = { execution_id: "old-execution" };
  const failed = event(51, "mission_result", { outcome: "CANCELLED" });
  expect(
    state([base, failed]).bt_nodes.find((node: Data) => node.id === "CleanDesk")
      .status,
  ).toBe("RUNNING");
  const next = state([
    base,
    failed,
    event(52, "transition", {
      to: "NAVIGATE_TO_TARGET",
      active_request: { mission_id: "mission-2" },
    }),
  ]);
  expect(next.bt_nodes).toEqual([]);
  expect(next.bt_stage).toBe("");
  expect(next.manipulation_feedback).toEqual({});
});

test("the cleaning view shows the final leaf when Runtime exits before its next snapshot", async ({
  page,
}) => {
  const commands: unknown[] = [];
  let finish: () => void = () => {};
  await page.routeWebSocket("**/ws", (ws) => {
    ws.send(
      JSON.stringify({
        initial: true,
        boot_id: "monitor-fixture",
        rows: [snapshot()],
      }),
    );
    finish = () =>
      ws.send(
        JSON.stringify({
          boot_id: "monitor-fixture",
          rows: [
            tick(51, "ExecuteOne", "SUCCESS"),
            tick(52, "Reobserve", "SUCCESS"),
            tick(53, "VerifyCompletion", "SUCCESS"),
            tick(54, "ActionCheckpoint", "SUCCESS"),
            tick(55, "UntilVerified", "SUCCESS"),
            tick(56, "CleanDesk", "SUCCESS"),
            event(57, "transition", {
              to: "POST_MISSION",
              active_request: { mission_id: "mission-1" },
            }),
            event(58, "mission_result", { outcome: "SUCCESS" }),
            event(59, "transition", { to: "IDLE", active_request: null }),
          ],
        }),
      );
    ws.onMessage((message) => commands.push(JSON.parse(String(message))));
  });
  await page.goto("/");
  await page.getByRole("tab", { name: "청소 BT", exact: true }).click();
  await expect(page.locator(".react-flow__node")).toHaveCount(9);
  await expect(page.locator(".react-flow__edge")).toHaveCount(8);
  await expect(page.locator(".bt-node-order")).toHaveCount(7);
  await expect(
    page.locator('.react-flow__node[data-id="ExecuteOne"] .bt-node-order'),
  ).toHaveText("3");
  await expect(
    page.locator('.react-flow__node[data-id="UntilVerified"]'),
  ).toContainText("완료까지 반복");
  await expect(
    page.locator('.react-flow__node[data-id="ActionCheckpoint"]'),
  ).toContainText("순서 실행");
  const order = await page
    .locator(".react-flow__node")
    .evaluateAll((elements) =>
      elements.map((element) => ({
        id: element.getAttribute("data-id"),
        x: element.getBoundingClientRect().x,
      })),
    );
  const leaves = [
    "PlanNext",
    "ValidateProposal",
    "ExecuteOne",
    "Reobserve",
    "VerifyCompletion",
  ];
  expect(
    order
      .filter((node) => leaves.includes(node.id!))
      .sort((a, b) => a.x - b.x)
      .map((node) => node.id),
  ).toEqual(leaves);
  await page.locator('.react-flow__node[data-id="VerifyCompletion"]').click();
  await expect(page.getByRole("region", { name: "노드 상세" })).toContainText(
    "SUCCESS여도 청소 완료가 확정되지 않으면",
  );
  await page.getByRole("button", { name: "상세 닫기" }).click();
  await expect(page.locator(".execution-summary").first()).toContainText(
    "ExecuteOne",
  );
  finish();
  const summary = page.locator(".execution-summary").first();
  await expect(summary).toContainText("IDLE");
  await expect(summary).toContainText("마지막 청소 단계");
  await expect(summary).toContainText("VerifyCompletion");
  await expect(summary).not.toContainText("ExecuteOne");
  await expect(
    page.locator('.react-flow__node[data-id="CleanDesk"]'),
  ).toContainText("SUCCESS");
  await expect(
    page.getByText("마지막 청소 결과", { exact: true }),
  ).toBeVisible();
  await expect(
    page.getByText("마지막 BT 관측 · 상태 미확인", { exact: true }),
  ).toBeVisible({ timeout: 6000 });
  await page.setViewportSize({ width: 390, height: 844 });
  await expect(page.locator(".bt-guide")).toBeVisible();
  expect(
    await page.evaluate(
      () => document.documentElement.scrollWidth <= window.innerWidth,
    ),
  ).toBe(true);
  expect(commands).toEqual([]);
});
