import { test, expect } from "@playwright/test";

// UI transport fixture only; this does not claim a ROS or robot execution.
const runtime = {
  boot_id: "fixture-runtime",
  state: "ERROR",
  safe_to_drive: false,
  drive_block_reason: "물체를 들고 있어 주행할 수 없습니다.",
  bt_stage: "ExecuteOne",
  active_request: null,
  fsm_states: ["IDLE", "NAVIGATE_TO_TARGET", "WORKING", "CANCELLING", "ERROR"],
  fsm_edges: [
    ["IDLE", "NAVIGATE_TO_TARGET"],
    ["NAVIGATE_TO_TARGET", "WORKING"],
    ["WORKING", "CANCELLING"],
    ["CANCELLING", "ERROR"],
  ],
  bt_nodes: [
    { id: "CleanDesk", parent: "", status: "INVALID" },
    { id: "ExecuteOne", parent: "CleanDesk", status: "INVALID", visited: true },
  ],
  modules: {
    execution: {
      ready: false,
      stopped: true,
      reason: "HELD",
      topics: ["/manipulation/events"],
    },
  },
  execution_profile: { execution: "mock" },
  manipulation: { status: "CANCELED", object_state: "HELD" },
};
const session = {
  id: "fixture-recording",
  start: 900,
  runtime_start: 1000,
  runtime_count: 1,
  end: 1010,
  count: 12,
  bytes: 1024,
  metadata: {},
};

async function setup(page: import("@playwright/test").Page) {
  const commands: unknown[] = [];
  const row = (received_at: number) => ({
    seq: 1,
    kind: "runtime",
    key: "runtime",
    received_at,
    ros_time: null,
    data: runtime,
  });
  await page.route("**/api/recordings", (route) =>
    route.fulfill({ json: { sessions: [session] } }),
  );
  await page.route("**/api/recordings/*/replay?*", (route) =>
    route.fulfill({ json: { rows: [row(1000)], metadata: {} } }),
  );
  await page.routeWebSocket("**/ws", (ws) => {
    ws.send(
      JSON.stringify({
        initial: true,
        boot_id: "fixture",
        sequence: 1,
        rows: [row(Date.now() / 1000)],
      }),
    );
    ws.onMessage((message) => commands.push(JSON.parse(String(message))));
  });
  return commands;
}

test("stale observations never imply drive readiness; replay remains isolated", async ({
  page,
}) => {
  const errors: string[] = [];
  page.on("pageerror", (e) => errors.push(e.message));
  const commands = await setup(page);
  await page.goto("/");
  await expect(page.getByText("주행 차단", { exact: true })).toBeVisible();
  await expect(page.getByText("확인 불가", { exact: true })).toBeVisible({
    timeout: 6000,
  });
  await page.getByRole("button", { name: "기록 & 재생", exact: true }).click();
  await page.getByRole("button", { name: "재생", exact: true }).click();
  await expect(page.locator(".header-status")).toContainText("REPLAY");
  await expect(page.getByText("주행 차단", { exact: true })).toBeVisible();
  await page.getByRole("button", { name: "실시간으로" }).click();
  await expect(page.getByText("확인 불가", { exact: true })).toBeVisible();
  expect(commands).toEqual([]);
  expect(errors).toEqual([]);
});

test("console navigation and diagnostics work at desktop and narrow widths", async ({
  page,
}) => {
  await setup(page);
  await page.goto("/");
  await expect(page.getByRole("region", { name: "노드 상세" })).toHaveCount(0);
  await page
    .locator(".react-flow__node")
    .filter({ hasText: "WORKING" })
    .click();
  await expect(page.getByRole("region", { name: "노드 상세" })).toBeVisible();
  await page.getByRole("button", { name: "상세 닫기" }).click();
  await page.getByRole("tab", { name: "청소 BT", exact: true }).click();
  await expect(
    page.locator(".react-flow__node").filter({ hasText: "ExecuteOne" }),
  ).toBeVisible();
  await page.getByRole("tab", { name: "FSM", exact: true }).click();
  await page.getByRole("button", { name: "사이드바 접기" }).click();
  await page.getByRole("button", { name: "ROS 탐색기", exact: true }).click();
  await page.getByRole("tab", { name: "Topics", exact: true }).focus();
  await page.keyboard.press("ArrowRight");
  await expect(page.getByRole("tab", { name: "연결 그래프" })).toHaveAttribute(
    "aria-selected",
    "true",
  );
  await page.getByRole("button", { name: "자율주행", exact: true }).click();
  await expect(page.getByRole("link", { name: "연결 가이드" })).toHaveAttribute(
    "href",
    /docs.foxglove.dev/,
  );
  await page.getByRole("button", { name: "사이드바 펼치기" }).click();
  await page.getByRole("button", { name: "실행 흐름", exact: true }).click();
  await page.screenshot({
    path: "test-results/console-desktop.png",
    fullPage: true,
  });
  await page.setViewportSize({ width: 390, height: 844 });
  await expect(page.getByRole("region", { name: "실행 그래프" })).toBeVisible();
  expect(
    await page.evaluate(
      () => document.documentElement.scrollWidth <= window.innerWidth,
    ),
  ).toBe(true);
  await expect
    .poll(async () => {
      const flow = page.locator(".flow").first();
      const bounds = await flow.boundingBox();
      const nodes = await flow.locator(".react-flow__node").all();
      for (const node of nodes) {
        const box = await node.boundingBox();
        if (
          !bounds ||
          !box ||
          box.x < bounds.x ||
          box.x + box.width > bounds.x + bounds.width
        )
          return false;
      }
      return true;
    })
    .toBe(true);
  await page.screenshot({
    path: "test-results/console-mobile.png",
    fullPage: true,
  });
});

test("telemetry refresh preserves graph DOM and viewport; state changes preserve geometry", async ({
  page,
}) => {
  let send: (state: string, sequence: number) => void = () => {};
  await page.routeWebSocket("**/ws", (ws) => {
    send = (state, sequence) =>
      ws.send(
        JSON.stringify({
          boot_id: "stable-graph",
          sequence,
          rows: [
            {
              seq: sequence,
              kind: "runtime",
              key: "runtime",
              received_at: Date.now() / 1000,
              data: { ...runtime, state, timestamp: sequence },
            },
          ],
        }),
      );
    send("IDLE", 1);
  });
  await page.goto("/");
  const flow = page.locator(".react-flow");
  await expect(flow.locator(".react-flow__node")).toHaveCount(
    runtime.fsm_states.length,
  );
  // Let the initial auto-fit settle before exercising user-controlled zoom.
  await page.waitForTimeout(250);
  await page.getByRole("button", { name: /zoom in/i }).click();
  const before = await flow
    .locator(".react-flow__viewport")
    .getAttribute("style");
  const geometry = await flow
    .locator(".react-flow__node")
    .evaluateAll((nodes) =>
      nodes.map((n) => ({
        id: n.getAttribute("data-id"),
        width: n.getBoundingClientRect().width,
        height: n.getBoundingClientRect().height,
      })),
    );
  await flow.evaluate((el) => {
    const target = el as HTMLElement & {
      changes: number;
      observer: MutationObserver;
    };
    target.changes = 0;
    target.observer = new MutationObserver((records) => {
      target.changes += records.length;
    });
    target.observer.observe(el, {
      attributes: true,
      childList: true,
      subtree: true,
      characterData: true,
    });
  });
  for (let sequence = 2; sequence <= 9; sequence++) {
    send("IDLE", sequence);
    await page.waitForTimeout(100);
  }
  expect(
    await flow.evaluate(
      (el) => (el as HTMLElement & { changes: number }).changes,
    ),
  ).toBe(0);
  send("WORKING", 10);
  await expect(
    page.locator('.react-flow__node[data-id="WORKING"]'),
  ).toHaveAttribute("aria-label", "WORKING (활성)");
  expect(
    await flow.locator(".react-flow__viewport").getAttribute("style"),
  ).toBe(before);
  expect(
    await flow.locator(".react-flow__node").evaluateAll((nodes) =>
      nodes.map((n) => ({
        id: n.getAttribute("data-id"),
        width: n.getBoundingClientRect().width,
        height: n.getBoundingClientRect().height,
      })),
    ),
  ).toEqual(geometry);
});

test("FSM highlight holds each observed state for 500ms without delaying actual status", async ({
  page,
}) => {
  await page.clock.install();
  let seq = 0;
  let send: (state: string, boot?: string) => void = () => {};
  await page.routeWebSocket("**/ws", (ws) => {
    send = (state, boot = "dwell") =>
      ws.send(
        JSON.stringify({
          boot_id: "transport",
          sequence: ++seq,
          rows: [
            {
              seq,
              kind: "runtime",
              key: "runtime",
              received_at: Date.now() / 1000,
              data: { ...runtime, state, boot_id: boot },
            },
          ],
        }),
      );
    send("IDLE");
  });
  await page.clock.pauseAt(new Date());
  await page.goto("/");
  const active = (state: string) =>
    page.locator(`.runtime-node[data-id="${state}"]`);
  await expect(active("IDLE")).toHaveAttribute("aria-label", "IDLE (활성)");
  send("NAVIGATE_TO_TARGET");
  await expect(page.locator(".fsm-playback-note")).toContainText(
    "NAVIGATE_TO_TARGET",
  );
  send("WORKING");
  await expect(page.locator(".fsm-playback-note")).toContainText("WORKING");
  send("ERROR");
  await expect(page.locator(".execution-summary")).toContainText("ERROR");
  await expect(active("IDLE")).toHaveAttribute("aria-label", "IDLE (활성)");
  await page.clock.runFor(500);
  await expect(active("NAVIGATE_TO_TARGET")).toHaveAttribute(
    "aria-label",
    "NAVIGATE_TO_TARGET (활성)",
  );
  await page.clock.runFor(499);
  await expect(active("NAVIGATE_TO_TARGET")).toHaveAttribute(
    "aria-label",
    "NAVIGATE_TO_TARGET (활성)",
  );
  await page.clock.runFor(1);
  await expect(active("WORKING")).toHaveAttribute(
    "aria-label",
    "WORKING (활성)",
  );
  await page.screenshot({ path: "test-results/fsm-dwell.png", fullPage: true });
  // A new runtime must discard the old presentation queue immediately.
  send("IDLE", "restarted");
  await expect(active("IDLE")).toHaveAttribute("aria-label", "IDLE (활성)");
  await page.clock.runFor(1000);
  await expect(active("IDLE")).toHaveAttribute("aria-label", "IDLE (활성)");
  await expect(page.locator(".fsm-playback-note")).toHaveCount(0);
});

test("ROS graph keeps shared nodes and viewport stable through discovery refresh", async ({
  page,
}) => {
  const topics = [
    {
      name: "/scan",
      publishers: ["/lidar"],
      subscribers: ["/localizer", "/planner"],
    },
    {
      name: "/odom",
      publishers: ["/base"],
      subscribers: ["/localizer", "/planner"],
    },
    { name: "/pose", publishers: ["/localizer"], subscribers: ["/planner"] },
    { name: "/cmd_vel", publishers: ["/planner"], subscribers: ["/base"] },
  ];
  let send: (sequence: number, extra?: boolean) => void = () => {};
  await page.routeWebSocket("**/ws", (ws) => {
    send = (sequence, extra = false) =>
      ws.send(
        JSON.stringify({
          boot_id: "ros-layout",
          sequence,
          rows: [
            {
              seq: sequence,
              kind: "graph",
              key: "ros",
              received_at: Date.now() / 1000,
              data: {
                topics: [...topics]
                  .reverse()
                  .map((t) => ({
                    name: t.name,
                    types: ["std_msgs/msg/String"],
                    state: "receiving",
                    received_hz: sequence,
                    publishers: [...t.publishers]
                      .reverse()
                      .map((node) => ({ node, qos: { depth: sequence } })),
                    subscribers: [
                      ...t.subscribers,
                      ...(extra && t.name === "/pose" ? ["/dashboard"] : []),
                    ]
                      .reverse()
                      .map((node) => ({ node })),
                  })),
              },
            },
          ],
        }),
      );
    send(1);
  });
  await page.goto("/#ros");
  await page.getByRole("tab", { name: "연결 그래프", exact: true }).click();
  const graph = page.locator(".ros-connections");
  await expect(graph.locator(".react-flow__node")).toHaveCount(8);
  // /base publishes odom and consumes cmd_vel, but appears only once.
  await expect(graph.locator('[data-id="node:/base"]')).toHaveCount(1);
  await page.waitForTimeout(250);
  await graph.getByRole("button", { name: /zoom in/i }).click();
  const viewport = await graph
    .locator(".react-flow__viewport")
    .getAttribute("style");
  await graph.evaluate((el) => {
    const target = el as HTMLElement & { changes: number };
    target.changes = 0;
    new MutationObserver((records) => {
      target.changes += records.length;
    }).observe(el, {
      attributes: true,
      childList: true,
      subtree: true,
      characterData: true,
    });
  });
  for (let seq = 2; seq < 8; seq++) {
    send(seq);
    await page.waitForTimeout(100);
  }
  expect(
    await graph.evaluate(
      (el) => (el as HTMLElement & { changes: number }).changes,
    ),
  ).toBe(0);
  expect(
    await graph.locator(".react-flow__viewport").getAttribute("style"),
  ).toBe(viewport);
  await graph.getByRole("button", { name: /fit view/i }).click();
  const boxes = await graph.locator(".react-flow__node").evaluateAll((nodes) =>
    nodes.map((n) => {
      const r = n.getBoundingClientRect();
      return { x: r.x, y: r.y, w: r.width, h: r.height };
    }),
  );
  for (let i = 0; i < boxes.length; i++)
    for (let j = i + 1; j < boxes.length; j++) {
      const a = boxes[i],
        b = boxes[j];
      expect(
        a.x + a.w <= b.x ||
          b.x + b.w <= a.x ||
          a.y + a.h <= b.y ||
          b.y + b.h <= a.y,
      ).toBe(true);
    }
  await page.screenshot({
    path: "test-results/ros-flexible-graph.png",
    fullPage: true,
  });
  send(8, true);
  await expect(graph.locator('[data-id="node:/dashboard"]')).toBeVisible();
});
