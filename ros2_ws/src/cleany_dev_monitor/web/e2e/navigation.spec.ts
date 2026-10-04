import { test, expect, type Page } from "@playwright/test";

// Browser transport fixtures exercise presentation/replay, not navigation success.
const config = {
  map_frame: "map",
  velocity_topics: ["/nav2/cmd_vel", "/safety_2d/cmd_vel", "/cmd_vel"],
  scan_topic: "/scan",
  depth_points_topic: "/camera/head/depth/obstacle_points",
  risk_points_topic: "/body_guard_3d/risk_voxels",
  footprint_topic: "/local_costmap/published_footprint",
  stop_polygon_topic: "/safety/stop_zone",
  slow_polygon_topic: "/safety/slow_zone",
  local_path_topic: "/local_plan",
  obstacle_memory_topic: "/obstacle_memory/grid",
  lidar_memory_topic: "/obstacle_memory/lidar",
  depth_memory_topic: "/obstacle_memory/depth",
  odometry_topic: "/wheel/odom",
  ground_truth_topic: "/ground_truth/odom",
  amcl_pose_topic: "/amcl_pose",
  pan_state_topic: "/head_pan/state",
  reactive_guard_topic: "/reactive_guard/state",
  voxel_guard_topic: "/body_guard_3d/state",
  coverage_topic: "/head_pan/coverage",
  nav2_bt_topic: "/behavior_tree_log",
  controller_evaluation_topic: "/evaluation",
  navigation_freshness_s: 0.75,
  stationary_speed_mps: 0.005,
  creep_speed_mps: 0.03,
  requested_rtf: 2,
  source_revision: "fixture-only",
};
const sessionId = "54dafce5-545b-4ab0-95db-a38003b98a08";
const base = Date.now() / 1000;
const run = (operation_id: string, wall_s: number) => ({
  operation_id,
  target: "13",
  phase: "NAVIGATE_TO_TARGET",
  start: base - 30,
  end: base - 10,
  wall_s,
  sim_s: wall_s * 1.4,
  rtf: 1.4,
  distance_m: 2.5,
  stationary_s: operation_id === "before" ? 4 : 1,
  creep_s: 2,
  unobserved_s: 0.2,
  stationary_episodes: 2,
  replan_starts: 1,
  goal_error_m: 0.04,
  result: "OK",
  measured_source: "odometry",
  session_id: sessionId,
  configuration: config,
});
function rows(now: number, replay = false) {
  let seq = 0;
  const row = (kind: string, key: string, data: any) => ({
    seq: ++seq,
    kind,
    key,
    data,
    received_at: now,
    ros_time: null,
  });
  const cells = Array.from({ length: 80 * 60 }, (_, i) => {
    const x = i % 80,
      y = Math.floor(i / 80);
    return x < 2 ||
      y < 2 ||
      x > 77 ||
      y > 57 ||
      (x > 24 && x < 35 && y > 18 && y < 38)
      ? 100
      : 0;
  });
  const grid = {
    valid: true,
    width: 80,
    height: 60,
    resolution: 0.1,
    origin: [-3, -2, 0],
    cells,
  };
  const commands = config.velocity_topics.map((topic, i) => ({
    topic,
    vector: [i === 0 ? 0.029 : 0.01, 0, 0],
    fresh: true,
    received_at: now,
    age_s: 0.01,
  }));
  const history = Array.from({ length: 80 }, (_, i) => [
    now - 16 + i * 0.2,
    0.029,
    0.01,
    0.01,
    0.011,
    1.4,
  ]);
  history[0][4] = 380; // A startup outlier must stay visible without flattening normal motion.
  const geometry = (topic: string, points: number[][]) => ({
    topic,
    valid: true,
    frame: "map",
    points,
  });
  return [
    row("config", "monitor", config),
    row("runtime", "runtime", {
      state: "NAVIGATE_TO_TARGET",
      boot_id: "nav-fixture",
      safe_to_drive: true,
      execution_profile: { navigation: "sim" },
      navigation: {
        operation_id: "current",
        target: "13",
        frame: "map",
        goal_pose: { x: 3, y: 2, yaw: 0 },
        lifecycle: { planner_server: "active", controller_server: "active" },
      },
    }),
    row("graph", "ros", {
      topics: Object.entries(config)
        .filter(([k, v]) => k.endsWith("_topic") && typeof v === "string")
        .map(([, v]) => ({
          name: v,
          state: "receiving",
          received_hz: 10,
          publishers: [],
          subscribers: [],
          types: [],
        })),
    }),
    row("layer", "map", grid),
    row("layer", "local_costmap", {
      ...grid,
      cells: cells.map((v) => (v ? 70 : 0)),
    }),
    row("layer", "global_costmap", {
      ...grid,
      cells: cells.map((v) => (v ? 90 : 0)),
    }),
    row("layer", "path", {
      valid: true,
      points: [
        [-2, -1],
        [-1, 0],
        [1, 1],
        [3, 2],
      ],
    }),
    row("pose", "robot", { valid: true, pose: [1, 1, 0.1], age: 0.01 }),
    row("navigation", "summary", {
      motion: replay ? "STATIONARY" : "CREEPING",
      reason: replay ? "STOP_OBSTACLE" : "SlowZone",
      reason_source: replay ? "reactive_guard" : "collision",
      speed_mps: replay ? 0 : 0.011,
      measured: [replay ? 0 : 0.011, 0, 0],
      measured_source: "odometry",
      rtf: 1.4,
      commands,
      history,
      trail: Array.from({ length: 40 }, (_, i) => [
        now - 8 + i * 0.2,
        -2 + i * 0.075,
        -1 + i * 0.05,
        0,
      ]),
      runs: [run("before", 30), run("after", 23)],
      bt_status: {
        ComputePathToPose: "SUCCESS",
        IsPathValid: "SUCCESS",
        FollowPath: "RUNNING",
        YieldWait: "IDLE",
      },
    }),
    row("navigation", "pan", {
      valid: true,
      pan: 0.52,
      target: 0.43,
      reason: "TRACKING",
      motion_authority: false,
    }),
    row("navigation", "reactive_guard", {
      valid: true,
      decision: "PASS",
      processing_s: 0.002,
      source_age_s: 0.05,
      observation_scope: "observed_surfaces_only",
    }),
    row(
      "navigation",
      "scan",
      geometry(
        config.scan_topic,
        Array.from({ length: 50 }, (_, i) => [
          2 + Math.cos(i * 0.13),
          1 + Math.sin(i * 0.13),
          0,
        ]),
      ),
    ),
    row(
      "navigation",
      "footprint",
      geometry(config.footprint_topic, [
        [0.5, 0.6],
        [0.5, 1.4],
        [1.5, 1.4],
        [1.5, 0.6],
      ]),
    ),
    row(
      "navigation",
      "stop_polygon",
      geometry(config.stop_polygon_topic, [
        [0.4, 0.5],
        [0.4, 1.5],
        [1.6, 1.5],
        [1.6, 0.5],
      ]),
    ),
    row(
      "navigation",
      "slow_polygon",
      geometry(config.slow_polygon_topic, [
        [0.2, 0.3],
        [0.2, 1.7],
        [1.8, 1.7],
        [1.8, 0.3],
      ]),
    ),
    row("navigation", "odometry", {
      valid: true,
      pose: [1, 1, 0.1],
      velocity: [0.011, 0, 0],
      source_frame: "odom",
    }),
    row("navigation", "amcl_pose", {
      valid: true,
      map_pose: [1, 1, 0.1],
      covariance: Array(36).fill(0.001),
    }),
    row("navigation", "controller_evaluation", {
      valid: true,
      count: 3,
      best_index: 1,
      transform: [0, 0, 0],
      trajectories: [
        {
          index: 0,
          selected: false,
          total: -1,
          velocity: [0.1, 0, 0],
          points: [
            [1, 1, 0],
            [1.3, 1.2, 0],
          ],
          scores: [{ name: "ObstacleFootprint", raw: -1, scale: 1 }],
        },
        {
          index: 1,
          selected: true,
          total: 4,
          velocity: [0.03, 0, 0],
          points: [
            [1, 1, 0],
            [1.4, 1.1, 0],
          ],
          scores: [{ name: "PathDist", raw: 2, scale: 2 }],
        },
      ],
    }),
    {
      ...row("event", "navigation", {
        kind: "navigation_motion",
        data: { motion: "STATIONARY", reason: "STOP_OBSTACLE" },
      }),
      received_at: base - 0.5,
    },
    row("event", "guard-event", {
      kind: "navigation_decision",
      data: { source: "reactive_guard", decision: "STOP_OBSTACLE" },
    }),
    row("event", "bt-event", {
      kind: "nav2_bt",
      data: { node: "FollowPath", previous: "IDLE", status: "RUNNING" },
    }),
  ];
}
async function fixture(page: Page, stale = false) {
  const messages: any[] = [];
  await page.route("**/api/recordings", (r) =>
    r.fulfill({
      json: {
        active: sessionId,
        sessions: [
          {
            id: sessionId,
            start: base - 100,
            end: base + 100,
            count: 200,
            runtime_count: 1,
            bytes: 2048,
            metadata: {},
          },
        ],
      },
    }),
  );
  await page.route("**/api/navigation/runs", (r) =>
    r.fulfill({ json: { runs: [run("before", 30), run("after", 23)] } }),
  );
  await page.route("**/api/recordings/*/replay?*", (r) => {
    const at = Number(new URL(r.request().url()).searchParams.get("at"));
    return r.fulfill({ json: { rows: rows(at, true), metadata: {} } });
  });
  await page.routeWebSocket("**/ws", (ws) => {
    const send = () =>
      ws.send(
        JSON.stringify({
          initial: true,
          boot_id: "nav-fixture",
          sequence: 100,
          rows: rows(stale ? base - 10 : Date.now() / 1000),
        }),
      );
    send();
    const timer = setInterval(send, 200);
    ws.onClose(() => clearInterval(timer));
    ws.onMessage((m) => {
      const data = JSON.parse(String(m));
      messages.push(data);
      if (data.op === "select")
        ws.send(JSON.stringify({ selection: data.topics }));
    });
  });
  return messages;
}

test("five navigation tabs, bounded topic leases, cause details and synchronized replay", async ({
  page,
}) => {
  const errors: string[] = [];
  page.on("pageerror", (e) => errors.push(e.message));
  const messages = await fixture(page);
  await page.goto("/#navigation");
  for (const name of [
    "주행 개요",
    "장애물·안전",
    "계획·제어",
    "센서·위치",
    "주행 비교",
  ])
    await expect(page.getByRole("tab", { name, exact: true })).toBeVisible();
  await expect(page.locator(".nav-summary-strip")).toContainText("저속 이동");
  await expect(
    page.getByRole("heading", { name: "현재 주행 판단" }),
  ).toBeVisible();
  await expect(
    page.getByRole("heading", { name: "전체 경로와 주행 진행" }),
  ).toBeVisible();
  await expect(
    page.getByRole("checkbox", { name: "계획 경로", exact: true }),
  ).toBeChecked();
  await expect(
    page.getByRole("checkbox", { name: "local costmap", exact: true }),
  ).toHaveCount(0);
  await expect
    .poll(() => messages.at(-1)?.topics)
    .toEqual([config.footprint_topic]);
  await expect(page.locator(".nav-event")).toHaveCount(1);
  await expect(
    page.getByRole("img", { name: "단계별 속도와 실측 속도 그래프" }),
  ).toHaveCount(0);
  await page
    .getByLabel("ROS 2D navigation map")
    .click({ position: { x: 100, y: 100 } });
  await expect(page.getByLabel("선택한 지도 셀")).toContainText(
    "OccupancyGrid",
  );
  await page.screenshot({
    path: "test-results/navigation-overview.png",
    fullPage: true,
  });
  await page
    .getByRole("button", { name: "정지·감속 원인 보기", exact: true })
    .click();
  await expect(
    page.getByRole("tab", { name: "장애물·안전", exact: true }),
  ).toHaveAttribute("aria-selected", "true");
  await expect(
    page.getByRole("heading", { name: "주변 장애물과 안전 영역" }),
  ).toBeVisible();
  await expect(
    page.getByRole("checkbox", { name: "정지 영역", exact: true }),
  ).toBeChecked();
  await expect(
    page.getByRole("checkbox", { name: "계획 경로", exact: true }),
  ).toHaveCount(0);
  await expect(page.locator(".nav-summary-strip")).toHaveCount(0);
  await expect(page.locator(".nav-event")).toHaveCount(2);
  await expect(
    page.locator(".nav-event").filter({ hasText: "nav2_bt" }),
  ).toHaveCount(0);
  await expect(
    page.getByRole("heading", { name: "Voxel guard · 별도 속도 감시" }),
  ).toBeVisible();
  await expect
    .poll(() => messages.at(-1)?.topics)
    .toContain(config.depth_points_topic);
  await page.screenshot({
    path: "test-results/navigation-safety.png",
    fullPage: true,
  });
  await page.getByRole("tab", { name: "계획·제어", exact: true }).click();
  await expect(
    page.getByRole("heading", { name: "계획 경로와 후보 궤적" }),
  ).toBeVisible();
  await expect(
    page.getByRole("checkbox", { name: "제어 경로", exact: true }),
  ).toBeChecked();
  await expect(
    page.getByRole("checkbox", { name: "LiDAR", exact: true }),
  ).toHaveCount(0);
  await expect(
    page.getByRole("img", { name: "단계별 속도와 실측 속도 그래프" }),
  ).toBeVisible();
  await expect(page.locator(".nav-event")).toHaveCount(1);
  await expect(page.locator(".nav-event").first()).toContainText("FollowPath");
  await expect(page.getByText("FollowPath", { exact: true })).toBeVisible();
  await expect(page.getByText("선택 #1", { exact: true })).toBeVisible();
  await expect.poll(() => messages.at(-1)?.topics).toContain("/evaluation");
  await expect
    .poll(() => messages.at(-1)?.topics)
    .not.toContain(config.depth_points_topic);
  await page.screenshot({
    path: "test-results/navigation-control.png",
    fullPage: true,
  });
  await page.getByRole("tab", { name: "센서·위치", exact: true }).click();
  await expect(
    page.getByRole("heading", { name: "센서·TF 수신 상태" }),
  ).toBeVisible();
  await page.getByRole("tab", { name: "주행 비교", exact: true }).click();
  await expect(page.getByText("-7.00 s", { exact: true })).toBeVisible();
  await page.screenshot({
    path: "test-results/navigation-comparison.png",
    fullPage: true,
  });
  await page.getByRole("tab", { name: "주행 개요", exact: true }).click();
  await page.locator(".nav-event").first().click();
  await expect(page.locator(".header-status")).toContainText("REPLAY");
  await expect(page.locator(".nav-summary-strip")).toContainText("정지");
  await expect.poll(() => messages.at(-1)?.topics).toEqual([]);
  await page.getByRole("button", { name: "실시간으로", exact: true }).click();
  await expect(page.locator(".nav-summary-strip")).toContainText("저속 이동");
  await page.getByRole("button", { name: "실행 흐름", exact: true }).click();
  await expect.poll(() => messages.at(-1)?.topics).toEqual([]);
  expect(messages.every((m) => m.op === "select")).toBe(true);
  expect(errors).toEqual([]);
});

test("stale navigation never shows fresh motion; narrow layout stays within viewport", async ({
  page,
}) => {
  await fixture(page, true);
  await page.goto("/#navigation");
  await expect(page.locator(".nav-summary-strip")).toContainText(
    "관측 지연 / 미수신",
  );
  await expect(page.getByText("관측 전용", { exact: true })).toHaveCount(0);
  await page.getByRole("tab", { name: "장애물·안전", exact: true }).click();
  await expect(page.locator(".nav-safety-decisions")).toContainText(
    "미수신 / 지연",
  );
  await expect(page.locator(".nav-safety-decisions")).not.toContainText("PASS");
  await page.setViewportSize({ width: 390, height: 844 });
  for (const name of [
    "주행 개요",
    "장애물·안전",
    "계획·제어",
    "센서·위치",
    "주행 비교",
  ]) {
    await page.getByRole("tab", { name, exact: true }).click();
    expect(
      await page.evaluate(
        () => document.documentElement.scrollWidth <= innerWidth,
      ),
    ).toBe(true);
  }
  await page.getByRole("tab", { name: "주행 개요", exact: true }).click();
  await page.screenshot({
    path: "test-results/navigation-mobile.png",
    fullPage: true,
  });
});
