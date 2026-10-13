import { test, expect, type Page, type WebSocketRoute } from "@playwright/test";
const texture = {
  width: 64,
  height: 96,
  resolution: 0.07,
  slice_pose: [0, 0, 0.0],
  image:
    "data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAEAAAABgCAYAAACtxXToAAABDklEQVR4nO3cQY6CQBQAUTVzFO9/Ig/jfiAaTTtv5FdtIU11JWz6E06niPjFTQt8kM3ezs9uOCjXRxePHGGzt4vx+D8UQAtoCqAFNAXQApoCaAFNAbSApgBaQFMALaApgBbQjA/ws2idvzxGe3ie9yorAtxWSz151lLGvwIF0AKaAmgBTQG0gKYAWkBTAC2gGR9gjz6QmEQBtICmAFpAUwAtoCmAFtA0F1iwRnOBb6YAWkBTAC2gKYAW0BRAC2gKoAU04wPs0VxgEgXQApoCaAFNAbSApgBaQNNcYMEazQW+mQJoAU0BtICmAFpAUwAtoCmAFtCMD7BHc4FJFEALaMYH6Le6Oxw5wpH3FvEOd7wlFZe7C3PmAAAAAElFTkSuQmCC",
  light_image:
    "data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAEAAAABgCAYAAACtxXToAAABJElEQVR4nO3cMWrDQBQA0XXwPXM2nVRuUhhVISx+KH+mMQKzHg/rLfSNHud5rsl8aQHN+ADPy/X3Wuv4ef2PHG/fca211mP6GTB+B4w/AwqgBTQF0AKaAmgBTQG0gKYAWkBTAC2gKYAW0BRAC2jGB7jeEvsrxy/es4utt+t2BPjkPcTtocf/BAqgBTQF0AKaAmgBTQG0gKYAWkBTgA1rfPLPFNs/qx2gBTQF0AKaAmgBTQG0gGZ8gOYCG9ZoLnBnCqAFNAXQApoCaAFNAbSApgBaQFOADWs0F7gzBdACmgJoAU0BtICmAFpA01xgwxrNBe5MAbSApgBaQFMALaApgBbQFEALaAqwYY3mAnemAFpAMz5Aj9W9XI97rO74HTD+DBgf4AX6pB/gxiCGrgAAAABJRU5ErkJggg==",
};
const base = Date.now() / 1000;
const session = "95636583-c97b-4df1-b4c3-64d2d7b51222";
const config = {
  map_frame: "map",
  scan_topic: "/scan",
  slam_submaps_topic: "/submap_list",
  slam_trajectory_topic: "/trajectory_node_list",
  slam_constraints_topic: "/constraint_list",
  slam_matched_points_topic: "/scan_matched_points2",
};
function rows(
  at: number,
  past = false,
  stale = false,
  mismatch = false,
  lagging = false,
) {
  let seq = 0;
  const row = (kind: string, key: string, data: any) => ({
    seq: ++seq,
    kind,
    key,
    data,
    received_at: stale ? at - 10 : at,
    ros_time: null,
  });
  const submaps = [
    { id: "0/0", version: past ? 1 : 2, pose: [past ? 0 : 0.35, 3, 0] },
    { id: "0/1", version: 1, pose: [0.1, -0.5, 0.08] },
    { id: "0/2", version: 1, pose: [4.4, -1, Math.PI / 2] },
  ];
  return [
    row("config", "monitor", config),
    row("graph", "ros", {
      topics: Object.entries(config)
        .filter(([k]) => k.endsWith("_topic"))
        .map(([, name]) => ({
          name,
          state: "receiving",
          publishers: [],
          subscribers: [],
          types: [],
        })),
    }),
    row("slam", "submaps", { valid: true, epoch: "fixture", submaps }),
    row("slam", "status", {
      epoch: "fixture",
      state: "ready",
      reason: "브라우저 검증용 모의 SLAM 데이터",
    }),
    ...submaps.map((s) =>
      row("slam_texture", s.id, {
        ...texture,
        id: s.id,
        epoch: "fixture",
        version: mismatch ? 99 : lagging && s.id === "0/0" ? 1 : s.version,
        valid: true,
      }),
    ),
    row("slam", "trajectory", {
      valid: true,
      epoch: "fixture",
      markers: [
        {
          type: 4,
          color: [0.4, 0.7, 1, 1],
          points: [
            [-3, 1],
            [-3, 0],
            [-2, -2],
            [-1, -3],
            [1, -3],
            [3, -2],
          ],
          id: "trajectory:0",
        },
      ],
    }),
    row("slam", "constraints", {
      valid: true,
      epoch: "fixture",
      markers: [
        {
          type: 5,
          color: [0.6, 0.4, 0.9, 0.7],
          points: [
            [-3, 0],
            [-1, -2],
            [-2, -2],
            [0, -1],
            [1, -3],
            [2, 0],
          ],
          id: "constraints:0",
        },
      ],
    }),
    row("slam", "matched_points", {
      valid: true,
      epoch: "fixture",
      points: [
        [-1, -1],
        [-1.1, -1.1],
        [1, 0],
        [1.1, 0],
        [1.2, 0],
      ],
      sampled: true,
    }),
    row("navigation", "scan", {
      valid: true,
      points: [
        [-1, -1],
        [1, 0],
        [1.2, 0],
      ],
    }),
    row("pose", "robot", { valid: true, pose: [-1, -2, 0.2] }),
    ...(!past
      ? [
          {
            ...row("event", "/submap_list", {
              kind: "slam_pose_adjustment",
              id: "0/0",
              epoch: "fixture",
              distance_m: 0.35,
              before: [0, 3, 0],
              after: [0.35, 3, 0],
            }),
            received_at: base - 8,
          },
        ]
      : []),
  ];
}
async function fixture(
  page: Page,
  stale = false,
  mismatch = false,
  lagging = false,
) {
  let socket: WebSocketRoute | undefined;
  let connections = 0;
  await page.route("**/api/recordings", (r) =>
    r.fulfill({
      json: {
        active: session,
        sessions: [
          {
            id: session,
            start: base - 60,
            end: base,
            count: 100,
            bytes: 1000,
            metadata: {},
          },
        ],
      },
    }),
  );
  await page.route("**/api/navigation/runs", (r) =>
    r.fulfill({ json: { runs: [] } }),
  );
  await page.route("**/api/recordings/*/replay?*", (r) => {
    const at = Number(new URL(r.request().url()).searchParams.get("at"));
    return r.fulfill({ json: { rows: rows(at, at < base - 8), metadata: {} } });
  });
  await page.routeWebSocket("**/ws", (ws) => {
    socket = ws;
    connections++;
    ws.send(
      JSON.stringify({
        initial: true,
        boot_id: "slam-fixture",
        sequence: 100,
        rows: rows(Date.now() / 1000, false, stale, mismatch, lagging),
        recording: { id: session },
      }),
    );
    ws.onMessage((m) => {
      const data = JSON.parse(String(m));
      if (data.op === "select")
        ws.send(JSON.stringify({ selection: data.topics }));
    });
  });
  return { disconnect: () => socket?.close(), connections: () => connections };
}

test("integrated SLAM layers, selection, pan, export, refresh and reconnect", async ({
  page,
}) => {
  const errors: string[] = [];
  page.on("pageerror", (e) => errors.push(e.message));
  const transport = await fixture(page);
  await page.goto("/#navigation");
  await page.getByRole("button", { name: "SLAM 분석", exact: true }).click();
  const canvas = page.locator("canvas.slam-map");
  await expect(canvas).toHaveAttribute("data-submaps", "3");
  await page.getByLabel("서브맵 선택").selectOption("0/0");
  await expect(canvas).toHaveAttribute("data-selected", "0/0");
  await expect(page.locator(".slam-facts")).toContainText("0.35, 3.00 m");
  const before = await canvas.screenshot();
  await page.getByLabel("서브맵", { exact: true }).uncheck();
  const after = await canvas.screenshot();
  expect(Buffer.compare(before, after)).not.toBe(0);
  await page.getByLabel("서브맵", { exact: true }).check();
  for (const label of ["현재 스캔", "매칭 점군", "궤적", "제약선"]) {
    await page.getByLabel(label, { exact: true }).uncheck();
    await page.getByLabel(label, { exact: true }).check();
  }
  const original = await page.locator(".slam-map-caption").innerText();
  await page.getByRole("button", { name: "SLAM 확대", exact: true }).click();
  expect(await page.locator(".slam-map-caption").innerText()).not.toBe(
    original,
  );
  const box = await canvas.boundingBox();
  if (!box) throw Error("canvas absent");
  await page.mouse.move(box.x + box.width / 2, box.y + box.height / 2);
  await page.mouse.down();
  await page.mouse.move(
    box.x + box.width / 2 + 70,
    box.y + box.height / 2 + 30,
  );
  await page.mouse.up();
  await page.getByRole("button", { name: "전체 보기", exact: true }).click();
  const download = page.waitForEvent("download");
  await page.getByRole("button", { name: "이미지 저장", exact: true }).click();
  expect((await download).suggestedFilename()).toMatch(/slam-.*\.png/);
  await page.reload();
  await expect(
    page.getByRole("tab", { name: "SLAM 분석", exact: true }),
  ).toHaveAttribute("aria-selected", "true");
  await expect(canvas).toHaveAttribute("data-submaps", "3");
  transport.disconnect();
  await expect.poll(transport.connections).toBeGreaterThan(1);
  await expect(canvas).toHaveAttribute("data-submaps", "3");
  await expect(page.locator(".connection-banner")).toHaveCount(0);
  await page.getByLabel("서브맵 선택").selectOption("0/0");
  await page.screenshot({
    path: "test-results/slam-integrated-desktop.png",
    fullPage: true,
  });
  await page.setViewportSize({ width: 390, height: 844 });
  expect(
    await page.evaluate(
      () => document.documentElement.scrollWidth <= innerWidth,
    ),
  ).toBe(true);
  await page.screenshot({
    path: "test-results/slam-integrated-mobile.png",
    fullPage: true,
  });
  expect(errors).toEqual([]);
});

test("timeline uses historical textures and poses and excludes future adjustments", async ({
  page,
}) => {
  await fixture(page);
  await page.goto("/#navigation");
  await page.getByRole("button", { name: "SLAM 분석", exact: true }).click();
  await page.locator(".slam-events button").click();
  await expect(page.locator(".slam-state")).toContainText("기록 재생");
  const pointer = page.getByRole("slider", { name: "재생 포인터" });
  await pointer.press("Shift+ArrowLeft");
  await expect(pointer).toHaveAttribute("aria-valuenow", String(base - 18));
  await page.getByLabel("서브맵 선택").selectOption("0/0");
  await expect(page.locator(".slam-facts")).toContainText("0.00, 3.00 m");
  await expect(page.locator(".slam-events button")).toHaveCount(0);
  await expect(page.locator(".slam-selection")).toContainText("0/0 · v1");
  await pointer.press("Shift+ArrowRight");
  await expect(page.locator(".slam-facts")).toContainText("0.35, 3.00 m");
  await expect(page.locator(".slam-events button")).toHaveCount(1);
  await expect(page.locator("canvas.slam-map")).toHaveAttribute(
    "data-submaps",
    "3",
  );
});

test("stale maps remain labelled and mismatched texture versions are never drawn", async ({
  page,
}) => {
  await fixture(page, true, true);
  await page.goto("/#navigation");
  await page.getByRole("button", { name: "SLAM 분석", exact: true }).click();
  await expect(page.locator(".slam-state")).toContainText("수신 지연 / 대기");
  await expect(page.locator("canvas.slam-map")).toHaveAttribute(
    "data-submaps",
    "0",
  );
  await page.getByLabel("서브맵 선택").selectOption("0/0");
  await expect(page.locator(".slam-facts")).toContainText("현재 버전 미수신");
});

test("active submaps retain earlier verified images with explicit version lag", async ({
  page,
}) => {
  await fixture(page, false, false, true);
  await page.goto("/#navigation");
  await page.getByRole("button", { name: "SLAM 분석", exact: true }).click();
  await expect(page.locator("canvas.slam-map")).toHaveAttribute(
    "data-submaps",
    "3",
  );
  await expect(page.locator(".slam-details")).toContainText(
    "1개 이미지 갱신 중",
  );
  await page.getByLabel("서브맵 선택").selectOption("0/0");
  await expect(page.locator(".slam-facts")).toContainText("표시 v1 · 최신 v2");
});
