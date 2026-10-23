import { test, expect } from "@playwright/test";

const keys = [
  "motion",
  "paths",
  "costmaps",
  "bt",
  "safety",
  "logs",
  "resources",
  "lidar",
  "point_cloud",
  "voxel_layer",
  "voxel_world",
  "camera",
];
const heavy = ["lidar", "point_cloud", "voxel_layer", "voxel_world", "camera"];
function initial() {
  return {
    source_hz: { path_topic: 2, local_path_topic: 5, rosout_topic: null },
    rate_defaults_version: 3,
    sources: {
      path_topic: "/plan",
      local_path_topic: "/local_plan",
      velocity_topics: ["/cmd_vel"],
    },
    active: false,
    preset: "standard",
    limit_mib: 100,
    enabled: Object.fromEntries(keys.map((k) => [k, !heavy.includes(k)])),
    hz: Object.fromEntries(
      keys
        .filter((k) => k !== "logs")
        .map((k) => [k, heavy.includes(k) ? 2 : k === "resources" ? 1 : 10]),
    ),
    topics: {
      lidar: "/scan",
      point_cloud: "/camera/head/depth/points",
      voxel_layer: "/local_costmap/voxel_grid",
      voxel_world: "/obstacle_memory/voxels",
      camera: "/camera/head/color/image_raw",
    },
  };
}
test("recording presets, independent toggles, rate, stop and fresh starts", async ({
  page,
}) => {
  let settings = initial();
  const writes: any[] = [];
  const methods: string[] = [];
  let reads = 0;
  await page.route("**/api/recordings", (route) =>
    route.fulfill({ json: { sessions: [], active: "fixture", reason: "" } }),
  );
  await page.route("**/api/navigation/runs", (route) =>
    route.fulfill({ json: { runs: [] } }),
  );
  await page.route("**/api/recording/*", async (route) => {
    if (route.request().method() !== "GET") {
      settings = route.request().postDataJSON();
      writes.push(settings);
      methods.push(
        `${route.request().method()} ${new URL(route.request().url()).pathname}`,
      );
    } else reads++;
    await route.fulfill({
      json: {
        settings,
        rate_maximums: {
          resources: { hz: 1, estimated: false, detail: "호스트 측정 주기" },
          point_cloud: { hz: 15, estimated: false, detail: "Depth 설정" },
        },
        rate_profiles: {
          low: {
            source_hz: {
              path_topic: 0.5,
              local_path_topic: 1,
              rosout_topic: null,
            },
            hz: {
              resources: 0.2,
              lidar: 1,
              point_cloud: 0.5,
              voxel_layer: 0.5,
              voxel_world: 0.2,
              camera: 0.2,
            },
          },
          standard: {
            source_hz: {
              path_topic: 2,
              local_path_topic: 5,
              rosout_topic: null,
            },
            hz: {
              resources: 1,
              lidar: 5,
              point_cloud: 2,
              voxel_layer: 2,
              voxel_world: 1,
              camera: 1,
            },
          },
          precise: {
            source_hz: {
              path_topic: 5,
              local_path_topic: 10,
              rosout_topic: null,
            },
            hz: {
              resources: 1,
              lidar: 5.5,
              point_cloud: 15,
              voxel_layer: 10,
              voxel_world: 1,
              camera: 15,
            },
          },
        },
        source_fields: {
          velocity_topics: {
            group: "motion",
            label: "속도 체인",
            multiple: true,
            default_hz: 10,
            maximum: { hz: 20, estimated: false, detail: "제어 주기" },
          },
          rosout_topic: {
            group: "logs",
            label: "ROS 로그",
            multiple: false,
            default_hz: null,
            maximum: { hz: 100, estimated: true, detail: "로그 burst 추정" },
          },
          path_topic: {
            group: "paths",
            label: "전체 경로",
            multiple: false,
            default_hz: 2,
            maximum: { hz: 5, estimated: true, detail: "재계획 주기 추정" },
          },
          local_path_topic: {
            group: "paths",
            label: "지역 경로",
            multiple: false,
          },
        },
        source_applied: { path_topic: "/plan" },
        observation: {
          resources: {
            summary: "수집 설정 2초 (0.5 Hz)",
            detail: "새 호스트 표본을 저장합니다.",
          },
          logs: {
            summary: "/rosout 수신 이벤트별 기록",
            detail: "고정 Hz로 표본화하지 않습니다.",
          },
        },
        reason: "",
        status: Object.fromEntries(
          keys.map((k) => [
            k,
            {
              state: settings.enabled[k]
                ? settings.active
                  ? "토픽 미수신"
                  : "녹화 정지"
                : "미녹화",
            },
          ]),
        ),
      },
    });
  });
  await page.routeWebSocket("**/ws", (ws) =>
    ws.send(
      JSON.stringify({
        initial: true,
        boot_id: "recording-test",
        sequence: 1,
        rows: [],
      }),
    ),
  );
  await page.goto("/#recordings");
  await expect(
    page.getByText("가공 데이터는 SQLite에", { exact: false }),
  ).toBeVisible();
  await expect(
    page.getByText("카메라, 점군, voxel 원본은 용량이 큽니다.", {
      exact: false,
    }),
  ).toBeVisible();
  await expect(
    page.getByRole("checkbox", { name: "호스트 리소스", exact: true }),
  ).toBeChecked();
  await page
    .getByRole("checkbox", { name: "호스트 리소스", exact: true })
    .uncheck();

  await expect(page.getByLabel("Depth point cloud 원본 토픽")).toHaveCount(0);
  await page
    .getByRole("button", { name: "센서 원본 토픽 펼치기", exact: true })
    .click();
  const rawGroup = page.getByRole("checkbox", {
    name: "센서 원본, rosbag2",
    exact: true,
  });
  await rawGroup.check();
  for (const label of [
    "LiDAR 원본",
    "Depth point cloud 원본",
    "Nav2 voxel layer 원본",
    "Voxel world, 장애물 메모리",
    "카메라 원본",
  ]) {
    await expect(
      page.getByRole("checkbox", { name: label, exact: true }),
    ).toBeChecked();
  }
  await rawGroup.uncheck();
  await expect(
    page.getByRole("checkbox", { name: "Depth point cloud 원본", exact: true }),
  ).not.toBeChecked();
  await page
    .getByRole("checkbox", { name: "Nav2 voxel layer 원본", exact: true })
    .check();
  await page.getByLabel("주기 프로필").selectOption("low");
  await expect(page.getByLabel("Depth point cloud 원본 기록 Hz")).toHaveValue(
    "0.5",
  );
  await expect(
    page.getByRole("checkbox", { name: "호스트 리소스", exact: true }),
  ).not.toBeChecked();
  await page.getByLabel("주기 프로필").selectOption("precise");
  await expect(page.getByLabel("Depth point cloud 원본 기록 Hz")).toHaveValue(
    "15",
  );
  await page.getByLabel("주기 프로필").selectOption("standard");
  await expect(
    page.getByRole("checkbox", { name: "Nav2 voxel layer 원본", exact: true }),
  ).toBeChecked();
  await expect(
    page.getByRole("checkbox", { name: "카메라 원본", exact: true }),
  ).not.toBeChecked();
  await page
    .getByRole("checkbox", { name: "Depth point cloud 원본", exact: true })
    .uncheck();
  expect(
    await rawGroup.evaluate(
      (element: HTMLInputElement) => element.indeterminate,
    ),
  ).toBeTruthy();
  await page.getByLabel("Depth point cloud 원본 기록 Hz").fill("3.5");
  await expect(page.getByText("최대 기록 Hz", { exact: true })).toHaveCount(0);
  await expect(
    page.getByLabel("Depth point cloud 원본 기록 Hz").locator(".."),
  ).toContainText("Hz / (15 Hz)");
  await expect(
    page.getByLabel("호스트 리소스 기록 Hz").locator(".."),
  ).toContainText("Hz / (1 Hz)");
  await page
    .getByRole("button", { name: "위치, 속도, 미션 토픽 펼치기", exact: true })
    .click();
  await expect(
    page.getByLabel("속도 체인 1 기록 Hz", { exact: true }).locator(".."),
  ).toContainText("Hz / (20 Hz)");
  await page.getByLabel("세션 용량 MiB").fill("300");
  await page
    .getByRole("button", { name: "계획, 제어 경로 토픽 펼치기", exact: true })
    .click();
  await expect(
    page.getByLabel("전체 경로 기록 Hz", { exact: true }),
  ).toHaveValue("2");
  await expect(
    page.getByLabel("지역 경로 기록 Hz", { exact: true }),
  ).toHaveValue("5");
  await page
    .getByRole("button", { name: "로그 토픽 펼치기", exact: true })
    .click();
  await expect(
    page.getByLabel("ROS 로그 기록 방식", { exact: true }),
  ).toHaveCount(0);
  await expect(
    page.getByLabel("ROS 로그 기록 Hz", { exact: true }),
  ).toHaveValue("");
  await expect(
    page.getByLabel("ROS 로그 기록 Hz", { exact: true }),
  ).toHaveAttribute("placeholder", "수신마다");
  await expect(
    page.getByLabel("ROS 로그 기록 Hz", { exact: true }).locator(".."),
  ).toContainText("Hz / (≈100 Hz)");
  await page.getByLabel("ROS 로그 기록 Hz", { exact: true }).fill("2");
  await page.getByLabel("ROS 로그 기록 Hz", { exact: true }).fill("");
  await expect(
    page.getByLabel("ROS 로그 기록 Hz", { exact: true }),
  ).toHaveValue("");
  await page.getByLabel("ROS 로그 기록 Hz", { exact: true }).fill("2");
  await page.getByLabel("전체 경로 기본 토픽").fill("/custom/plan");
  await page
    .getByRole("checkbox", { name: "전체 경로 기록", exact: true })
    .uncheck();
  await expect(
    page.getByRole("checkbox", { name: "지역 경로 기록", exact: true }),
  ).toBeChecked();
  expect(
    await page
      .getByRole("checkbox", { name: "계획, 제어 경로", exact: true })
      .evaluate((element: HTMLInputElement) => element.indeterminate),
  ).toBeTruthy();
  await expect(
    page.getByLabel("계획, 제어 경로 기록 Hz", { exact: true }),
  ).toHaveCount(0);
  await page.getByLabel("주기 프로필").selectOption("precise");
  await expect(
    page.getByLabel("전체 경로 기록 Hz", { exact: true }),
  ).toHaveValue("5");
  await expect(
    page.getByLabel("지역 경로 기록 Hz", { exact: true }),
  ).toHaveValue("10");
  await expect(page.getByLabel("전체 경로 기본 토픽")).toHaveValue(
    "/custom/plan",
  );
  await expect(
    page.getByRole("checkbox", { name: "전체 경로 기록", exact: true }),
  ).not.toBeChecked();
  await expect(
    page.getByRole("checkbox", { name: "지역 경로 기록", exact: true }),
  ).toBeChecked();
  await page.getByLabel("주기 프로필").selectOption("standard");
  await page.getByLabel("Depth point cloud 원본 기록 Hz").fill("3.5");
  await page.getByLabel("ROS 로그 기록 Hz", { exact: true }).fill("2");
  await page.getByLabel("전체 경로 기록 Hz", { exact: true }).fill("0.5");
  await page.getByLabel("지역 경로 기록 Hz", { exact: true }).fill("3");
  await page.getByLabel("호스트 리소스 기록 Hz").fill("0.2");
  await page
    .getByRole("checkbox", { name: "호스트 리소스", exact: true })
    .uncheck();
  await expect.poll(() => reads).toBeGreaterThan(1);
  await expect(page.getByLabel("Depth point cloud 원본 기록 Hz")).toHaveValue(
    "3.5",
  );
  await page.getByRole("button", { name: "설정 적용", exact: true }).click();
  await expect(page.getByRole("status")).toContainText(
    "녹화 설정을 적용했습니다.",
  );
  expect(writes[0].sources.path_topic).toBe("/custom/plan");
  expect(writes[0].source_enabled.path_topic).toBe(false);
  expect(writes[0].enabled.paths).toBe(true);
  expect(writes[0].preset).toBe("standard");
  expect(writes[0].source_hz.rosout_topic).toBe(2);
  expect(writes[0].source_hz.path_topic).toBe(0.5);
  expect(writes[0].source_hz.local_path_topic).toBe(3);
  expect(writes[0].hz.resources).toBe(0.2);
  expect(writes[0].enabled.resources).toBe(false);
  expect(writes[0].enabled.point_cloud).toBe(false);
  expect(writes[0].hz.point_cloud).toBe(3.5);
  expect(writes[0].limit_mib).toBe(300);
  await page.getByRole("button", { name: "녹화 시작", exact: true }).click();
  await expect(page.getByLabel("주기 프로필")).toBeDisabled();
  await expect(
    page.getByRole("button", { name: "설정 적용", exact: true }),
  ).toBeDisabled();
  await page.getByRole("button", { name: "녹화 정지", exact: true }).click();
  await expect(
    page.getByRole("button", { name: "녹화 시작", exact: true }),
  ).toBeVisible();
  await page.getByRole("button", { name: "녹화 시작", exact: true }).click();
  await expect(
    page.getByRole("button", { name: "녹화 정지", exact: true }),
  ).toBeVisible();
  await expect.poll(() => writes.length).toBe(4);
  expect(methods).toEqual([
    "PUT /api/recording/settings",
    "POST /api/recording/session",
    "PUT /api/recording/settings",
    "POST /api/recording/session",
  ]);
  expect(writes[2].active).toBe(false);
  expect(writes[3].active).toBe(true);
  await page.mouse.move(0, 0);
  await expect(
    page.getByRole("button", { name: "녹화 정지", exact: true }),
  ).toHaveCSS("background-color", "rgb(220, 38, 38)");
  await expect(
    page.getByRole("button", { name: "새 녹화 시작", exact: true }),
  ).toHaveCount(0);
  await page.screenshot({
    path: "test-results/recording-settings.png",
    fullPage: true,
  });
});

test("recording exports separate processed data and only finalized bags", async ({
  page,
}) => {
  const base = {
    count: 3,
    runtime_count: 0,
    start: 100,
    end: 101,
    bytes: 1024,
    segments: [{ start: 100, end: 101 }],
  };
  await page.route("**/api/recordings", (route) =>
    route.fulfill({
      json: {
        sessions: [
          {
            ...base,
            id: "finished",
            metadata: { bag_state: "finalized" },
            bag_available: true,
            bag_recording: false,
          },
          {
            ...base,
            id: "active",
            metadata: { bag_state: "recording" },
            bag_available: false,
            bag_recording: true,
          },
          {
            ...base,
            id: "incomplete",
            metadata: { bag_state: "incomplete", raw_gap: "camera dropped" },
            bag_available: false,
            bag_recording: false,
          },
        ],
        active: "active",
        reason: "",
      },
    }),
  );
  await page.route("**/api/navigation/runs", (route) =>
    route.fulfill({ json: { runs: [] } }),
  );
  await page.route("**/api/recording/settings", (route) =>
    route.fulfill({
      json: {
        settings: initial(),
        status: {},
        source_fields: {},
        rate_profiles: {},
        reason: "",
      },
    }),
  );
  await page.routeWebSocket("**/ws", (ws) =>
    ws.send(
      JSON.stringify({
        initial: true,
        boot_id: "export-test",
        sequence: 1,
        rows: [],
      }),
    ),
  );
  await page.goto("/#recordings");
  await page
    .locator("summary")
    .filter({ hasText: "기록 관리, 내보내기" })
    .click();
  const finished = page.getByRole("row").filter({ hasText: "finished" });
  await expect(
    finished.getByRole("link", { name: "SQLite 내보내기 ↓" }),
  ).toHaveAttribute("href", "/api/recordings/finished/export");
  await expect(
    finished.getByRole("link", { name: "rosbag2 내보내기 ↓" }),
  ).toHaveAttribute("href", "/api/recordings/finished/bag");
  const active = page.getByRole("row").filter({ hasText: "active" });
  await expect(active).toContainText("녹화 정지 후 내보낼 수 있습니다.");
  await expect(
    active.getByRole("link", { name: "rosbag2 내보내기 ↓" }),
  ).toHaveCount(0);
  const incomplete = page.getByRole("row").filter({ hasText: "incomple" });
  await expect(incomplete).toContainText("rosbag2 미완료");
  await expect(incomplete).toContainText("유실 있음");
  await expect(
    incomplete.getByRole("link", { name: "rosbag2 내보내기 ↓" }),
  ).toHaveCount(0);
});

test("logger disconnect disables recording controls and preserves the draft", async ({
  page,
}) => {
  let offline = false;
  await page.route("**/api/recordings", (route) =>
    route.fulfill({ json: { sessions: [], active: "", reason: "" } }),
  );
  await page.route("**/api/navigation/runs", (route) =>
    route.fulfill({ json: { runs: [] } }),
  );
  await page.route("**/api/recording/settings", (route) =>
    offline
      ? route.fulfill({
          status: 503,
          body: "logger 서비스를 사용할 수 없습니다.",
        })
      : route.fulfill({
          json: {
            settings: initial(),
            status: {},
            rate_profiles: {},
            reason: "",
          },
        }),
  );
  await page.routeWebSocket("**/ws", (socket) =>
    socket.send(
      JSON.stringify({
        initial: true,
        boot_id: "offline-test",
        sequence: 1,
        rows: [],
      }),
    ),
  );
  await page.goto("/#recordings");
  await expect(page.getByLabel("세션 용량 MiB")).toBeEnabled();
  await page.getByLabel("세션 용량 MiB").fill("250");
  offline = true;
  await expect(
    page.getByText("logger 연결 끊김", { exact: true }),
  ).toBeVisible();
  await expect(page.getByLabel("세션 용량 MiB")).toBeDisabled();
  await expect(page.getByLabel("세션 용량 MiB")).toHaveValue("250");
  await expect(
    page.getByRole("button", { name: "녹화 시작", exact: true }),
  ).toBeDisabled();
});
