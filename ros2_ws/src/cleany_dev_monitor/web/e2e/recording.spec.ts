import { test, expect } from "@playwright/test";

const keys = [
  "motion",
  "paths",
  "costmaps",
  "bt",
  "safety",
  "logs",
  "lidar",
  "point_cloud",
  "voxel_layer",
  "voxel_world",
  "camera",
];
const heavy = keys.slice(6);
function initial() {
  return {
    active: true,
    preset: "basic",
    limit_mib: 100,
    enabled: Object.fromEntries(keys.map((k) => [k, !heavy.includes(k)])),
    hz: Object.fromEntries(heavy.map((k) => [k, 2])),
    topics: {
      lidar: "/scan",
      point_cloud: "/camera/head/depth/points",
      voxel_layer: "/local_costmap/voxel_grid",
      voxel_world: "/obstacle_memory/voxels",
      camera: "/camera/head/color/image_raw",
    },
  };
}
test("recording presets, independent toggles, rate, pause and new session", async ({
  page,
}) => {
  let settings = initial();
  const writes: any[] = [];
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
    } else reads++;
    await route.fulfill({
      json: {
        settings,
        reason: "",
        status: Object.fromEntries(
          keys.map((k) => [
            k,
            {
              state: settings.enabled[k]
                ? settings.active
                  ? "토픽 미수신"
                  : "일시정지"
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
    page.getByRole("checkbox", { name: "Depth point cloud 원본", exact: true }),
  ).not.toBeChecked();
  await page.getByLabel("녹화 프리셋").selectOption("obstacles");
  await expect(
    page.getByRole("checkbox", { name: "Nav2 voxel layer 원본", exact: true }),
  ).toBeChecked();
  await expect(
    page.getByRole("checkbox", { name: "카메라 원본", exact: true }),
  ).not.toBeChecked();
  await page
    .getByRole("checkbox", { name: "Depth point cloud 원본", exact: true })
    .uncheck();
  await page.getByLabel("Depth point cloud 원본 기록 Hz").fill("3.5");
  await page.getByLabel("세션 용량 MiB").fill("300");
  await expect.poll(() => reads).toBeGreaterThan(1);
  await expect(page.getByLabel("Depth point cloud 원본 기록 Hz")).toHaveValue(
    "3.5",
  );
  await page.getByRole("button", { name: "설정 적용", exact: true }).click();
  await expect(page.getByRole("status")).toContainText(
    "녹화 설정을 적용했습니다.",
  );
  expect(writes[0].enabled.point_cloud).toBe(false);
  expect(writes[0].hz.point_cloud).toBe(3.5);
  expect(writes[0].limit_mib).toBe(300);
  await page
    .getByRole("button", { name: "녹화 일시정지", exact: true })
    .click();
  await expect(
    page.getByRole("button", { name: "녹화 재개", exact: true }),
  ).toBeVisible();
  await page.getByRole("button", { name: "녹화 재개", exact: true }).click();
  await expect(
    page.getByRole("button", { name: "녹화 일시정지", exact: true }),
  ).toBeVisible();
  await page.getByRole("button", { name: "새 녹화 시작", exact: true }).click();
  await expect.poll(() => writes.length).toBe(4);
  await page.screenshot({
    path: "test-results/recording-settings.png",
    fullPage: true,
  });
});
