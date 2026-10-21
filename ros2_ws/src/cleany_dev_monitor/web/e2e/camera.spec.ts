import { test, expect } from "@playwright/test";

test("camera preview pauses, clears stale frames and never polls live images during replay", async ({
  page,
}) => {
  const session = "44444444-4444-4444-8444-444444444444";
  const now = Date.now() / 1000;
  const topic = "/camera/color/image_raw";
  const graph = {
    seq: 1,
    kind: "graph",
    key: "ros",
    received_at: now,
    ros_time: null,
    data: { topics: [
      { name: topic, types: ["sensor_msgs/msg/Image"], camera_supported: true },
      { name: "/camera/depth/image_raw", types: ["sensor_msgs/msg/Image"], camera_supported: false },
    ] },
  };
  const jpeg = await page.evaluate(() => {
    const canvas = document.createElement("canvas");
    canvas.width = canvas.height = 8;
    const context = canvas.getContext("2d")!;
    context.fillStyle = "#4080c0";
    context.fillRect(0, 0, 8, 8);
    return canvas.toDataURL("image/jpeg").split(",")[1];
  });
  await page.route("**/api/recordings", (route) =>
    route.fulfill({
      json: {
        active: null,
        sessions: [
          {
            id: session,
            start: now - 60,
            end: now - 5,
            count: 2,
            runtime_count: 1,
            bytes: 100,
            metadata: {},
          },
        ],
      },
    }),
  );
  await page.route("**/api/recordings/*/replay?*", (route) =>
    route.fulfill({ json: { rows: [graph], metadata: {} } }),
  );
  await page.route("**/api/recording/settings", (route) =>
    route.fulfill({ status: 503, json: {} }),
  );
  await page.routeWebSocket("**/ws", (ws) =>
    ws.send(
      JSON.stringify({
        initial: true,
        sequence: 1,
        boot_id: "camera-fixture",
        rows: [graph],
      }),
    ),
  );
  let requests = 0,
    stale = false;
  await page.route("**/api/camera?*", (route) => {
    requests++;
    return stale
      ? route.fulfill({ status: 202, json: { state: "stale" } })
      : route.fulfill({
          contentType: "image/jpeg",
          body: Buffer.from(jpeg, "base64"),
          headers: {
            "X-Frame-Age": "0.01",
            "X-Image-Width": "8",
            "X-Image-Height": "8",
          },
        });
  });
  await page.goto("/#camera");
  await expect(
    page.getByRole("heading", { name: "카메라", exact: true }),
  ).toBeVisible();
  expect(requests).toBe(0);
  await expect(page.locator('select[aria-label="카메라 토픽"] option[value="/camera/depth/image_raw"]')).toHaveCount(0);
  await page.getByLabel("카메라 토픽").selectOption(topic);
  const image = page.getByRole("img", { name: topic, exact: true });
  await expect(image).toBeVisible();
  await expect
    .poll(() => image.evaluate((img: HTMLImageElement) => img.naturalWidth))
    .toBe(8);
  await page.getByRole("button", { name: "일시정지", exact: true }).click();
  await expect(image).toHaveCount(0);
  const paused = requests;
  await page.waitForTimeout(500);
  expect(requests).toBe(paused);
  await page.getByRole("button", { name: "재개", exact: true }).click();
  await expect(image).toBeVisible();
  stale = true;
  await expect(page.locator(".camera-viewport")).toContainText("수신 지연");
  await expect(image).toHaveCount(0);
  await page.getByRole("button", { name: "기록", exact: true }).click();
  await page.getByText("기록 관리, 내보내기", { exact: true }).first().click();
  await page
    .getByRole("button", { name: "시각으로 이동", exact: true })
    .click();
  await expect(page.locator(".header-status")).toContainText("REPLAY");
  const historical = requests;
  await page.getByRole("button", { name: "카메라", exact: true }).click();
  await expect(page.getByLabel("카메라 토픽")).toBeDisabled();
  await expect(page.locator(".camera-viewport")).toContainText(
    "원본 rosbag2 영상은 RViz에서 재생하세요.",
  );
  await page.waitForTimeout(500);
  expect(requests).toBe(historical);
});
