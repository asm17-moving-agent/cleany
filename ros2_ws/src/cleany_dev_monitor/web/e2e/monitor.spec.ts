import { test, expect } from "@playwright/test";

test("observed FSM, topic subscription, correlated logs and recording replay", async ({
  page,
  request,
}) => {
  test.skip(
    !process.env.MONITOR_URL,
    "Set MONITOR_URL to a running ROS monitor",
  );
  const errors: string[] = [];
  page.on("pageerror", (e) => errors.push(e.message));
  await page.goto("/");
  await expect(page.getByRole("region", { name: "실행 그래프" })).toBeVisible();
  await expect(page.locator(".react-flow__node").first()).toBeVisible();
  const runtime = (await (await request.get("/api/snapshot")).json()).rows.find(
    (r: any) => r.kind === "runtime",
  ).data;
  await expect(page.locator(".execution-summary").first()).toContainText(
    runtime.state,
  );
  await page.getByRole("button", { name: "ROS 탐색기" }).click();
  await expect(page.getByPlaceholder("토픽 이름 또는 타입 검색")).toBeVisible();
  await page
    .getByPlaceholder("토픽 이름 또는 타입 검색")
    .fill("/parameter_events");
  const check = page.getByRole("checkbox", {
    name: "구독 /parameter_events",
    exact: true,
  });
  await check.click();
  await expect(check).toBeChecked();
  await check.click();
  await expect(check).not.toBeChecked();
  await page.getByRole("button", { name: "로그" }).click();
  await expect(
    page.getByRole("heading", { name: "이벤트 타임라인" }),
  ).toBeVisible();
  await page.getByRole("button", { name: "기록 & 재생" }).click();
  await expect(
    page.getByRole("heading", { name: "저장된 관측 세션" }),
  ).toBeVisible();
  const sessions = (await (await request.get("/api/recordings")).json())
    .sessions;
  const session = sessions.find((s: any) => s.count > 0);
  await page
    .locator("tr")
    .filter({ hasText: session.id.slice(0, 8) })
    .getByRole("button", { name: "재생", exact: true })
    .click();
  await expect(page.locator(".replay-bar")).toBeVisible();
  await page.getByRole("slider", { name: "재생 시각" }).focus();
  await page.getByRole("slider", { name: "재생 시각" }).press("End");
  await expect(page.locator(".header-status")).toContainText("REPLAY");
  await page.getByRole("button", { name: "실시간으로" }).click();
  await expect(page.locator(".replay-bar")).toHaveCount(0);
  expect(errors).toEqual([]);
  await page.screenshot({ path: "test-results/execution.png", fullPage: true });
});

test("Gazebo map, TF and both costmaps render", async ({ page, request }) => {
  test.skip(
    !process.env.NAV_MONITOR_URL,
    "Set NAV_MONITOR_URL to a Gazebo monitor",
  );
  const url = process.env.NAV_MONITOR_URL!;
  const snapshot = await (await request.get(url + "/api/snapshot")).json();
  for (const layer of ["map", "local_costmap", "global_costmap", "path"])
    expect(
      snapshot.rows.find((r: any) => r.kind === "layer" && r.key === layer).data
        .valid,
    ).toBe(true);
  await page.goto(url + "/#navigation");
  await expect(
    page.getByRole("heading", { name: "2D Navigation" }),
  ).toBeVisible();
  await expect(page.locator(".layer-status")).toContainText("수신");
  const canvas = page.getByLabel("ROS 2D navigation map");
  const colors = await canvas.evaluate((el: HTMLCanvasElement) => {
    const data = el
      .getContext("2d")!
      .getImageData(0, 0, el.width, el.height).data;
    const seen = new Set<string>();
    for (let i = 0; i < data.length; i += 400)
      seen.add(`${data[i]},${data[i + 1]},${data[i + 2]}`);
    return seen.size;
  });
  expect(colors).toBeGreaterThan(8);
  await page.screenshot({
    path: "test-results/navigation.png",
    fullPage: true,
  });
});
