import { test, expect } from "@playwright/test";
import { applyRuntimeEvents } from "../src/lib/runtime-observation";

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
  // A final transition may arrive after the last periodic snapshot. Compare
  // the same observed event boundary as the UI and allow an active transition
  // to settle between the HTTP response and the browser's WebSocket refresh.
  await expect
    .poll(async () => {
      const snapshot = await (await request.get("/api/snapshot")).json();
      const runtime = applyRuntimeEvents(snapshot.rows).find(
        (row) => row.kind === "runtime",
      )?.data;
      return (
        !!runtime &&
        (
          await page
            .locator(".execution-summary")
            .first()
            .locator(".badge")
            .first()
            .innerText()
        ).trim() === runtime.state
      );
    })
    .toBe(true);
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
    page.getByRole("heading", { name: "전체 경로와 주행 진행", exact: true }),
  ).toBeVisible();
  await expect(page.locator(".layer-status")).toContainText("수신");
  const canvas = page.getByLabel("ROS 2D navigation map");
  const mapToggle = page.getByRole("checkbox", { name: "지도", exact: true });
  await expect(mapToggle).toBeChecked();
  await page.evaluate(
    () =>
      new Promise<void>((resolve) =>
        requestAnimationFrame(() => requestAnimationFrame(() => resolve())),
      ),
  );
  await canvas.evaluate(
    (el: HTMLCanvasElement & { verificationPixels?: Uint8ClampedArray }) => {
      el.verificationPixels = el
        .getContext("2d")!
        .getImageData(0, 0, el.width, el.height).data;
    },
  );
  const changedMapArea = () =>
    canvas.evaluate(
      (el: HTMLCanvasElement & { verificationPixels?: Uint8ClampedArray }) => {
        const before = el.verificationPixels!;
        const after = el
          .getContext("2d")!
          .getImageData(0, 0, el.width, el.height).data;
        let changed = 0;
        for (let i = 0; i < after.length; i += 4)
          if (
            before[i] !== after[i] ||
            before[i + 1] !== after[i + 1] ||
            before[i + 2] !== after[i + 2]
          )
            changed++;
        return changed / (after.length / 4);
      },
    );
  // A valid occupancy map can contain only three shades after the simulator
  // stops and live pose overlays expire. Check real map-layer visibility
  // instead of depending on the number of colors in moving overlays.
  await mapToggle.uncheck();
  await expect.poll(changedMapArea).toBeGreaterThan(0.02);
  await mapToggle.check();
  await expect.poll(changedMapArea).toBeLessThan(0.02);
  await page.screenshot({
    path: "test-results/navigation.png",
    fullPage: true,
  });
});
