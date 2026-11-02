import { test, expect } from "@playwright/test";

test("resource sorting, stale observation and narrow layout", async ({
  page,
}) => {
  const at = Date.now() / 1000 - 10;
  const data = {
    at,
    host: "fixture-host",
    interval_s: 1,
    cpu_percent: 25,
    memory_percent: 40,
    cpu_cores: [20, 30],
    memory: { used: 400, total: 1000 },
    swap: { used: 0, total: 0, percent: 0 },
    disk: { used: 500, total: 1000, percent: 50 },
    disk_devices: [],
    networks: [],
    processes_cpu: [
      { pid: 1, name: "cpu-worker", cpu_percent: 80, rss_bytes: 1024 },
    ],
    processes_memory: [
      { pid: 2, name: "memory-worker", cpu_percent: 0, rss_bytes: 4096 },
    ],
    process_count: 2,
    denied_processes: 1,
    errors: [],
    temperatures: [],
    gpu: [],
    history: [
      { at: at - 1, cpu_percent: 20, memory_percent: 39 },
      { at, cpu_percent: 25, memory_percent: 40 },
    ],
  };
  await page.route("**/api/**", (route) =>
    route.fulfill({
      json: { sessions: [], labels: {}, settings: {}, rows: [] },
    }),
  );
  await page.routeWebSocket("**/ws", (ws) =>
    ws.send(
      JSON.stringify({
        initial: true,
        boot_id: "fixture",
        sequence: 1,
        rows: [{ seq: 1, kind: "resources", key: "host", data }],
      }),
    ),
  );
  await page.goto("/#resources");
  await expect(page.getByText("서버 호스트, fixture-host")).toBeVisible();
  await expect(
    page.getByText(
      "리소스 수집이 지연되었습니다. 아래 값은 마지막 관측값입니다.",
    ),
  ).toBeVisible();
  const cpu = page.getByRole("region", { name: "CPU 상세", exact: true });
  await expect(cpu.locator(".resource-trend-heading")).toContainText("25.0%");
  await expect(cpu.locator(".resource-trend-stats")).toContainText("22.5%");
  await expect(cpu.locator(".resource-trend-stats")).toContainText("20.0%");
  await expect(cpu.locator(".resource-trend-details")).toContainText("2개");
  await expect(
    page.getByRole("region", { name: "메모리 상세", exact: true }),
  ).toContainText("비활성, 할당 없음");
  await expect(page.getByText("cpu-worker", { exact: true })).toBeVisible();
  await page.getByLabel("프로세스 정렬").selectOption("memory");
  await expect(page.getByText("memory-worker", { exact: true })).toBeVisible();
  await page.setViewportSize({ width: 390, height: 844 });
  expect(
    await page.evaluate(
      () => document.documentElement.scrollWidth <= innerWidth,
    ),
  ).toBeTruthy();
});

test("real host updates, reconnect and replay", async ({ page, request }) => {
  test.skip(
    !process.env.RESOURCE_MONITOR_URL,
    "Requires actual host resource preview",
  );
  const url = process.env.RESOURCE_MONITOR_URL!;
  const errors: string[] = [];
  page.on("pageerror", (e) => errors.push(e.message));
  await page.goto(url + "/#resources");
  await expect(page.getByText(/서버 호스트, /)).toBeVisible();
  const get = async () =>
    (await (await request.get(url + "/api/snapshot")).json()).rows.find(
      (r: any) => r.kind === "resources",
    );
  const first = await get();
  await expect.poll(async () => (await get()).seq).toBeGreaterThan(first.seq);
  await page.screenshot({
    path: "test-results/resources-desktop.png",
    fullPage: true,
  });
  await page.setViewportSize({ width: 390, height: 844 });
  expect(
    await page.evaluate(
      () => document.documentElement.scrollWidth <= innerWidth,
    ),
  ).toBeTruthy();
  await page.screenshot({
    path: "test-results/resources-mobile.png",
    fullPage: true,
  });
  await page.setViewportSize({ width: 1440, height: 1000 });
  await page.reload();
  await expect(page.getByText(/서버 호스트, /)).toBeVisible();
  const sessions = (await (await request.get(url + "/api/recordings")).json())
    .sessions;
  const session = sessions.find((s: any) => s.count > 0);
  await page.getByRole("button", { name: "기록", exact: true }).click();
  await page.getByText("기록 관리, 내보내기", { exact: true }).first().click();
  await page
    .locator("tr")
    .filter({ hasText: session.id.slice(0, 8) })
    .getByRole("button", { name: "시각으로 이동", exact: true })
    .click();
  await page.getByRole("slider", { name: "재생 포인터" }).press("End");
  await page.getByRole("button", { name: "자원", exact: true }).click();
  await expect(page.locator(".resource-source")).toContainText("기록 재생");
  await expect(page.getByText(/서버 호스트, /)).toBeVisible();
  await page.screenshot({
    path: "test-results/resources-replay.png",
    fullPage: true,
  });
  await page.getByRole("button", { name: "라이브" }).click();
  await expect(page.locator(".resource-source")).toContainText("실시간 관측");
  expect(errors).toEqual([]);
});
