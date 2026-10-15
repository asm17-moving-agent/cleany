import { test, expect } from "@playwright/test";
test("absolute timeline seeks sessions, distinguishes pauses, pans without seeking, and crosses boundaries", async ({
  page,
}) => {
  const now = Date.parse("2026-10-10T12:00:00Z") / 1000;
  const a = "11111111-1111-4111-8111-111111111111",
    b = "22222222-2222-4222-8222-222222222222";
  await page.addInitScript((t) => {
    Date.now = () => t * 1000;
  }, now);
  const requests: string[] = [];
  await page.route("**/api/recordings", (r) =>
    r.fulfill({
      json: {
        sessions: [
          {
            id: a,
            start: now - 3600,
            end: now - 600,
            count: 50,
            bytes: 100,
            metadata: {},
            segments: [
              { start: now - 3500, end: now - 2400 },
              { start: now - 1800, end: now - 600 },
            ],
          },
          {
            id: b,
            start: now - 600,
            end: now - 10,
            count: 20,
            bytes: 100,
            metadata: {},
            segments: [{ start: now - 600, end: now - 10 }],
          },
        ],
        active: b,
        reason: "",
      },
    }),
  );
  await page.route("**/api/recordings/*/replay?*", (r) => {
    requests.push(r.request().url());
    return r.fulfill({ json: { rows: [], metadata: {} } });
  });
  await page.route("**/api/navigation/runs", (r) =>
    r.fulfill({ json: { runs: [] } }),
  );
  await page.routeWebSocket("**/ws", () => {});
  await page.goto("/#resources");
  const bar = page.getByTestId("timeline-track");
  await expect(bar).toBeVisible();
  await expect(page.locator("header").getByLabel("날짜 선택")).toBeVisible();
  await expect(
    page.locator("header").getByTestId("timeline-track"),
  ).toBeVisible();
  await expect(page.locator(".content .recording-timeline")).toHaveCount(0);
  let box = (await bar.boundingBox())!;
  const click = async (ago: number) => {
    box = (await bar.boundingBox())!;
    await page.mouse.click(
      box.x + box.width * (1 - ago / 3600),
      box.y + box.height / 2,
    );
  };
  await click(3000);
  await expect.poll(() => requests.at(-1)).toContain(a);
  await click(2100);
  await expect(
    page.getByRole("status").filter({ hasText: "해당 시각에 기록 없음" }),
  ).toBeVisible();
  const old = requests.length;
  await page.waitForTimeout(150);
  expect(requests.length).toBe(old);
  await click(1200);
  await expect.poll(() => requests.at(-1)).toContain(a);
  const pointer = page.getByRole("slider", { name: "재생 포인터" });
  const pb = (await pointer.boundingBox())!;
  await page.mouse.move(pb.x + 1, pb.y + pb.height / 2);
  await page.mouse.down();
  await page.mouse.move(
    box.x + box.width * (1 - 300 / 3600),
    box.y + box.height / 2,
  );
  await page.mouse.up();
  await expect.poll(() => requests.at(-1)).toContain(b);
  // Empty areas now seek too, without moving the overview window.
  const windowEnd = await pointer.getAttribute("aria-valuemax");
  box = (await bar.boundingBox())!;
  await page.mouse.move(box.x + box.width * 0.4, box.y + box.height / 2);
  await page.mouse.down();
  await page.mouse.move(box.x + box.width * 0.45, box.y + box.height / 2);
  await page.mouse.up();
  await expect(
    page.getByRole("status").filter({ hasText: "해당 시각에 기록 없음" }),
  ).toBeVisible();
  expect(await pointer.getAttribute("aria-valuemax")).toBe(windowEnd);
  expect(+(await pointer.getAttribute("aria-valuenow"))!).toBeCloseTo(
    now - 1980,
    0,
  );
  await click(300);
  await expect.poll(() => requests.at(-1)).toContain(b);
  const before = await pointer.getAttribute("aria-valuenow");
  const labels = page.getByTestId("timeline-range-ticks");
  const tb = (await labels.boundingBox())!;
  await page.mouse.move(tb.x + tb.width * 0.5, tb.y + tb.height / 2);
  await page.mouse.down();
  await page.mouse.move(tb.x + tb.width * 0.55, tb.y + tb.height / 2);
  await page.mouse.up();
  expect(await pointer.getAttribute("aria-valuenow")).toBe(before);
  const moved = +(await pointer.getAttribute("aria-valuemax"))!;
  box = (await bar.boundingBox())!;
  await page.keyboard.down("Shift");
  await page.mouse.click(box.x + box.width * 0.6, box.y + box.height / 2);
  expect(await pointer.getAttribute("aria-valuenow")).toBe(before);
  await page.mouse.move(box.x + box.width * 0.6, box.y + box.height / 2);
  await page.mouse.down();
  await page.mouse.move(box.x + box.width * 0.58, box.y + box.height / 2);
  await page.mouse.up();
  await page.keyboard.up("Shift");
  expect(await pointer.getAttribute("aria-valuenow")).toBe(before);
  expect(+(await pointer.getAttribute("aria-valuemax"))!).toBeGreaterThan(
    moved + 30,
  );
  await page.mouse.move(0, 0);
  await page.getByRole("button", { name: "라이브", exact: true }).click();
  await click(600.3);
  await page.mouse.move(0, 0);
  await page.getByRole("button", { name: "재생", exact: true }).click();
  await expect.poll(() => requests.at(-1), { timeout: 3000 }).toContain(b);
  await page.getByRole("button", { name: "일시정지", exact: true }).click();
  await page.getByLabel("날짜 선택").fill("2026-10-09");
  await expect(pointer).toHaveCount(0);
  await page.getByLabel("날짜 선택").fill("2026-10-10");
  await expect(pointer).toBeVisible();
  expect(+(await pointer.getAttribute("aria-valuemax"))!).toBe(now);
  const dayStart = +(await pointer.getAttribute("aria-valuemin"))!;
  expect(
    await page.evaluate((t) => new Date(t * 1000).getHours(), dayStart),
  ).toBe(0);
  await page.getByLabel("날짜 선택").fill("2026-10-11");
  await expect(page.getByLabel("날짜 선택")).toHaveValue("2026-10-10");
  await expect(
    page.getByRole("button", { name: "이동", exact: true }),
  ).toHaveCount(0);
  await expect(page.getByLabel("현재 시각")).toHaveText(/^\d{2}:\d{2}:\d{2}$/);
  await expect(page.getByText("읽기 전용", { exact: true })).toHaveCount(0);
  await page.setViewportSize({ width: 390, height: 844 });
  expect(
    await page.evaluate(
      () => document.documentElement.scrollWidth <= innerWidth,
    ),
  ).toBeTruthy();
  const row = await Promise.all(
    [
      page.getByLabel("날짜 선택"),
      page.getByLabel("재생 배속"),
      page.getByRole("button", { name: "라이브", exact: true }),
    ].map((v) => v.boundingBox()),
  );
  expect(
    Math.max(...row.map((v) => v!.y)) - Math.min(...row.map((v) => v!.y)),
  ).toBeLessThan(5);
  await page.screenshot({
    path: "test-results/timeline-controls-mobile.png",
    fullPage: true,
  });
  await page.getByRole("button", { name: "라이브", exact: true }).click();
  await expect(pointer).toHaveAttribute("aria-valuenow", String(now));
  const limitBox = (await labels.boundingBox())!;
  await page.mouse.move(
    limitBox.x + limitBox.width * 0.8,
    limitBox.y + limitBox.height / 2,
  );
  await page.mouse.down();
  await page.mouse.move(
    limitBox.x + limitBox.width * 0.2,
    limitBox.y + limitBox.height / 2,
  );
  await page.mouse.up();
  await expect(pointer).toHaveAttribute("aria-valuemax", String(now));
  await pointer.press("ArrowRight");
  await expect(pointer).toHaveAttribute("aria-valuenow", String(now));
  expect(
    requests.every((url) => +new URL(url).searchParams.get("at")! <= now),
  ).toBeTruthy();
});

test("precision hover preserves playback; clicks and edge drags seek tenths of a second", async ({
  page,
}) => {
  const now = Date.parse("2026-10-10T12:00:00Z") / 1000;
  await page.addInitScript((t) => {
    Date.now = () => t * 1000;
  }, now);
  const requests: string[] = [];
  await page.route("**/api/recordings", (r) =>
    r.fulfill({
      json: {
        sessions: [
          {
            id: "fine-session",
            start: now - 7200,
            end: now,
            segments: [{ start: now - 7200, end: now }],
          },
        ],
      },
    }),
  );
  await page.route("**/api/recordings/*/replay?*", (r) => {
    requests.push(r.request().url());
    return r.fulfill({ json: { rows: [], metadata: {} } });
  });
  await page.routeWebSocket("**/ws", () => {});
  await page.goto("/#resources");
  const track = page.getByTestId("timeline-track");
  const popup = page.getByTestId("timeline-fine");
  const fine = page.getByRole("slider", { name: "정밀 재생 시각" });
  const box = (await track.boundingBox())!;
  await page.mouse.move(box.x + box.width / 2, box.y + box.height / 2);
  await expect(popup).toBeVisible();
  expect(requests).toHaveLength(0);
  const fb = (await fine.boundingBox())!;
  // Move into the floating bar, retaining its original center.
  await page.mouse.move(fb.x + fb.width / 2, fb.y + fb.height / 2);
  const start = +(await fine.getAttribute("data-start"))!;
  await page.mouse.move(fb.x + fb.width * 0.98, fb.y + fb.height / 2);
  await expect
    .poll(async () => +(await fine.getAttribute("data-start"))!)
    .toBeGreaterThan(start + 2);
  expect(requests).toHaveLength(0);
  const right = +(await fine.getAttribute("data-start"))!;
  await page.mouse.move(fb.x + fb.width * 0.02, fb.y + fb.height / 2);
  await expect
    .poll(async () => +(await fine.getAttribute("data-start"))!)
    .toBeLessThan(right - 2);
  await page.mouse.move(fb.x + fb.width * 0.5, fb.y + fb.height / 2);
  const stopped = +(await fine.getAttribute("data-start"))!;
  await page.waitForTimeout(200);
  expect(+(await fine.getAttribute("data-start"))!).toBe(stopped);
  await page.mouse.click(fb.x + fb.width * 0.543, fb.y + fb.height / 2);
  await expect.poll(() => requests.length).toBeGreaterThan(0);
  const pointer = page.getByRole("slider", { name: "재생 포인터" });
  const selected = +(await pointer.getAttribute("aria-valuenow"))!;
  expect(selected).toBeCloseTo(Math.round((stopped + 0.543 * 60) * 10) / 10, 1);
  await page.mouse.move(fb.x + fb.width * 0.5, fb.y + fb.height / 2);
  await page.mouse.down();
  await page.mouse.move(fb.x + fb.width * 0.98, fb.y + fb.height / 2);
  const edgeSelected = +(await pointer.getAttribute("aria-valuenow"))!;
  await expect
    .poll(async () => +(await pointer.getAttribute("aria-valuenow"))!)
    .toBeGreaterThan(edgeSelected + 2);
  await page.mouse.up();
  await page.mouse.move(fb.x + fb.width * 0.5, fb.y + fb.height / 2);
  await fine.focus();
  const keyboard = +(await pointer.getAttribute("aria-valuenow"))!;
  await fine.press("ArrowRight");
  await expect(pointer).toHaveAttribute(
    "aria-valuenow",
    String(Math.round((keyboard + 0.1) * 10) / 10),
  );
  await page.screenshot({
    path: "test-results/precision-timeline.png",
    fullPage: true,
  });
  await page.mouse.move(0, 0);
  await expect(popup).toHaveCount(0);
  const closedAt = await pointer.getAttribute("aria-valuenow");
  await page.waitForTimeout(200);
  expect(await pointer.getAttribute("aria-valuenow")).toBe(closedAt);
  await page.setViewportSize({ width: 390, height: 844 });
  const narrow = (await track.boundingBox())!;
  await page.mouse.move(narrow.x + 2, narrow.y + narrow.height / 2);
  await expect(popup).toBeVisible();
  const small = (await popup.boundingBox())!;
  expect(small.x).toBeGreaterThanOrEqual(narrow.x - 1);
  expect(small.x + small.width).toBeLessThanOrEqual(
    narrow.x + narrow.width + 1,
  );
  expect(
    await page.evaluate(
      () => document.documentElement.scrollWidth <= innerWidth,
    ),
  ).toBeTruthy();
});

test("live timeline and recording intervals refresh every second", async ({
  page,
}) => {
  const date = new Date("2026-10-10T12:00:00Z");
  await page.clock.install({ time: date });
  let polls = 0;
  await page.route("**/api/recordings", (r) => {
    polls++;
    return r.fulfill({ json: { sessions: [] } });
  });
  await page.routeWebSocket("**/ws", () => {});
  await page.goto("/#resources");
  const pointer = page.getByRole("slider", { name: "재생 포인터" });
  await expect(pointer).toBeVisible();
  await expect.poll(() => polls).toBe(1);
  const initial = +(await pointer.getAttribute("aria-valuenow"))!;
  await page.clock.runFor(500);
  await expect(pointer).toHaveAttribute("aria-valuenow", String(initial));
  expect(polls).toBe(1);
  await page.clock.runFor(600);
  await expect
    .poll(async () => +(await pointer.getAttribute("aria-valuenow"))!)
    .toBeGreaterThanOrEqual(initial + 1);
  await expect.poll(() => polls).toBe(2);
});
