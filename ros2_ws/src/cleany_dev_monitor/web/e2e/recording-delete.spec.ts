import { test, expect, type Page } from "@playwright/test";

const old = "11111111-1111-4111-8111-111111111111";
const other = "22222222-2222-4222-8222-222222222222";
const current = "33333333-3333-4333-8333-333333333333";

function recording(id: string, deletable = true) {
  const now = Date.now() / 1000;
  const start = now - (id === old ? 900 : id === other ? 600 : 300);
  return {
    id,
    count: 5,
    start,
    end: start + 200,
    bytes: 1024 ** 2,
    deletable,
    metadata: {},
    segments: [{ start, end: start + 200 }],
  };
}

async function setup(page: Page) {
  const replayed: string[] = [];
  await page.route("**/api/navigation/runs", (route) =>
    route.fulfill({ json: { runs: [] } }),
  );
  await page.route("**/api/recording/settings", (route) =>
    route.fulfill({
      json: {
        settings: {
          active: false,
          preset: "standard",
          limit_mib: 100,
          enabled: {},
          hz: {},
          topics: {},
          sources: {},
          source_enabled: {},
          source_hz: {},
        },
        status: {},
      },
    }),
  );
  await page.route("**/api/recordings/*/replay?*", (route) => {
    replayed.push(route.request().url());
    return route.fulfill({ json: { rows: [], metadata: {} } });
  });
  await page.routeWebSocket("**/ws", (ws) =>
    ws.send(JSON.stringify({ initial: true, sequence: 1, rows: [] })),
  );
  return replayed;
}

test("delete confirms, protects open writers, reports failure and exits deleted replay", async ({
  page,
}) => {
  const replayed = await setup(page);
  let sessions = [recording(old), recording(other), recording(current, false)];
  await page.route("**/api/recordings", (route) =>
    route.fulfill({ json: { sessions, active: current, reason: "" } }),
  );
  let attempts = 0;
  let complete: (() => void) | undefined;
  await page.route(`**/api/recordings/${old}`, async (route) => {
    expect(route.request().method()).toBe("DELETE");
    expect(route.request().headers().origin).toBe(
      new URL(route.request().url()).origin,
    );
    attempts++;
    if (attempts === 1) {
      await route.fulfill({
        status: 409,
        body: "녹화를 정지한 뒤 삭제하세요.",
      });
      return;
    }
    await new Promise<void>((resolve) => {
      complete = resolve;
    });
    sessions = sessions.filter((s) => s.id !== old);
    await route.fulfill({ json: { deleted: old } });
  });
  await page.goto("/#recordings");
  await page
    .locator("summary")
    .filter({ hasText: "기록 관리, 내보내기" })
    .click();
  const oldRow = page.getByRole("row").filter({ hasText: old.slice(0, 8) });
  const otherRow = page.getByRole("row").filter({ hasText: other.slice(0, 8) });
  const activeRow = page
    .getByRole("row")
    .filter({ hasText: current.slice(0, 8) });
  await expect(activeRow.getByRole("button", { name: /삭제/ })).toBeDisabled();
  await expect(oldRow.getByRole("button", { name: /삭제/ })).toBeEnabled();
  expect(await oldRow.locator("td").last().getAttribute("class")).toContain(
    "recording-delete-cell",
  );
  await page.screenshot({
    path: "test-results/recording-delete.png",
    fullPage: true,
  });

  page.once("dialog", async (dialog) => {
    expect(dialog.message()).toContain("SQLite와 센서 원본");
    await dialog.dismiss();
  });
  await oldRow.getByRole("button", { name: /삭제/ }).click();
  expect(attempts).toBe(0);
  await expect(oldRow).toBeVisible();

  await oldRow.getByRole("button", { name: "시각으로 이동" }).click();
  await expect(page.locator(".header-status")).toContainText("REPLAY");
  await expect.poll(() => replayed.at(-1)).toContain(old);
  page.once("dialog", (dialog) => dialog.accept());
  await oldRow.getByRole("button", { name: /삭제/ }).click();
  await expect(
    page.getByRole("status").filter({ hasText: "삭제 실패" }),
  ).toContainText("녹화를 정지");
  await expect(oldRow).toBeVisible();
  await expect(oldRow.getByRole("button", { name: /삭제/ })).toBeEnabled();

  page.once("dialog", (dialog) => dialog.accept());
  await oldRow.getByRole("button", { name: /삭제/ }).click();
  await expect(oldRow.getByRole("button", { name: /삭제/ })).toHaveAttribute(
    "aria-busy",
    "true",
  );
  await expect(otherRow.getByRole("button", { name: /삭제/ })).toBeDisabled();
  await expect.poll(() => !!complete).toBe(true);
  complete!();
  await expect(oldRow).toHaveCount(0);
  await expect(otherRow).toBeVisible();
  await expect(page.locator(".header-status")).not.toContainText("REPLAY");
  await expect(
    page.getByRole("status").filter({ hasText: "녹화를 삭제했습니다." }),
  ).toBeVisible();
  await page.getByRole("button", { name: "새로고침", exact: true }).click();
  await expect(oldRow).toHaveCount(0);
  expect(attempts).toBe(2);
});

test("stopped current recording can be deleted and empty catalog is displayed", async ({
  page,
}) => {
  await setup(page);
  let sessions = [recording(current)];
  await page.route("**/api/recordings", (route) =>
    route.fulfill({ json: { sessions, active: current, reason: "" } }),
  );
  await page.route(`**/api/recordings/${current}`, (route) => {
    sessions = [];
    return route.fulfill({ json: { deleted: current } });
  });
  await page.goto("/#recordings");
  await page
    .locator("summary")
    .filter({ hasText: "기록 관리, 내보내기" })
    .click();
  const row = page.getByRole("row").filter({ hasText: current.slice(0, 8) });
  await expect(row).toContainText("기록 정지");
  await expect(row.getByRole("button", { name: /삭제/ })).toBeEnabled();
  page.once("dialog", (dialog) => dialog.accept());
  await row.getByRole("button", { name: /삭제/ }).click();
  await expect(page.getByText("기록 없음", { exact: true })).toBeVisible();
  await expect(row).toHaveCount(0);
});
