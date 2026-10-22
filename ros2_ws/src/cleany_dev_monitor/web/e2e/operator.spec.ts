import { test, expect, type Page } from "@playwright/test";
import { rows, run, transport } from "./fixtures/manipulation";
import { type Data } from "../src/lib/monitor";

async function fixture(page: Page) {
  const values = rows([run(true)]);
  Object.assign(values[0].data, { control_enabled: true });
  const wire = await transport(page, values);
  const sent: Data[] = [];
  const status: Data = {
    enabled: true,
    authenticated: false,
    actor: "",
    jobs: [],
    active_job: null,
    state: {
      available: true,
      guard_fresh: true,
      guard: { state: "ready" },
      inhibited: false,
      operation_mode: "pick_return_release",
      blocked: [{ execution_id: "blocked-execution", revision: 7 }],
    },
    runtime: {},
    capabilities: {
      prepare: "",
      inspect: "",
      run: "",
      return_arm: "",
      recover: "",
      shutdown: "",
    },
  };
  let loseResponse = false;
  let rejectRequest = false;
  await page.route("**/api/control/**", async (route) => {
    const path = new URL(route.request().url()).pathname;
    if (path.endsWith("/status")) return route.fulfill({ json: status });
    const data = route.request().postDataJSON();
    if (path.endsWith("/login")) {
      status.authenticated = true;
      status.actor = data.name;
      return route.fulfill({ json: { authenticated: true } });
    }
    if (path.endsWith("/logout")) {
      status.authenticated = false;
      return route.fulfill({ json: {} });
    }
    sent.push({ path, ...data });
    if (path.endsWith("/jobs")) {
      if (loseResponse) {
        loseResponse = false;
        return route.abort("failed");
      }
      if (rejectRequest)
        return route.fulfill({
          status: 409,
          body: "차단 기록이 변경되었습니다.",
        });
      if (["approve", "skip"].includes(data.action) && status.proposal)
        status.proposal.state = "CONSUMED";
      status.jobs = [
        {
          id: data.request_id,
          action: data.action,
          actor: status.actor,
          state: "SUCCEEDED",
        },
      ];
    }
    return route.fulfill({ json: { accepted: true, stop_confirmed: false } });
  });
  await page.goto("/#manipulation");
  const panel = page.getByRole("region", { name: "실물 조작 제어" });
  async function login() {
    await panel
      .getByLabel("조작자 이름", { exact: true })
      .fill("테스트 운영자");
    await panel.getByLabel("조작키", { exact: true }).fill("fixture-key");
    await panel
      .getByRole("button", { name: "조작 권한 연결", exact: true })
      .click();
    await expect(panel.getByText("조작자: 테스트 운영자")).toBeVisible();
  }
  return {
    panel,
    sent,
    status,
    wire,
    values,
    login,
    lose: () => {
      loseResponse = true;
    },
    reject: () => {
      rejectRequest = true;
    },
  };
}

test("viewers authenticate before an exact single run request; server gates are visible", async ({
  page,
}) => {
  const f = await fixture(page);
  const runButton = f.panel.getByRole("button", {
    name: "물체 1개 실행",
    exact: true,
  });
  await expect(runButton).toBeDisabled();
  expect(f.sent).toEqual([]);
  if (process.env.OPERATOR_VISUAL_DIR)
    await f.panel.screenshot({
      path: process.env.OPERATOR_VISUAL_DIR + "/viewer.png",
    });
  await f.login();
  if (process.env.OPERATOR_VISUAL_DIR)
    await f.panel.screenshot({
      path: process.env.OPERATOR_VISUAL_DIR + "/ready.png",
    });
  await runButton.click();
  await expect.poll(() => f.sent.length).toBe(1);
  expect(f.sent[0].action).toBe("run");
  expect(f.sent[0].request_id).toMatch(/^[a-f0-9-]{36}$/);
  expect(f.sent[0].args).toEqual({});
  f.status.capabilities.run = "실행 차단을 먼저 해제하세요.";
  await f.panel.getByRole("button", { name: "상태 다시 확인" }).click();
  await expect(runButton).toBeDisabled();
  await expect(f.panel).toContainText("실행 차단을 먼저 해제하세요.");
});

test("a reviewed proposal approves only its immutable ID and the next task waits again", async ({
  page,
}) => {
  const f = await fixture(page);
  const id = "1b567183-65af-4f3b-a6e6-12e843b43291";
  f.status.proposal = {
    id,
    state: "READY",
    expires_at: Date.now() / 1000 + 80,
    operation_mode: "pick_return_release",
    image_available: true,
    target: {
      object_id: 3,
      label: "cup",
      distance_m: 0.42,
      confidence: 0.94,
      classification_reason: "빈 종이컵",
    },
    reason: "조건을 통과한 물체 중 로봇에 가장 가깝습니다.",
  };
  await page.route("**/api/control/proposals/*/image", (route) =>
    route.fulfill({
      contentType: "image/svg+xml",
      body: '<svg xmlns="http://www.w3.org/2000/svg" width="640" height="360"><rect width="640" height="360" fill="#243447"/><rect x="230" y="90" width="120" height="180" rx="8" fill="#eee" stroke="#00efb0" stroke-width="6"/><text x="240" y="70" fill="white" font-size="24">3 · cup</text></svg>',
    }),
  );
  await f.login();
  const card = page.getByRole("region", { name: "다음 작업 제안" });
  await expect(card).toContainText("3번 · cup");
  await expect(card).toContainText("원래 위치에 놓기");
  await expect(card.getByRole("img")).toBeVisible();
  expect(f.sent).toHaveLength(0);
  if (process.env.OPERATOR_VISUAL_DIR)
    await card.screenshot({
      path: process.env.OPERATOR_VISUAL_DIR + "/proposal-desktop.png",
    });
  await card.getByRole("button", { name: "승인하여 1회 실행" }).click();
  await expect.poll(() => f.sent.length).toBe(1);
  expect(f.sent[0].action).toBe("approve");
  expect(f.sent[0].args).toEqual({ proposal_id: id });
  await expect(
    card.getByRole("button", { name: "승인하여 1회 실행" }),
  ).toBeDisabled();
  f.status.proposal = {
    ...f.status.proposal,
    id: "2b567183-65af-4f3b-a6e6-12e843b43291",
    state: "READY",
  };
  await f.panel.getByRole("button", { name: "상태 다시 확인" }).click();
  await expect(
    card.getByRole("button", { name: "승인하여 1회 실행" }),
  ).toBeEnabled();
  expect(f.sent).toHaveLength(1);
  await page.setViewportSize({ width: 390, height: 844 });
  if (process.env.OPERATOR_VISUAL_DIR)
    await card.screenshot({
      path: process.env.OPERATOR_VISUAL_DIR + "/proposal-mobile.png",
    });
  await card.getByRole("button", { name: "이 물체 건너뛰기" }).click();
  await expect.poll(() => f.sent.length).toBe(2);
  expect(f.sent[1].action).toBe("skip");
  expect(f.sent[1].args.proposal_id).toBe(f.status.proposal.id);
});

test("expired proposals cannot execute and can request a fresh observation", async ({
  page,
}) => {
  const f = await fixture(page);
  f.status.proposal = {
    id: "1b567183-65af-4f3b-a6e6-12e843b43291",
    state: "READY",
    expires_at: Date.now() / 1000 - 1,
    target: { object_id: 1, label: "cup", distance_m: 0.4, confidence: 0.9 },
  };
  await f.login();
  const card = page.getByRole("region", { name: "다음 작업 제안" });
  await expect(
    card.getByRole("button", { name: "승인하여 1회 실행" }),
  ).toBeDisabled();
  await expect(
    card.getByRole("button", { name: "이 물체 건너뛰기" }),
  ).toBeDisabled();
  await card.getByRole("button", { name: "다시 제안받기" }).click();
  await expect.poll(() => f.sent.length).toBe(1);
  expect(f.sent[0].action).toBe("propose");
  expect(f.sent[0].args).toEqual({});
});

test("torque, gripper and recovery confirmations are explicit and revisions stay reviewed", async ({
  page,
}) => {
  const f = await fixture(page);
  await f.login();
  await f.panel.getByRole("button", { name: "로봇 준비", exact: true }).click();
  let dialog = page.getByRole("dialog");
  await expect(
    dialog.getByRole("button", { name: "확인 후 요청" }),
  ).toBeDisabled();
  await dialog.getByLabel("모터 토크 활성화를 확인했습니다.").check();
  await dialog.getByRole("button", { name: "확인 후 요청" }).click();
  await expect(dialog).toHaveCount(0);
  expect(f.sent[0].args).toEqual({ confirm_torque_on: true });
  await f.panel
    .getByRole("button", { name: "팔 기본 자세 복귀", exact: true })
    .click();
  dialog = page.getByRole("dialog");
  if (process.env.OPERATOR_VISUAL_DIR)
    await page.screenshot({
      path: process.env.OPERATOR_VISUAL_DIR + "/return-confirm.png",
    });
  await dialog.getByLabel("복귀할 팔").selectOption("left");
  await dialog.getByLabel("그리퍼 열기와 팔 복귀를 확인했습니다.").check();
  await dialog.getByRole("button", { name: "확인 후 요청" }).click();
  await expect(dialog).toHaveCount(0);
  expect(f.sent[1].args).toEqual({ arm: "left", confirm_open: true });
  await f.panel
    .getByRole("button", { name: "실행 차단 해제", exact: true })
    .click();
  f.status.state.blocked[0].revision = 8;
  await page.waitForTimeout(2200);
  dialog = page.getByRole("dialog");
  await expect(dialog.getByRole("combobox")).toContainText("revision 7");
  await dialog.getByLabel("로봇이 정지했음을 확인했습니다.").check();
  await dialog.getByLabel("양쪽 그리퍼가 비었음을 확인했습니다.").check();
  await dialog.getByLabel("확인 내용").fill("실물 확인 완료");
  await dialog.getByRole("button", { name: "확인 후 요청" }).click();
  await expect.poll(() => f.sent.length).toBe(3);
  expect(f.sent[2].args.revision).toBe(7);
});

test("a lost response retries only the same UUID; rejected requests can be corrected", async ({
  page,
}) => {
  const f = await fixture(page);
  await f.login();
  f.lose();
  await f.panel
    .getByRole("button", { name: "물체 1개 실행", exact: true })
    .click();
  await expect(
    f.panel.getByRole("button", { name: "같은 요청 다시 보내기" }),
  ).toBeVisible();
  await expect(
    f.panel.getByRole("button", { name: "물체 1개 실행", exact: true }),
  ).toBeDisabled();
  await f.panel.getByRole("button", { name: "같은 요청 다시 보내기" }).click();
  await expect.poll(() => f.sent.length).toBe(2);
  expect(f.sent[1].request_id).toBe(f.sent[0].request_id);
  await expect(
    f.panel.getByRole("button", { name: "같은 요청 다시 보내기" }),
  ).toHaveCount(0);
  f.reject();
  await f.panel
    .getByRole("button", { name: "장면 인식만", exact: true })
    .click();
  await expect(f.panel).toContainText("차단 기록이 변경되었습니다.");
  await expect(
    f.panel.getByRole("button", { name: "장면 인식만", exact: true }),
  ).toBeEnabled();
  await expect(
    f.panel.getByRole("button", { name: "같은 요청 다시 보내기" }),
  ).toHaveCount(0);
});

test("stop ACK waits for physical evidence and narrow screens fit controls", async ({
  page,
}) => {
  const f = await fixture(page);
  await f.login();
  f.status.state.active_execution_id = "live-execution";
  await f.panel.getByRole("button", { name: "상태 다시 확인" }).click();
  await f.panel
    .getByRole("button", { name: "즉시 중단 요청", exact: true })
    .click();
  await expect(f.panel).toContainText("실물 정지 확인을 기다리고 있습니다.");
  expect(f.sent[0].mode).toBe("IMMEDIATE");
  await f.panel
    .getByRole("button", { name: "현재 단계 후 중단", exact: true })
    .click();
  expect(f.sent[1].mode).toBe("CHECKPOINT");
  await page.getByRole("button", { name: "사이드바 접기" }).click();
  await page.setViewportSize({ width: 620, height: 900 });
  expect(
    await page.evaluate(
      () => document.documentElement.scrollWidth <= innerWidth,
    ),
  ).toBe(true);
  if (process.env.OPERATOR_VISUAL_DIR)
    await f.panel.screenshot({
      path: process.env.OPERATOR_VISUAL_DIR + "/mobile.png",
    });
});

test("disabling the feature removes control UI without posting any command", async ({
  page,
}) => {
  const f = await fixture(page);
  await f.login();
  Object.assign(f.values[0].data, { control_enabled: false });
  f.wire.update(f.values);
  await expect(f.panel).toHaveCount(0);
  expect(f.sent).toEqual([]);
});

test("slow status responses stay usable without overlapping polls", async ({
  page,
}) => {
  const f = await fixture(page);
  await f.login();
  let active = 0,
    maximum = 0,
    requests = 0;
  await page.route("**/api/control/status", async (route) => {
    requests++;
    active++;
    maximum = Math.max(maximum, active);
    await new Promise((resolve) => setTimeout(resolve, 3200));
    active--;
    await route.fulfill({ json: f.status });
  });
  await f.panel.getByRole("button", { name: "상태 다시 확인" }).click();
  await page.waitForTimeout(7500);
  expect(requests).toBeGreaterThanOrEqual(2);
  expect(maximum).toBe(1);
  await expect(
    f.panel.getByRole("button", { name: "물체 1개 실행", exact: true }),
  ).toBeEnabled();
  expect(f.sent).toEqual([]);
});

test("command history shows five recent jobs and explains a blocked grasp", async ({
  page,
}) => {
  const f = await fixture(page);
  f.status.jobs = Array.from({ length: 8 }, (_, i) => ({
    id: `job-${i}`,
    action: "run",
    actor: `operator-${i}`,
    state: "FAILED",
    message: "선택한 물체를 잡을 팔 자세(IK)를 양팔 모두에서 찾지 못했습니다.",
    result: {
      status: "BLOCKED",
      error_code: "TARGET_UNAVAILABLE",
      failed_substage: "SelectArmAndPath",
    },
  }));
  await f.panel.getByRole("button", { name: "상태 다시 확인" }).click();
  await expect(f.panel.locator("p.operator-message")).toContainText("양팔");
  await f.panel.locator("summary").filter({ hasText: "웹 명령 이력" }).click();
  await expect(f.panel.locator(".operator-jobs > li")).toHaveCount(5);
  await expect(f.panel.locator(".operator-jobs > li").first()).toContainText(
    "operator-0",
  );
  await expect(f.panel.locator(".operator-jobs > li").last()).toContainText(
    "operator-4",
  );
  await expect(f.panel.locator(".operator-jobs")).not.toContainText(
    "operator-5",
  );
  if (process.env.OPERATOR_VISUAL_DIR) {
    await f.panel.screenshot({
      path: process.env.OPERATOR_VISUAL_DIR + "/history.png",
    });
    await page.getByRole("button", { name: "다크 모드로 전환" }).click();
    await f.panel.screenshot({
      path: process.env.OPERATOR_VISUAL_DIR + "/dark.png",
    });
  }
  expect(f.sent).toEqual([]);
});
