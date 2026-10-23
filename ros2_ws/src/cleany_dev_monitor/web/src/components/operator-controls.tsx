import { useEffect, useRef, useState, type ReactNode } from "react";
import {
  Activity,
  ArrowRight,
  Ban,
  ChevronDown,
  CircleAlert,
  Hand,
  History,
  KeyRound,
  LoaderCircle,
  LockKeyhole,
  LogOut,
  Pause,
  Play,
  PlugZap,
  Power,
  RefreshCw,
  RotateCcw,
  ScanLine,
  ShieldCheck,
  Square,
  Workflow,
  type LucideIcon,
} from "lucide-react";
import { Button } from "@/components/ui/button";
import { type Data } from "@/lib/monitor";
import { TaskProposal } from "./task-proposal";
import "./operator-controls.css";

const labels: Record<string, string> = {
  prepare: "로봇 준비",
  inspect: "장면 인식만",
  run: "물체 1개 실행",
  propose: "다음 작업 제안받기",
  approve: "제안 승인 실행",
  skip: "물체 건너뛰고 재제안",
  return_arm: "팔 기본 자세 복귀",
  recover: "실행 차단 해제",
  shutdown: "로봇 종료·토크 해제",
};
const jobStates: Record<string, string> = {
  PENDING: "접수됨",
  RUNNING: "명령 처리 중",
  SUCCEEDED: "명령 처리 완료",
  FAILED: "명령 실패",
  INTERRUPTED: "명령 중단됨",
  UNKNOWN: "결과 확인 필요",
  ACKNOWLEDGED: "운영자 기록 확인",
};
const proposalPhases: Record<string, string> = {
  PROPOSING: "현재 장면을 관찰하고 다음 작업을 제안하고 있습니다.",
  REVALIDATING: "승인한 대상의 동일성과 현재 위치를 다시 확인하고 있습니다.",
  EXECUTING: "승인한 물체의 동작을 실행하고 있습니다.",
  PROPOSING_NEXT: "실행 결과를 반영해 다음 제안을 준비하고 있습니다.",
};

const actionIcons: Record<string, LucideIcon> = {
  prepare: Power,
  inspect: ScanLine,
  run: Play,
  propose: ScanLine,
  approve: Play,
  skip: RefreshCw,
  return_arm: RotateCcw,
  recover: ShieldCheck,
  shutdown: Power,
};
const mainActions = [
  { id: "prepare", tag: "준비", detail: "센서와 실행기를 준비합니다" },
  { id: "inspect", tag: "선택 사항", detail: "주변 물체를 먼저 확인합니다" },
  {
    id: "propose",
    tag: "승인 방식",
    detail: "추천 대상을 확인한 뒤 승인합니다",
  },
  {
    id: "run",
    tag: "직접 실행",
    detail: "제안 검토 없이 물체 하나를 자동 선택해 실행합니다",
  },
];

function jobTone(job: Data) {
  if (job.state === "SUCCEEDED" || job.state === "ACKNOWLEDGED") return "good";
  if (job.result?.status === "BLOCKED" || job.state === "UNKNOWN")
    return "warn";
  if (job.state === "FAILED") return "bad";
  if (job.state === "RUNNING" || job.state === "PENDING") return "info";
  return "muted";
}

function requestId() {
  const bytes = crypto.getRandomValues(new Uint8Array(16));
  bytes[6] = (bytes[6] & 15) | 64;
  bytes[8] = (bytes[8] & 63) | 128;
  const hex = Array.from(bytes, (n) => n.toString(16).padStart(2, "0")).join(
    "",
  );
  return `${hex.slice(0, 8)}-${hex.slice(8, 12)}-${hex.slice(12, 16)}-${hex.slice(16, 20)}-${hex.slice(20)}`;
}

async function api(path: string, data?: Data) {
  const response = await fetch(
    "/api/control/" + path,
    data
      ? {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify(data),
          signal: AbortSignal.timeout(10000),
        }
      : { signal: AbortSignal.timeout(10000) },
  );
  if (!response.ok) {
    const error = new Error((await response.text()).slice(0, 500));
    Object.assign(error, {
      rejected: response.status >= 400 && response.status < 500,
    });
    throw error;
  }
  return response.json();
}

export function OperatorControls({
  enabled,
  connected,
  replay,
  summary,
  children,
}: {
  enabled: boolean;
  connected: boolean;
  replay: boolean;
  summary: ReactNode;
  children: ReactNode;
}) {
  const [status, setStatus] = useState<Data>({});
  const [error, setError] = useState("");
  const [name, setName] = useState("");
  const [key, setKey] = useState("");
  const [takeover, setTakeover] = useState(false);
  const [sending, setSending] = useState(false);
  const [modal, setModal] = useState("");
  const [arm, setArm] = useState("both");
  const [confirmed, setConfirmed] = useState(false);
  const [empty, setEmpty] = useState(false);
  const [note, setNote] = useState("");
  const [blocked, setBlocked] = useState("");
  const [reviewedBlocks, setReviewedBlocks] = useState<Data[]>([]);
  const pending = useRef<Data | null>(null);
  const [uncertain, setUncertain] = useState(false);
  const [lastPoll, setLastPoll] = useState(0);
  const sequence = useRef(0);
  const polling = useRef<Promise<void> | null>(null);
  const enabledRef = useRef(enabled && connected && !replay);
  enabledRef.current = enabled && connected && !replay;

  function refresh(): Promise<void> {
    // A slow response must finish before the next poll; otherwise every response
    // is discarded by a newer request and the controls never become ready.
    if (polling.current) return polling.current;
    const seq = ++sequence.current;
    const task = poll(seq);
    polling.current = task;
    void task.finally(() => {
      if (polling.current === task) polling.current = null;
    });
    return task;
  }

  async function poll(seq: number) {
    try {
      const value = await api("status");
      if (seq !== sequence.current) return;
      setStatus(value);
      setLastPoll(Date.now());
      if (
        pending.current &&
        value.jobs?.some((job: Data) => job.id === pending.current!.request_id)
      ) {
        pending.current = null;
        sessionStorage.removeItem("cleany.operator.pending");
        setUncertain(false);
      }
    } catch (e) {
      if (seq === sequence.current) {
        setLastPoll(0);
        setError(String(e));
      }
    }
  }

  useEffect(() => {
    if (!enabled || replay) return;
    try {
      pending.current = JSON.parse(
        sessionStorage.getItem("cleany.operator.pending") || "null",
      );
      setUncertain(!!pending.current);
    } catch {
      pending.current = null;
    }
    void refresh();
    const timer = setInterval(() => void refresh(), 2000);
    return () => {
      clearInterval(timer);
      sequence.current++;
    };
  }, [enabled, replay]);

  if (!enabled)
    return (
      <>
        <section aria-label="실물 조작 제어" className="operator-disabled">
          <LockKeyhole size={20} aria-hidden="true" />
          <div>
            <h2>
              실물 조작 제어 <span>웹 제어 꺼짐</span>
            </h2>
            <p>
              현재 관측 전용입니다. 조작 기능을 켜면 이곳에 제어 패널이
              표시됩니다.
            </p>
          </div>
        </section>
        <div className="manip-summary-slot">{summary}</div>
        <div className="manip-monitor-column">{children}</div>
      </>
    );
  const fresh =
    connected && !replay && lastPoll > 0 && Date.now() - lastPoll < 5000;
  const authenticated = status.authenticated === true;
  const state = status.state || {};
  const busy = !!status.active_job;
  const reason = (action: string) =>
    !fresh
      ? "라이브 연결과 최신 상태가 필요합니다."
      : !authenticated
        ? "조작 권한 인증이 필요합니다."
        : sending
          ? "요청 전송 중입니다."
          : uncertain
            ? "응답을 확인하지 못한 요청을 먼저 조회하세요."
            : status.capabilities?.[action] || "";
  const sharedActionReason = mainActions.every(
    ({ id }) => reason(id) === reason(mainActions[0].id),
  )
    ? reason(mainActions[0].id)
    : "";
  const canCancel =
    fresh &&
    authenticated &&
    !sending &&
    (busy ||
      state.active_execution_id ||
      state.arm_return_active ||
      status.runtime?.pipeline_busy);

  async function login(event: React.FormEvent) {
    event.preventDefault();
    setSending(true);
    setError("");
    try {
      await api("login", { name, key, takeover });
      setKey("");
      await refresh();
    } catch (e) {
      setError(String(e));
      setKey("");
    } finally {
      setSending(false);
    }
  }

  async function submit(action: string, args: Data = {}, reuse = false) {
    if (!enabledRef.current) return;
    const request =
      reuse && pending.current
        ? pending.current
        : { request_id: requestId(), action, args };
    pending.current = request;
    sessionStorage.setItem("cleany.operator.pending", JSON.stringify(request));
    setSending(true);
    setError("");
    try {
      await api("jobs", request);
      pending.current = null;
      sessionStorage.removeItem("cleany.operator.pending");
      setUncertain(false);
      setModal("");
      await refresh();
    } catch (e) {
      setError(String(e));
      const rejected =
        e instanceof Error && "rejected" in e && e.rejected === true;
      if (rejected) {
        pending.current = null;
        sessionStorage.removeItem("cleany.operator.pending");
      }
      setUncertain(!rejected);
      await refresh();
    } finally {
      setSending(false);
    }
  }

  async function cancel(mode: string) {
    if (!enabledRef.current) return;
    setSending(true);
    setError("");
    try {
      const response = await api("cancel", { mode });
      setError(
        response.accepted
          ? "중단을 요청했습니다. 실물 정지 확인을 기다리고 있습니다."
          : response.message,
      );
      await refresh();
    } catch (e) {
      setError(String(e));
    } finally {
      setSending(false);
    }
  }

  function open(action: string) {
    setModal(action);
    setConfirmed(false);
    setEmpty(false);
    setNote("");
    setBlocked("");
    setReviewedBlocks(structuredClone(state.blocked || []));
  }
  function confirm() {
    if (modal === "prepare")
      void submit(modal, { confirm_torque_on: confirmed });
    if (modal === "return_arm")
      void submit(modal, { arm, confirm_open: confirmed });
    if (modal === "shutdown")
      void submit(modal, { confirm_torque_off: confirmed });
    if (modal === "recover") {
      const record = reviewedBlocks.find(
        (item: Data) =>
          item.execution_id === (blocked || reviewedBlocks[0]?.execution_id),
      );
      if (record)
        void submit(modal, {
          execution_id: record.execution_id,
          revision: record.revision,
          stopped: confirmed,
          empty,
          note,
        });
    }
    if (modal.startsWith("ack:")) {
      setSending(true);
      void api(`jobs/${modal.slice(4)}/acknowledge`, { confirmed, note })
        .then(() => {
          setModal("");
          return refresh();
        })
        .catch((e) => setError(String(e)))
        .finally(() => setSending(false));
    }
  }

  const guardReady =
    fresh && state.guard_fresh && state.guard?.state === "ready";
  const guardLabel =
    !fresh || !state.guard_fresh
      ? "미확인"
      : (
          { ready: "준비됨", stopped: "정지됨", fault: "오류" } as Record<
            string,
            string
          >
        )[state.guard?.state] ||
        state.guard?.state ||
        "미확인";
  const statusCards = [
    {
      label: "실행기 연결",
      icon: PlugZap,
      value: fresh && state.available ? "연결됨" : "확인 필요",
      tone: fresh && state.available ? "good" : "muted",
    },
    {
      label: "제어기 상태",
      icon: Activity,
      value: guardLabel,
      tone: guardReady ? "good" : fresh && state.guard_fresh ? "warn" : "muted",
    },
    {
      label: "현재 작업",
      icon: Hand,
      value: !fresh
        ? "미확인"
        : state.active_execution_id
          ? "실물 조작 중"
          : state.arm_return_active
            ? "팔 복귀 중"
            : busy
              ? "명령 처리 중"
              : "대기 중",
      tone:
        fresh && (busy || state.active_execution_id || state.arm_return_active)
          ? "info"
          : "muted",
    },
    {
      label: "실행 차단",
      icon: Ban,
      value:
        !fresh || !state.available
          ? "미확인"
          : state.inhibited
            ? "확인 필요"
            : "없음",
      tone:
        fresh && state.available
          ? state.inhibited
            ? "warn"
            : "good"
          : "muted",
    },
  ];
  const ModalIcon = actionIcons[modal] || ShieldCheck;
  const attention = state.message || status.capabilities?.run;

  return (
    <section aria-label="실물 조작 제어" className="operator-region">
      <div className="operator-console">
        <div className="operator-overview">
          <header className="operator-header">
            <div className="operator-heading">
              <span className="operator-brand-icon">
                <Hand aria-hidden="true" />
              </span>
              <div>
                <h2>실물 조작 제어</h2>
              </div>
            </div>
            <div className="operator-toolbar">
              <span className="operator-access" data-active={authenticated}>
                {authenticated ? (
                  <ShieldCheck aria-hidden="true" />
                ) : (
                  <LockKeyhole aria-hidden="true" />
                )}
                <span>
                  {authenticated ? `조작자: ${status.actor}` : "보기 전용"}
                </span>
              </span>
              <Button
                variant="ghost"
                size="icon"
                disabled={sending}
                aria-label="상태 다시 확인"
                title="상태 다시 확인"
                onClick={() => void refresh()}
              >
                <RefreshCw aria-hidden="true" />
              </Button>
              {authenticated && (
                <Button
                  variant="ghost"
                  size="icon"
                  aria-label="조작 권한 해제"
                  title="조작 권한 해제"
                  onClick={() =>
                    void api("logout", {})
                      .then(refresh)
                      .catch((e) => setError(String(e)))
                  }
                >
                  <LogOut aria-hidden="true" />
                </Button>
              )}
            </div>
          </header>
          <div className="operator-status" aria-label="제어 준비 상태">
            {statusCards.map(({ label, icon: Icon, value, tone }) => (
              <div className="operator-stat" key={label} data-tone={tone}>
                <div>
                  <span>{label}</span>
                  <Icon aria-hidden="true" />
                </div>
                <strong>
                  <i className="operator-status-dot" />
                  {value}
                </strong>
              </div>
            ))}
          </div>
          <div className="operator-mode">
            <Workflow aria-hidden="true" />
            <span>현재 동작</span>
            <strong>
              {state.operation_mode === "pick_return_release"
                ? "집기 → 들기 → 원위치 놓기"
                : state.operation_mode || "확인 필요"}
            </strong>
            {state.operation_mode === "pick_return_release" && (
              <small>안착은 가정</small>
            )}
          </div>
        </div>
        <div className="operator-layout">
          <div className="manip-summary-slot">{summary}</div>
          <aside className="operator-control-column" aria-label="조작 명령">
            <div className="operator-control-panel">
              {!replay && !authenticated && (
                <details className="operator-auth">
                  <summary>
                    <KeyRound aria-hidden="true" />
                    <span>조작 권한 연결</span>
                    <small>인증 필요</small>
                    <ChevronDown aria-hidden="true" />
                  </summary>
                  <p className="operator-auth-caption">
                    이름과 조작키로 인증한 후 실행할 수 있습니다.
                  </p>
                  <form onSubmit={login} className="operator-login">
                    <div className="operator-login-fields">
                      <label>
                        조작자 이름
                        <input
                          value={name}
                          onChange={(e) => setName(e.target.value)}
                          maxLength={40}
                          autoComplete="name"
                          placeholder="이름 입력"
                          required
                        />
                      </label>
                      <label>
                        조작키
                        <input
                          type="password"
                          value={key}
                          onChange={(e) => setKey(e.target.value)}
                          autoComplete="off"
                          placeholder="Jetson에서 확인한 조작키"
                          required
                        />
                      </label>
                      <Button disabled={sending || !fresh} type="submit">
                        조작 권한 연결
                        <ArrowRight aria-hidden="true" />
                      </Button>
                    </div>
                    <div className="operator-login-footer">
                      <label className="operator-check operator-takeover">
                        <input
                          type="checkbox"
                          checked={takeover}
                          onChange={(e) => setTakeover(e.target.checked)}
                        />
                        다른 브라우저의 조작 권한 가져오기
                      </label>
                      <details className="operator-key-help">
                        <summary>조작키 확인 방법</summary>
                        <small>
                          Jetson 터미널에서{" "}
                          <code>make dev-monitor-control-key</code>를
                          실행하세요.
                        </small>
                      </details>
                    </div>
                  </form>
                </details>
              )}
              <div className="operator-stop">
                <div className="operator-section-label">
                  <h3>
                    <Square aria-hidden="true" />
                    동작 중단
                  </h3>
                  <span>진행 중인 작업에 요청</span>
                </div>
                <div className="operator-stop-buttons">
                  <Button
                    variant="destructive"
                    disabled={!canCancel}
                    onClick={() => void cancel("IMMEDIATE")}
                  >
                    <Square aria-hidden="true" />
                    즉시 중단 요청
                  </Button>
                  <Button
                    variant="outline"
                    disabled={!canCancel || !state.active_execution_id}
                    onClick={() => void cancel("CHECKPOINT")}
                  >
                    <Pause aria-hidden="true" />
                    현재 단계 후 중단
                  </Button>
                </div>
                <small>실물 정지 여부는 실행 기록에서 확인하세요.</small>
              </div>
              <div className="operator-workspace">
                <div className="operator-section-label">
                  <h3>작업 실행</h3>
                  <span>준비 → 관측 → 실행</span>
                </div>
                {authenticated && sharedActionReason && (
                  <p
                    className="operator-action-reason"
                    id="operator-common-reason"
                  >
                    {sharedActionReason}
                  </p>
                )}
                <div className="operator-actions">
                  {mainActions.map(({ id, detail, tag }) => {
                    const Icon = actionIcons[id];
                    return (
                      <span
                        key={id}
                        title={reason(id)}
                        className="operator-action-wrap"
                      >
                        <Button
                          className="operator-action"
                          data-primary={id === "propose"}
                          variant="outline"
                          aria-label={labels[id]}
                          disabled={!!reason(id)}
                          aria-describedby={
                            authenticated && reason(id)
                              ? sharedActionReason
                                ? "operator-common-reason"
                                : `operator-reason-${id}`
                              : undefined
                          }
                          onClick={() =>
                            id === "prepare" ? open(id) : void submit(id)
                          }
                        >
                          <span className="operator-action-top">
                            <span className="operator-action-icon">
                              <Icon aria-hidden="true" />
                            </span>
                            <span className="operator-step">{tag}</span>
                          </span>
                          <span className="operator-action-title">
                            {labels[id]}
                            <ArrowRight aria-hidden="true" />
                          </span>
                          <span className="operator-action-description">
                            {detail}
                          </span>
                        </Button>
                        {authenticated && !sharedActionReason && reason(id) && (
                          <small
                            id={`operator-reason-${id}`}
                            className="operator-action-reason"
                          >
                            {reason(id)}
                          </small>
                        )}
                      </span>
                    );
                  })}
                </div>
              </div>
              <details className="operator-recovery">
                <summary>
                  <ShieldCheck aria-hidden="true" />
                  <span>복구 및 종료</span>
                  <ChevronDown aria-hidden="true" />
                </summary>
                <div className="operator-recovery-actions">
                  {["return_arm", "recover", "shutdown"].map((action) => {
                    const Icon = actionIcons[action];
                    return (
                      <span key={action} title={reason(action)}>
                        <Button
                          variant="outline"
                          className="operator-recovery-button"
                          aria-label={labels[action]}
                          disabled={!!reason(action)}
                          onClick={() => open(action)}
                        >
                          <Icon aria-hidden="true" />
                          <span>{labels[action]}</span>
                          <ArrowRight aria-hidden="true" />
                        </Button>
                      </span>
                    );
                  })}
                </div>
              </details>
              {!!status.jobs?.length && (
                <details className="operator-history">
                  <summary>
                    <History aria-hidden="true" />
                    <span>웹 명령 이력</span>
                    <small>최근 {Math.min(5, status.jobs.length)}개</small>
                    <ChevronDown
                      className="operator-history-chevron"
                      aria-hidden="true"
                    />
                  </summary>
                  <ul className="operator-jobs">
                    {status.jobs.slice(0, 5).map((job: Data) => {
                      const Icon = actionIcons[job.action] || Activity;
                      return (
                        <li key={job.id} data-tone={jobTone(job)}>
                          <span className="operator-job-icon">
                            <Icon aria-hidden="true" />
                          </span>
                          <div className="operator-job-content">
                            <div className="operator-job-heading">
                              <strong>{labels[job.action]}</strong>
                              <span className="operator-job-state">
                                {jobStates[job.state]}
                              </span>
                              <span className="operator-job-meta">
                                {job.actor}
                                {Number.isFinite(job.created_at) && (
                                  <time
                                    dateTime={new Date(
                                      job.created_at * 1000,
                                    ).toISOString()}
                                  >
                                    {new Date(
                                      job.created_at * 1000,
                                    ).toLocaleTimeString("ko-KR", {
                                      hour12: false,
                                      hour: "2-digit",
                                      minute: "2-digit",
                                    })}
                                  </time>
                                )}
                              </span>
                            </div>
                            {job.message && <p>{job.message}</p>}
                            {job.next_proposal_error && (
                              <p>
                                다음 제안 준비 중단: {job.next_proposal_error}
                              </p>
                            )}
                            {job.result?.status && (
                              <small className="operator-job-result">
                                실행 결과: {job.result.status} ·{" "}
                                {job.result.error_code}{" "}
                                {job.result.failed_substage}
                              </small>
                            )}
                            {job.result?.objects && (
                              <p>
                                인식 물체:{" "}
                                {job.result.objects
                                  .map(
                                    (item: Data) =>
                                      `${item.object_id}: ${item.label}`,
                                  )
                                  .join(", ") || "없음"}
                              </p>
                            )}
                            {job.state === "UNKNOWN" && (
                              <Button
                                variant="outline"
                                disabled={!authenticated || !fresh}
                                onClick={() => open("ack:" + job.id)}
                              >
                                명령 기록 확인
                              </Button>
                            )}
                          </div>
                        </li>
                      );
                    })}
                  </ul>
                </details>
              )}
            </div>
          </aside>
          <div className="manip-monitor-column">
            {replay && (
              <p className="operator-message" data-tone="warn">
                <History aria-hidden="true" />
                기록 재생 중에는 제어할 수 없습니다.
              </p>
            )}
            {authenticated && attention && attention !== sharedActionReason && (
              <p className="operator-message" data-tone="info">
                <CircleAlert aria-hidden="true" />
                {attention}
              </p>
            )}
            {status.active_job && (
              <p className="operator-message" data-tone="info" role="status">
                <LoaderCircle className="operator-spin" aria-hidden="true" />
                <strong>{labels[status.active_job.action]}</strong>
                {jobStates[status.active_job.state]}
              </p>
            )}
            {!status.active_job &&
              ["run", "approve", "propose", "skip"].includes(
                status.jobs?.[0]?.action,
              ) &&
              status.jobs[0].state === "FAILED" && (
                <p
                  className="operator-message"
                  data-tone={jobTone(status.jobs[0])}
                  role="status"
                >
                  <CircleAlert aria-hidden="true" />
                  <span>
                    <strong>{labels[status.jobs[0].action]}</strong>
                    {status.jobs[0].message || "실행 결과를 확인하세요."}
                  </span>
                </p>
              )}
            {error && (
              <p className="operator-message" data-tone="info" role="status">
                <CircleAlert aria-hidden="true" />
                {error}
              </p>
            )}
            {uncertain && (
              <div className="operator-uncertain">
                <p>
                  <CircleAlert aria-hidden="true" />
                  응답 미확인 요청이 있습니다. 같은 요청 ID로만 다시 전송합니다.
                </p>
                <Button
                  variant="outline"
                  disabled={!fresh || !authenticated || sending}
                  onClick={() => void submit("", {}, true)}
                >
                  같은 요청 다시 보내기
                  <RefreshCw aria-hidden="true" />
                </Button>
              </div>
            )}
            <TaskProposal
              proposal={status.proposal}
              reason={reason}
              submit={submit}
            />
            {status.active_job?.phase && (
              <p role="status">{proposalPhases[status.active_job.phase]}</p>
            )}
            {children}
          </div>
        </div>
        {modal && (
          <div className="operator-modal-backdrop">
            <section
              className="operator-modal"
              role="dialog"
              aria-modal="true"
              aria-label={labels[modal] || "명령 기록 확인"}
            >
              <div className="operator-modal-heading">
                <span className="operator-brand-icon">
                  <ModalIcon aria-hidden="true" />
                </span>
                <div>
                  <span className="operator-eyebrow">실물 조작 확인</span>
                  <h3>{labels[modal] || "명령 기록 확인"}</h3>
                </div>
              </div>
              {modal === "prepare" && (
                <p>
                  모터 토크를 활성화하고 센서·계획 서비스와 실행기를 준비합니다.
                </p>
              )}
              {modal === "return_arm" && (
                <>
                  <p>
                    선택한 그리퍼를 먼저 연 뒤 저장 자세로 복귀하며 닫습니다.
                  </p>
                  <label>
                    복귀할 팔
                    <select
                      value={arm}
                      onChange={(e) => setArm(e.target.value)}
                    >
                      <option value="both">양팔</option>
                      <option value="left">왼팔</option>
                      <option value="right">오른팔</option>
                    </select>
                  </label>
                </>
              )}
              {modal === "shutdown" && (
                <p>
                  현재 런타임을 종료하고 모터 토크를 해제합니다. 팔과 물체를
                  지지한 상태인지 확인하세요.
                </p>
              )}
              {modal === "recover" && (
                <>
                  <p>
                    팔을 움직이지 않고 과거 실행 차단을 해제합니다. 서버가 최신
                    상태를 다시 검증합니다.
                  </p>
                  <label>
                    차단된 실행
                    <select
                      value={blocked || reviewedBlocks[0]?.execution_id || ""}
                      onChange={(e) => setBlocked(e.target.value)}
                    >
                      {reviewedBlocks.map((item: Data) => (
                        <option
                          key={item.execution_id}
                          value={item.execution_id}
                        >
                          {item.execution_id} / revision {item.revision}
                        </option>
                      ))}
                    </select>
                  </label>
                </>
              )}
              {modal.startsWith("ack:") && (
                <p>
                  명령 처리 결과와 실물 상태를 직접 확인했음을 기록합니다.
                  실행기의 오류·차단은 해제하지 않습니다.
                </p>
              )}
              <label className="operator-check">
                <input
                  type="checkbox"
                  checked={confirmed}
                  onChange={(e) => setConfirmed(e.target.checked)}
                />
                {modal === "recover"
                  ? "로봇이 정지했음을 확인했습니다."
                  : modal === "return_arm"
                    ? "그리퍼 열기와 팔 복귀를 확인했습니다."
                    : modal === "shutdown"
                      ? "팔·물체 지지와 토크 해제를 확인했습니다."
                      : modal === "prepare"
                        ? "모터 토크 활성화를 확인했습니다."
                        : "실물 상태와 명령 처리 결과를 확인했습니다."}
              </label>
              {modal === "recover" && (
                <label className="operator-check">
                  <input
                    type="checkbox"
                    checked={empty}
                    onChange={(e) => setEmpty(e.target.checked)}
                  />
                  양쪽 그리퍼가 비었음을 확인했습니다.
                </label>
              )}
              {(modal === "recover" || modal.startsWith("ack:")) && (
                <label>
                  확인 내용
                  <input
                    value={note}
                    onChange={(e) => setNote(e.target.value)}
                    maxLength={500}
                  />
                </label>
              )}
              <div className="operator-buttons operator-modal-footer">
                <Button
                  variant="outline"
                  disabled={sending}
                  onClick={() => setModal("")}
                >
                  닫기
                </Button>
                <Button
                  disabled={
                    sending ||
                    !fresh ||
                    !authenticated ||
                    !confirmed ||
                    (modal === "recover" && !empty) ||
                    ((modal === "recover" || modal.startsWith("ack:")) &&
                      !note.trim())
                  }
                  onClick={confirm}
                >
                  확인 후 요청
                </Button>
              </div>
            </section>
          </div>
        )}
      </div>
    </section>
  );
}
