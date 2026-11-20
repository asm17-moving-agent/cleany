import "../hold_input.cjs";

export function mountRemote({ getSpeed, onSpeedDisabled, onMaxSpeed, onForwardOnly }) {
  const { HoldInput, HeldUpdates, stickAxes } = window.CleanyInput;
  const listeners = new AbortController();
  const listen = (target, event, handler, options = {}) =>
    target.addEventListener(event, handler, {
      ...(options === true ? { capture: true } : options),
      signal: listeners.signal,
    });
  let disposed = false;
  let forwardOnly = false;
  const token = new URLSearchParams(location.hash.slice(1)).get("token") || "";
  const client = Array.from(crypto.getRandomValues(new Uint8Array(16)), (x) =>
    x.toString(16).padStart(2, "0"),
  ).join("");
  const sticks = [
      document.getElementById("moveStick"),
      document.getElementById("turnStick"),
    ],
    message = document.getElementById("message");
  let sequence = 0,
    heartbeat = null,
    updates = null,
    pollPending = false,
    lastStopLatency = null,
    lastSignalAt = 0,
    ready = false,
    generation = 0,
    pressPending = false;
  let stopReason = "입력 해제",
    retryAt = 0;
  function send(action, extra = {}, keepalive = false) {
    const requestSequence = ++sequence,
      current = generation;
    const perform = async () => {
      if (action !== "stop" && current !== generation)
        throw new Error("종료된 조종 요청");
      const controller = new AbortController(),
        timeout = setTimeout(() => controller.abort(), 180);
      try {
        const response = await fetch("/api/control", {
          method: "POST",
          headers: {
            "Content-Type": "application/json",
            "X-Control-Token": token,
          },
          body: JSON.stringify({
            ui_version: 2,
            client,
            sequence: requestSequence,
            action,
            ...extra,
          }),
          signal: controller.signal,
          keepalive,
        });
        const data = await response.json();
        if (!response.ok) {
          const error = new Error(data.error || "조종 요청 실패");
          error.staleRequest = !!data.stale_request;
          throw error;
        }
        return data;
      } catch (error) {
        if (error.name === "AbortError")
          throw new Error("웹 명령 응답 지연(180ms)");
        throw error;
      } finally {
        clearTimeout(timeout);
      }
    };
    // Request sequences reject reordered traffic. STOP is never queued behind updates.
    return perform();
  }
  const pointers = new Map(),
    axes = new Map(),
    heldKeys = new Set();
  function resetSticks() {
    for (const stick of sticks) {
      stick.querySelector(".knob").style.transform = "translate(-50%,-50%)";
      stick.dataset.active = "false";
    }
  }
  function showHeld() {
    if (disposed) return;
    const held = pointers.size > 0 || heldKeys.size > 0;
    const age = lastSignalAt
      ? Math.round(performance.now() - lastSignalAt)
      : null;
    document.getElementById("inputState").textContent =
      "브라우저 입력 · " +
      (held
        ? input.suspended
          ? "누름 유지 · 연결 회복 시 자동 재개"
          : "누름 유지"
        : "해제") +
      (input.active && !input.suspended && age !== null
        ? " · 서버 응답 " + age + " ms 전"
        : !input.active && lastStopLatency !== null
          ? " · HTTP 응답 " + lastStopLatency + " ms"
          : "");
    onSpeedDisabled(!!input.active);
    sticks.forEach((s) => {
      s.setAttribute("aria-disabled", "false");
      s.style.opacity = !ready && !input.active ? ".4" : "1";
    });
  }
  function failed(error, current) {
    if (current === generation) {
      stopReason = "웹 명령 응답 오류 · " + error.message;
      input.suspend();
      message.textContent = error.message;
    }
  }
  function intent() {
    const move = axes.get(sticks[0]),
      turn = axes.get(sticks[1]);
    return {
      vector: [move?.forward || 0, forwardOnly ? 0 : move?.left || 0, turn?.ccw || 0],
      speed: getSpeed(),
    };
  }
  const input = new HoldInput({
    start: async (value) => {
      const current = ++generation;
      pressPending = true;
      stopReason = "입력 해제";
      showHeld();
      if (!ready) {
        pressPending = false;
        retryAt = performance.now() + 250;
        input.suspend(false);
        showHeld();
        return;
      }
      try {
        await send("press", value);
        lastSignalAt = performance.now();
        if (current !== generation) return;
        // Renew immediately after press, then every 70 ms without waiting for replies.
        updates = new HeldUpdates({
          send: () =>
            input.active ? send("update", input.active) : Promise.resolve(null),
          onReply: () => {
            if (current === generation) {
              lastSignalAt = performance.now();
              showHeld();
            }
          },
          onError: (error) => failed(error, current),
        });
        updates.tick();
        heartbeat = setInterval(() => updates?.tick(), 70);
      } catch (error) {
        failed(error, current);
      } finally {
        if (current === generation) pressPending = false;
      }
    },
    change: () => showHeld(),
    stop: (preserveHold = false) => {
      generation++;
      pressPending = false;
      clearInterval(heartbeat);
      heartbeat = null;
      updates?.cancel();
      updates = null;
      if (disposed) {
        send("stop", { reason: "조종 화면 종료" }, true).catch(() => {});
        return;
      }
      if (preserveHold) retryAt = performance.now() + 250;
      else {
        retryAt = 0;
        pointers.clear();
        axes.clear();
        heldKeys.clear();
        resetSticks();
      }
      showHeld();
      const stopAt = performance.now(),
        stopGeneration = generation;
      lastStopLatency = null;
      document.getElementById("stopState").textContent =
        "ESP 출력 정지 · STOP 확인 대기";
      document.getElementById("connection").textContent = "정지 요청 중";
      message.textContent = "STOP 응답을 기다립니다";
      const reason = stopReason;
      document.getElementById("lastStop").textContent =
        "최근 정지 사유 · " + reason;
      send("stop", { reason }, true)
        .then((status) => {
          if (stopGeneration === generation) {
            lastStopLatency = Math.round(performance.now() - stopAt);
            document.getElementById("connection").textContent =
              status.stop_confirmed ? "ESP32 연결됨" : "정지 확인 중";
            message.textContent = status.message;
            document.getElementById("target").textContent = status.target
              .map((v) => v.toFixed(2))
              .join(" / ");
            showHeld();
          }
        })
        .catch(() => {
          if (stopGeneration === generation)
            message.textContent =
              "정지 응답을 받지 못했습니다. 연결을 확인해주세요.";
        });
    },
  });
  function point(stick, event) {
    const rect = stick.getBoundingClientRect(),
      rotation = stick === sticks[1];
    const radius = rotation
      ? Math.max(
          1,
          (rect.width - stick.querySelector(".knob").offsetWidth) / 2 - 8,
        )
      : rect.width * 0.31;
    const a = stickAxes(
      event.clientX - rect.left - rect.width / 2,
      event.clientY - rect.top - rect.height / 2,
      radius,
      rotation,
    );
    if (forwardOnly && !rotation) {
      a.x = 0;
      a.left = 0;
      a.ccw = 0;
    }
    axes.set(stick, a);
    stick.dataset.active = "true";
    stick.querySelector(".knob").style.transform =
      `translate(calc(-50% + ${a.x * radius}px),calc(-50% + ${a.y * radius}px))`;
    input.press(intent());
    showHeld();
  }
  function releasePointer(stick, event) {
    if (pointers.get(stick) !== event.pointerId) return;
    pointers.delete(stick);
    axes.delete(stick);
    stick.querySelector(".knob").style.transform = "translate(-50%,-50%)";
    stick.dataset.active = "false";
    if (pointers.size || heldKeys.size) input.press(intent());
    else input.release();
    showHeld();
  }
  for (const stick of sticks) {
    listen(stick, "pointerdown", (event) => {
      event.preventDefault();
      if (pointers.has(stick) || heldKeys.size || event.button !== 0) return;
      pointers.set(stick, event.pointerId);
      stick.setPointerCapture(event.pointerId);
      point(stick, event);
    });
    listen(stick, "pointermove", (event) => {
      if (pointers.get(stick) !== event.pointerId) return;
      if (event.pointerType === "mouse" && event.buttons === 0) input.cancel();
      else point(stick, event);
    });
    listen(stick, "pointerup", (event) => releasePointer(stick, event));
    listen(stick, "lostpointercapture", (event) => {
      if (pointers.get(stick) === event.pointerId) {
        stopReason = "포인터 캡처 해제";
        releasePointer(stick, event);
      }
    });
    listen(stick, "pointercancel", () => {
      stopReason = "터치 취소";
      input.cancel();
    });
    listen(stick, "contextmenu", (event) => event.preventDefault());
  }
  // Catch release outside the stick even if pointer capture is lost.
  listen(
    window,
    "pointerup",
    (event) => {
      for (const stick of sticks) releasePointer(stick, event);
    },
    true,
  );
  listen(document.getElementById("stop"), "pointerdown", (event) => {
    event.preventDefault();
    stopReason = "STOP 버튼";
    input.cancel(true);
  });
  listen(document.getElementById("stop"), "click", () => {
    stopReason = "STOP 버튼";
    input.cancel(true);
  });
  listen(window, "blur", () => {
    stopReason = "화면 초점 이탈";
    input.cancel();
  });
  listen(window, "pagehide", () => {
    stopReason = "페이지 이탈";
    input.cancel();
  });
  listen(document, "visibilitychange", () => {
    if (document.hidden) {
      stopReason = "화면 숨김";
      input.cancel();
    }
  });
  const keys = {
    ArrowUp: [1, 0, 0],
    ArrowDown: [-1, 0, 0],
    ArrowLeft: [0, 1, 0],
    ArrowRight: [0, -1, 0],
    a: [0, 0, 1],
    d: [0, 0, -1],
  };
  function keyIntent() {
    const vector = [0, 0, 0];
    for (const key of heldKeys) keys[key].forEach((v, i) => (vector[i] += v));
    return {
      vector: vector.map((v, i) => forwardOnly && i === 1 ? 0 : Math.max(-1, Math.min(1, v))),
      speed: getSpeed(),
    };
  }
  listen(document, "keydown", (event) => {
    if (event.code === "Space" || event.key === "Escape") {
      event.preventDefault();
      input.cancel(true);
    } else if (
      keys[event.key] &&
      (!forwardOnly || event.key === "ArrowUp" || event.key === "ArrowDown" || event.key === "a" || event.key === "d") &&
      !event.target.closest?.('[data-slot="slider"]') &&
      !pointers.size
    ) {
      event.preventDefault();
      if (event.repeat && !heldKeys.has(event.key)) return;
      heldKeys.add(event.key);
      input.press(keyIntent());
      showHeld();
    }
  });
  listen(document, "keyup", (event) => {
    if (!heldKeys.delete(event.key)) return;
    if (heldKeys.size) input.press(keyIntent());
    else input.release();
    showHeld();
  });
  async function poll() {
    if (disposed || pollPending) return;
    pollPending = true;
    const current = generation,
      wasPending = pressPending;
    try {
      const response = await fetch("/api/status", {
        headers: { "X-Control-Token": token },
        signal: AbortSignal.timeout(700),
      });
      const status = await response.json();
      if (!response.ok) throw new Error(status.error);
      if (current !== generation) return;
      ready = status.ready;
      forwardOnly = status.forward_only === true;
      onForwardOnly?.(forwardOnly);
      if (
        input.active &&
        !input.suspended &&
        (!status.connected ||
          (!wasPending &&
            !pressPending &&
            (status.phase === "idle" || status.phase === "stopping")))
      ) {
        stopReason = status.connected ? status.message : "ESP32 상태 수신 지연";
        input.suspend();
      }
      document.getElementById("mock").hidden = !status.mock;
      document
        .getElementById("dot")
        .classList.toggle("online", status.connected);
      document.getElementById("connection").textContent = status.connected
        ? status.phase === "moving"
          ? "조종 중"
          : "ESP32 연결됨"
        : "ESP32 상태 수신 지연";
      message.textContent = status.message;
      document.getElementById("stopState").textContent = status.stop_confirmed
        ? "ESP 출력 0 확인" +
          (status.stop_ack_ms !== null
            ? " · 서버 STOP → ESP 확인 " + status.stop_ack_ms + " ms"
            : "")
        : status.phase === "stopping"
          ? "ESP 출력 정지 · STOP 확인 대기"
          : status.phase === "moving"
            ? "ESP 출력 활성"
            : "ESP 출력 정지 · 미확인";
      document.getElementById("velocity").textContent = status.velocity
        .map((v) => v.toFixed(2))
        .join(" / ");
      document.getElementById("target").textContent = status.target
        .map((v) => v.toFixed(2))
        .join(" / ");
      document.getElementById("pwm").textContent = status.pwm.join(" / ");
      onMaxSpeed(status.web_max_velocity);
      document.getElementById("mcuLimit").textContent =
        status.mcu_max_velocity === null
          ? "—"
          : status.mcu_max_velocity.toFixed(1) + " rad/s";
      document.getElementById("feedbackAge").textContent =
        status.feedback_age_ms === null
          ? "ESP 상태 수신 대기"
          : "ESP 상태 " +
            status.feedback_age_ms.toFixed(0) +
            " ms 전 · boot " +
            status.boot_id;
      if (status.last_stop_reason)
        document.getElementById("lastStop").textContent =
          "최근 정지 사유 · " + status.last_stop_reason;
      const fault = document.getElementById("lastFault");
      const permissionError = status.last_permission_error?.reason;
      fault.hidden = !status.last_fault_reason && !permissionError;
      if (!fault.hidden)
        fault.textContent = [
          status.last_fault_reason && ("최근 통신·제어 정지 · " + status.last_fault_reason),
          permissionError && ("권한 요청 · " + permissionError),
        ].filter(Boolean).join(" / ");
      showHeld();
      if (
        input.suspended &&
        ready &&
        performance.now() >= retryAt &&
        !document.hidden &&
        (pointers.size || heldKeys.size)
      )
        input.resume();
    } catch (error) {
      if (current !== generation) return;
      ready = false;
      stopReason = "웹 상태 조회 오류 · " + error.message;
      input.suspend();
      showHeld();
      document.getElementById("dot").classList.remove("online");
      document.getElementById("connection").textContent = "조종 연결 확인 필요";
      document.getElementById("feedbackAge").textContent =
        "ESP 상태 수신 중단 · 연결 회복 대기";
      message.textContent = error.message;
    } finally {
      pollPending = false;
    }
  }
  poll();
  const pollTimer = setInterval(poll, 100);
  return () => {
    disposed = true;
    input.cancel();
    generation++;
    clearInterval(pollTimer);
    clearInterval(heartbeat);
    updates?.cancel();
    listeners.abort();
  };
}
