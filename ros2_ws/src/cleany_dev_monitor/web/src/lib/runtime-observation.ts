import { type Data, type Row } from "./monitor";

// snapshot과 이벤트는 서로 다른 ROS 토픽이므로 도착 순서가 Runtime sequence와
// 다를 수 있습니다. snapshot에는 이미 반영한 event_sequence가 들어 있습니다.
export function applyRuntimeEvents(rows: Row[]): Row[] {
  const base = rows.find((row) => row.kind === "runtime");
  if (!base) return rows;
  const data = base.data;
  const hasSequence = Number.isFinite(data.event_sequence);
  let sequence = data.event_sequence;
  // debug snapshot은 Runtime tick 하나가 끝난 뒤 발행합니다. root bt_tick 이벤트가
  // 다음 tick의 경계이므로 visited를 tick 사이에 누적하지 않습니다.
  let nextTick = true;
  const events = rows
    .filter(
      (row) =>
        row.kind === "event" &&
        row.data.boot_id === data.boot_id &&
        (hasSequence && Number.isFinite(row.data.sequence)
          ? row.data.sequence > data.event_sequence
          : row.seq > base.seq),
    )
    .sort((a, b) =>
      Number.isFinite(a.data.sequence) && Number.isFinite(b.data.sequence)
        ? a.data.sequence - b.data.sequence
        : a.seq - b.seq,
    );
  if (!events.length) return rows;
  // 아래에서 제자리 수정하는 것은 트리뿐입니다. 다른 snapshot 필드는 overlay에서
  // 교체하므로 무관한 telemetry까지 패킷마다 깊은 복사할 필요가 없습니다.
  const overlay: Data = {
    ...data,
    bt_nodes: data.bt_nodes?.map((node: Data) => ({ ...node })),
  };
  for (const row of events) {
    const event = row.data;
    if (hasSequence && Number.isFinite(event.sequence)) {
      if (event.sequence <= sequence) continue;
      sequence = event.sequence;
    }
    if (event.kind === "transition") {
      overlay.state = event.data.to;
      overlay.active_request = event.data.active_request;
      overlay.ready = false;
      overlay.reason = "전이 후 준비 상태 확인 중";
      if (event.data.to === "NAVIGATE_TO_TARGET") {
        // Runtime은 미션 수락 시 이전 청소 트리를 버립니다.
        overlay.bt_nodes = [];
        overlay.bt_stage = "";
        overlay.manipulation_feedback = {};
      }
    }
    if (event.kind === "bt_tick") {
      const nodes: Data[] = overlay.bt_nodes || [];
      const node = nodes.find(
        (candidate) => candidate.id === event.data.node_id,
      );
      if (!node) continue;
      if (nextTick) nodes.forEach((candidate) => (candidate.visited = false));
      node.status = event.data.status;
      node.visited = true;
      // leaf만 CleaningContext.stage를 갱신하고 composite는 tick을 마칩니다.
      if (!nodes.some((candidate) => candidate.parent === node.id))
        overlay.bt_stage = node.id;
      nextTick = !node.parent;
    }
    if (event.kind === "mission_result") overlay.last_result = event.data;
  }
  // 이벤트 overlay는 새 준비 상태 관측이 아니므로 원래 수신 시각을 유지합니다.
  // 오래된 안전·모듈 보고가 최신인 것처럼 보이게 해서는 안 됩니다.
  return rows.map((row) => (row === base ? { ...base, data: overlay } : row));
}
