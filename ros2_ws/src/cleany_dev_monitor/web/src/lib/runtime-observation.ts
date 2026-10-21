import { type Data, type Row } from "./monitor";

// Snapshots and events use separate ROS topics. Their arrival order need not match
// the Runtime sequence, and a snapshot already includes its event_sequence.
export function applyRuntimeEvents(rows: Row[]): Row[] {
  const base = rows.find((row) => row.kind === "runtime");
  if (!base) return rows;
  const data = base.data;
  const hasSequence = Number.isFinite(data.event_sequence);
  let sequence = data.event_sequence;
  // A debug snapshot is published after a whole Runtime tick. Root bt_tick
  // events delimit subsequent ticks; visited must never accumulate across them.
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
  // Only the tree is mutated in place below. Other snapshot fields are replaced
  // on the overlay, so unrelated telemetry need not be deep-cloned per packet.
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
        // Runtime discards the previous cleaning tree when accepting a mission.
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
      // Only leaves update CleaningContext.stage; composites close the tick.
      if (!nodes.some((candidate) => candidate.parent === node.id))
        overlay.bt_stage = node.id;
      nextTick = !node.parent;
    }
    if (event.kind === "mission_result") overlay.last_result = event.data;
  }
  // Keep the original receive timestamp: event overlays are not new readiness
  // observations and must not make stale safety/module reports appear fresh.
  return rows.map((row) => (row === base ? { ...base, data: overlay } : row));
}
