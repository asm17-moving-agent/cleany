import { useEffect, useMemo, useState } from "react";
import {
  ReactFlow,
  Background,
  Controls,
  Handle,
  Position,
  type NodeProps,
} from "@xyflow/react";
import "@xyflow/react/dist/style.css";
import { type Row, time } from "@/lib/monitor";
import "./nav2-bt-tree.css";

type TreeNode = {
  id: string;
  parent: string | null;
  order: number;
  type: string;
  label: string;
  ports: Record<string, string>;
};
const labels: Record<string, string> = {
  PlanThenNavigate: "경로 계획 후 주행",
  InitialPlanWithPatience: "초기 계획 · 재시도",
  NavigateWithPatience: "주행 · 복구",
  CheckBeforeFollowing: "경로 확인 후 추종",
  PathClearOrReplan: "유효 경로 또는 재계획",
  UpcomingPathClear: "앞쪽 경로 검사",
  ControllerFailurePatience: "제어 실패 후 재계획",
  ComputePathToPose: "경로 계산",
  FollowPath: "경로 추종",
  YieldWait: "재시도 대기",
  TruncatePathLocal: "앞쪽 경로 추출",
  IsPathValid: "경로 유효성 검사",
};
const descriptions: Record<string, string> = {
  Sequence:
    "자식을 순서대로 실행합니다. 실패하면 다음 단계로 넘어가지 않습니다.",
  ReactiveSequence:
    "앞선 조건을 매번 확인하고, 조건이 바뀌면 뒤의 실행을 중단합니다.",
  ReactiveFallback: "앞선 조건이 실패하면 다음 대안을 실행합니다.",
  RecoveryNode:
    "첫 작업이 실패하면 복구 단계를 실행하고 설정된 횟수만큼 재시도합니다.",
  ComputePathToPose: "현재 위치에서 목표까지 전역 경로를 계산합니다.",
  FollowPath:
    "계산된 경로를 따라 속도 명령을 생성합니다. 실제 속도는 안전 계층의 제한을 받습니다.",
  TruncatePathLocal: "로봇 앞의 경로 일부를 잘라 장애물 검사에 전달합니다.",
  IsPathValid: "검사할 경로가 현재 장애물 지도에서 유효한지 확인합니다.",
  YieldWait: "재시도 전에 잠시 기다립니다.",
};
function Tile({ data }: NodeProps) {
  return (
    <div className="nav2-tree-node" data-status={String(data.status)}>
      <Handle type="target" position={Position.Top} />
      <strong>{String(data.label)}</strong>
      <span>{String(data.type)}</span>
      <b>{String(data.status)}</b>
      <Handle type="source" position={Position.Bottom} />
    </div>
  );
}
const nodeTypes = { bt: Tile };
export function Nav2BtTree({
  row,
  timeAt,
  replay,
}: {
  row: Row;
  timeAt: number;
  replay: boolean;
}) {
  const tree = row.data;
  const [selected, select] = useState<string | null>(null);
  const topology = useMemo(() => {
    const source: TreeNode[] = tree.nodes || [];
    const positions = new Map<string, { x: number; y: number }>();
    let leaf = 0;
    const visit = (node: TreeNode, depth: number): number => {
      const children = source
        .filter((n) => n.parent === node.id)
        .sort((a, b) => a.order - b.order);
      const xs = children.map((n) => visit(n, depth + 1));
      const x = xs.length ? (xs[0] + xs[xs.length - 1]) / 2 : leaf++ * 210;
      positions.set(node.id, { x, y: depth * 125 });
      return x;
    };
    source.filter((n) => !n.parent).forEach((n) => visit(n, 0));
    return {
      source,
      positions,
      edges: source
        .filter((n) => n.parent)
        .map((n) => ({ id: n.id, source: n.parent!, target: n.id })),
    };
  }, [tree.xml_sha256]);
  useEffect(() => select(null), [tree.boot_id, tree.epoch, tree.xml_sha256]);
  const stale = timeAt - row.received_at > 2;
  const node = topology.source.find((n) => n.id === selected);
  const history = (tree.history || [])
    .filter((event: any) => !selected || event.node === selected)
    .slice(-20)
    .reverse();
  return (
    <section className="nav2-tree" aria-label="Nav2 실행 트리">
      <div className="nav2-tree-summary">
        <strong>
          {replay ? "기록 재생 · " : ""}
          {stale
            ? "연결 끊김 · 마지막 관측"
            : !tree.active
              ? "Nav2 비활성"
              : [4, 5, 6].includes(tree.goal_status)
                ? "주행 종료 · 마지막 관측"
                : "실행 상태 관측 중"}
        </strong>
        <span>
          {tree.reason ||
            (tree.observed
              ? "노드를 선택하면 역할과 입출력을 확인할 수 있습니다."
              : "실행 상태 수신 대기")}
        </span>
        <small>
          UNKNOWN은 미관측입니다. 목표 연결은 수신 시점 기준이며, 이벤트
          자체에는 목표 ID가 없습니다.
        </small>
      </div>
      <div className="nav2-tree-canvas">
        <ReactFlow
          key={tree.xml_sha256}
          nodeTypes={nodeTypes}
          nodes={topology.source.map((n) => ({
            id: n.id,
            type: "bt",
            initialWidth: 190,
            initialHeight: 90,
            position: topology.positions.get(n.id)!,
            selected: n.id === selected,
            data: {
              label: labels[n.label] || n.label,
              type: n.type,
              status: tree.states?.[n.id] || "UNKNOWN",
            },
          }))}
          edges={topology.edges}
          nodesDraggable={false}
          nodesConnectable={false}
          fitView
          minZoom={0.1}
          onNodeClick={(_, n) => select(n.id)}
        >
          <Background />
          <Controls showInteractive={false} />
        </ReactFlow>
      </div>
      {node && (
        <div className="nav2-tree-detail">
          <strong>
            {node.label} · {tree.states?.[node.id] || "UNKNOWN"}
          </strong>
          <p>{descriptions[node.type] || `${node.type} 노드`}</p>
          <small>인스턴스: {node.id}</small>
          <dl>
            {Object.entries(node.ports).map(([key, value]) => (
              <div key={key}>
                <dt>{key}</dt>
                <dd>{value}</dd>
              </div>
            ))}
          </dl>
        </div>
      )}
      <details>
        <summary>최근 상태 전이 {selected ? "· 선택한 노드" : ""}</summary>
        {history.length ? (
          history.map((event: any) => (
            <div key={event.sequence}>
              {time(event.received_at)} · {event.node} · {event.previous} →{" "}
              {event.status}
            </div>
          ))
        ) : (
          <p>관측한 상태 전이가 없습니다.</p>
        )}
      </details>
      <details>
        <summary>실행 XML 정보</summary>
        <small>
          {tree.source}
          <br />
          SHA256: {tree.xml_sha256}
          <br />
          목표: {tree.goal_id || "미관측"} · 관측 구간: {tree.epoch}
        </small>
      </details>
    </section>
  );
}
