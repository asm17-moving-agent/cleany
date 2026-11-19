import React, { useMemo } from "react";
import {
  ReactFlow,
  Background,
  Controls,
  Handle,
  Position,
  MarkerType,
  type Node,
  type Edge,
  type NodeProps,
} from "@xyflow/react";
import { FitGraphToViewport } from "@/components/fit-graph-to-viewport";

type Connection = { name: string; publishers: string[]; subscribers: string[] };
type Point = { x: number; y: number };

function RosNode({ data }: NodeProps) {
  return (
    <>
      {[Position.Left, Position.Right, Position.Top, Position.Bottom].map(
        (position) => (
          <React.Fragment key={position}>
            <Handle
              id={`${position}-in`}
              type="target"
              position={position}
              style={
                position === Position.Top || position === Position.Bottom
                  ? { left: "42%" }
                  : { top: "42%" }
              }
            />
            <Handle
              id={`${position}-out`}
              type="source"
              position={position}
              style={
                position === Position.Top || position === Position.Bottom
                  ? { left: "58%" }
                  : { top: "58%" }
              }
            />
          </React.Fragment>
        ),
      )}
      <small>{data.kind as string}</small>
      <strong>{data.label as string}</strong>
    </>
  );
}
const nodeTypes = { ros: RosNode };

// Deterministic, bounded layout. It runs only when connection identities change.
function layout(ids: string[], links: [string, string][]): Map<string, Point> {
  const points = new Map<string, Point>();
  const adjacency = new Map(ids.map((id) => [id, new Set<string>()]));
  links.forEach(([a, b]) => {
    adjacency.get(a)!.add(b);
    adjacency.get(b)!.add(a);
  });
  const pending = new Set(ids);
  let componentX = 0;
  while (pending.size) {
    const component: string[] = [];
    const visit = [pending.values().next().value!];
    pending.delete(visit[0]);
    while (visit.length) {
      const id = visit.pop()!;
      component.push(id);
      for (const next of adjacency.get(id)!) {
        if (pending.delete(next)) visit.push(next);
      }
    }
    component.sort();
    const radius = Math.max(180, component.length * 32);
    const local = component.map((_, i) => ({
      x: Math.cos((i * Math.PI * 2) / component.length) * radius,
      y: Math.sin((i * Math.PI * 2) / component.length) * radius,
    }));
    const indices = new Map(component.map((id, i) => [id, i]));
    const pairs = links
      .filter(([a]) => indices.has(a))
      .map(([a, b]) => [indices.get(a)!, indices.get(b)!]);
    for (let iteration = 0; iteration < 160; iteration++) {
      const forces = local.map((p) => ({ x: -p.x * 0.002, y: -p.y * 0.002 }));
      for (let i = 0; i < local.length; i++)
        for (let j = i + 1; j < local.length; j++) {
          const dx = local[i].x - local[j].x;
          const dy = local[i].y - local[j].y;
          const distance = Math.max(1, Math.hypot(dx, dy));
          const repulsion = Math.min(16, 18000 / (distance * distance));
          const fx = (dx / distance) * repulsion;
          const fy = (dy / distance) * repulsion;
          forces[i].x += fx;
          forces[j].x -= fx;
          forces[i].y += fy;
          forces[j].y -= fy;
          // Keep rectangular labels apart, including non-neighbor nodes.
          if (Math.abs(dx) < 230 && Math.abs(dy) < 100) {
            if (230 - Math.abs(dx) < 100 - Math.abs(dy)) {
              const push = (230 - Math.abs(dx)) * 0.15 * (dx >= 0 ? 1 : -1);
              forces[i].x += push;
              forces[j].x -= push;
            } else {
              const push = (100 - Math.abs(dy)) * 0.15 * (dy >= 0 ? 1 : -1);
              forces[i].y += push;
              forces[j].y -= push;
            }
          }
        }
      for (const [i, j] of pairs) {
        const dx = local[j].x - local[i].x;
        const dy = local[j].y - local[i].y;
        const distance = Math.max(1, Math.hypot(dx, dy));
        const pull = (distance - 255) * 0.035;
        forces[i].x += (dx / distance) * pull;
        forces[j].x -= (dx / distance) * pull;
        forces[i].y += (dy / distance) * pull;
        forces[j].y -= (dy / distance) * pull;
      }
      local.forEach((p, i) => {
        p.x += Math.max(-16, Math.min(16, forces[i].x));
        p.y += Math.max(-16, Math.min(16, forces[i].y));
      });
    }
    const minX = Math.min(...local.map((p) => p.x));
    const minY = Math.min(...local.map((p) => p.y));
    component.forEach((id, i) =>
      points.set(id, {
        x: componentX + local[i].x - minX,
        y: local[i].y - minY,
      }),
    );
    componentX += Math.max(...local.map((p) => p.x)) - minX + 310;
  }
  return points;
}

export const RosGraph = React.memo(function RosGraph({
  spec,
}: {
  spec: string;
}) {
  const connections = useMemo(() => {
    const topics: Connection[] = JSON.parse(spec);
    const labels = new Map<string, { label: string; kind: string }>();
    const links: [string, string][] = [];
    for (const topic of topics) {
      const topicId = `topic:${topic.name}`;
      labels.set(topicId, { label: topic.name, kind: "Topic" });
      for (const publisher of topic.publishers) {
        const id = `node:${publisher}`;
        labels.set(id, { label: publisher, kind: "Node" });
        links.push([id, topicId]);
      }
      for (const subscriber of topic.subscribers) {
        const id = `node:${subscriber}`;
        labels.set(id, { label: subscriber, kind: "Node" });
        links.push([topicId, id]);
      }
    }
    const positions = layout([...labels.keys()].sort(), links);
    const nodes: Node[] = [...labels].map(([id, data]) => ({
      id,
      data,
      position: positions.get(id)!,
      type: "ros",
      className: "ros-connection-node react-flow__node-default",
      width: 190,
      height: 68,
      style: {
        width: 190,
        height: 68,
        background: data.kind === "Topic" ? "var(--info-bg)" : "var(--card)",
      },
    }));
    const edges: Edge[] = links.map(([source, target]) => {
      const a = positions.get(source)!;
      const b = positions.get(target)!;
      const horizontal = Math.abs(b.x - a.x) > Math.abs(b.y - a.y);
      const from = horizontal
        ? b.x > a.x
          ? Position.Right
          : Position.Left
        : b.y > a.y
          ? Position.Bottom
          : Position.Top;
      const to = horizontal
        ? b.x > a.x
          ? Position.Left
          : Position.Right
        : b.y > a.y
          ? Position.Top
          : Position.Bottom;
      return {
        id: JSON.stringify([source, target]),
        source,
        target,
        sourceHandle: `${from}-out`,
        targetHandle: `${to}-in`,
        type: "bezier",
        markerEnd: { type: MarkerType.Arrow, color: "var(--graph-edge)" },
      };
    });
    return { nodes, edges };
  }, [spec]);
  return (
    <div className="flow large ros-connections">
      <ReactFlow
        {...connections}
        nodeTypes={nodeTypes}
        fitView
        minZoom={0.1}
        nodesDraggable={false}
        nodesConnectable={false}
      >
        <FitGraphToViewport topology={spec} />
        <Background color="var(--graph-dot)" />
        <Controls showInteractive={false} />
      </ReactFlow>
    </div>
  );
});
