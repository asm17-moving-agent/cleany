import { useEffect, useState } from "react";
import { Panel } from "@/components/panel";
import type { Data, Row } from "@/lib/monitor";
import "./resource-dashboard.css";

const percent = (v: number | null | undefined) =>
  v == null ? "미측정" : `${v.toFixed(1)}%`;
const bytes = (v: number | null | undefined) => {
  if (v == null) return "미측정";
  const units = ["B", "KiB", "MiB", "GiB", "TiB"];
  const i = Math.min(
    4,
    Math.max(0, Math.floor(Math.log2(Math.max(1, v)) / 10)),
  );
  return `${(v / 1024 ** i).toFixed(1)} ${units[i]}`;
};
function Trend({
  history,
  metric,
  label,
  unit,
  ceiling,
  details,
}: {
  history: Data[];
  metric: string;
  label: string;
  unit: string;
  ceiling?: number;
  details: [string, string][];
}) {
  const valid = (value: unknown): value is number =>
    typeof value === "number" && Number.isFinite(value);
  const values = history.map((d) => d[metric]).filter(valid);
  const maximum = ceiling || Math.max(1, ...values);
  const start = history[0]?.at || 0;
  const end = history.at(-1)?.at || start;
  const span = Math.max(1, end - start);
  const format = (value: number | null | undefined) =>
    value == null
      ? "미측정"
      : unit === "%"
        ? percent(value)
        : `${bytes(value)}/s`;
  const segments: string[] = [];
  let path = "";
  for (const d of history) {
    if (!valid(d[metric])) {
      if (path) segments.push(path);
      path = "";
      continue;
    }
    const x = 64 + ((d.at - start) / span) * 528;
    const y = 108 - Math.min(1, d[metric] / maximum) * 96;
    path += `${path ? " L" : "M"}${x},${y}`;
  }
  if (path) segments.push(path);
  const average = values.length
    ? values.reduce((sum, value) => sum + value, 0) / values.length
    : null;
  return (
    <section className="resource-trend" aria-label={`${label} 상세`}>
      <div className="resource-trend-heading">
        <h3>{label}</h3>
        <div>
          <small>현재</small>
          <strong>{format(history.at(-1)?.[metric])}</strong>
        </div>
      </div>
      <dl className="resource-trend-stats">
        {[
          ["표본 평균", average],
          ["최소", values.length ? Math.min(...values) : null],
          ["최대", values.length ? Math.max(...values) : null],
        ].map(([name, value]) => (
          <div key={String(name)}>
            <dt>{name}</dt>
            <dd>{format(value as number | null)}</dd>
          </div>
        ))}
      </dl>
      <svg viewBox="0 0 600 130" role="img" aria-label={`${label} 추이`}>
        {[0, 0.5, 1].map((fraction) => (
          <g key={fraction}>
            <path
              d={`M64 ${108 - fraction * 96}H592`}
              stroke="var(--border)"
              fill="none"
            />
            <text x="56" y={112 - fraction * 96} textAnchor="end">
              {unit === "%"
                ? `${maximum * fraction}%`
                : bytes(maximum * fraction)}
            </text>
          </g>
        ))}
        {segments.map((d, i) => (
          <path
            key={i}
            d={d}
            stroke="var(--primary)"
            strokeWidth="2"
            fill="none"
          />
        ))}
      </svg>
      <div className="resource-trend-period">
        <span>
          {history.length
            ? new Date(start * 1000).toLocaleTimeString()
            : "표본 대기"}
        </span>
        <span>
          {values.length}개 유효 표본 · {Math.round(end - start)}초
        </span>
        <span>
          {history.length ? new Date(end * 1000).toLocaleTimeString() : ""}
        </span>
      </div>
      <dl className="resource-trend-details">
        {details.map(([name, value]) => (
          <div key={name}>
            <dt>{name}</dt>
            <dd>{value}</dd>
          </div>
        ))}
      </dl>
    </section>
  );
}
export function ResourceDashboard({
  rows,
  replay,
}: {
  rows: Row[];
  replay: boolean;
}) {
  const data = rows.find(
    (r) => r.kind === "resources" && r.key === "host",
  )?.data;
  const [now, setNow] = useState(Date.now());
  const [sort, setSort] = useState("cpu");
  useEffect(() => {
    const timer = setInterval(() => setNow(Date.now()), 1000);
    return () => clearInterval(timer);
  }, []);
  if (!data)
    return (
      <Panel title="호스트 리소스">
        <p>
          {replay
            ? "이 시점에 기록된 리소스 표본이 없습니다."
            : "서버 호스트의 첫 리소스 표본을 기다립니다."}
        </p>
      </Panel>
    );
  const rosNodes: Data[] =
    rows.find((r) => r.kind === "graph" && r.key === "ros")?.data.nodes || [];
  const age = Math.max(0, now / 1000 - data.at);
  const stale = !replay && age > Math.max(3, 3 * data.interval_s);
  const history = (data.history || []).filter(
    (d: Data) => d.at >= data.at - 300,
  );
  const processes: Data[] =
    sort === "cpu" ? data.processes_cpu || [] : data.processes_memory || [];
  const trends = [
    {
      metric: "cpu_percent",
      label: "CPU",
      unit: "%",
      details: [
        ["논리 코어", `${data.cpu_cores?.length || 0}개`],
        [
          "관측 프로세스",
          `${data.process_count ?? 0}개 · 접근 불가 ${data.denied_processes ?? 0}개`,
        ],
      ],
    },
    {
      metric: "memory_percent",
      label: "메모리",
      unit: "%",
      details: [
        [
          "사용 / 전체",
          `${bytes(data.memory?.used)} / ${bytes(data.memory?.total)}`,
        ],
        ["사용 가능", bytes(data.memory?.available)],
        [
          "Swap",
          data.swap?.total === 0
            ? "비활성 · 할당 없음"
            : `${bytes(data.swap?.used)} / ${bytes(data.swap?.total)} (${percent(data.swap?.percent)})`,
        ],
      ],
    },
    {
      metric: "disk_read_bytes_s",
      label: "디스크 읽기",
      unit: "B/s",
      details: [
        [
          "루트 디스크 용량",
          `${bytes(data.disk?.used)} / ${bytes(data.disk?.total)} (${percent(data.disk?.percent)})`,
        ],
        [
          "남은 용량",
          data.disk ? bytes(data.disk.total - data.disk.used) : "미측정",
        ],
      ],
    },
    {
      metric: "disk_write_bytes_s",
      label: "디스크 쓰기",
      unit: "B/s",
      details: [
        ["I/O 관측 장치", data.disk_devices?.join(", ") || "미지원"],
        ["측정 범위", "관측 장치 합계 · 루트 디스크 용량과 별도"],
      ],
    },
    {
      metric: "network_rx_bytes_s",
      label: "네트워크 수신",
      unit: "B/s",
      details: [
        [
          "관측 인터페이스",
          data.networks?.map((n: Data) => n.name).join(", ") || "미지원",
        ],
        ["측정 범위", "loopback·가상 인터페이스 포함 합계"],
      ],
    },
    {
      metric: "network_tx_bytes_s",
      label: "네트워크 송신",
      unit: "B/s",
      details: [
        ["관측 인터페이스", `${data.networks?.length || 0}개`],
        ["집계 참고", "가상 인터페이스 트래픽이 중복될 수 있음"],
      ],
    },
  ];
  return (
    <div className="resources-dashboard">
      <div className="resource-source">
        <strong>서버 호스트 · {data.host}</strong>
        <span>
          {replay
            ? "기록 재생"
            : stale
              ? `수집 지연 · ${age.toFixed(0)}초 전`
              : "실시간 관측"}{" "}
          · {new Date(data.at * 1000).toLocaleString()} · {data.interval_s}초
          주기
        </span>
      </div>
      {stale && (
        <div className="notice warn">
          리소스 수집이 지연되었습니다. 아래 값은 마지막 관측값입니다.
        </div>
      )}
      {data.errors?.length > 0 && (
        <div className="notice warn">{data.errors.join(" · ")}</div>
      )}
      <Panel title="최근 5분 추이">
        <div className="resource-trends">
          {trends.map(({ metric, label, unit, details }) => (
            <Trend
              key={metric}
              history={history}
              metric={metric}
              label={label}
              unit={unit}
              ceiling={unit === "%" ? 100 : undefined}
              details={details as [string, string][]}
            />
          ))}
        </div>
      </Panel>
      <Panel
        title="프로세스 리소스"
        aside={
          <select
            aria-label="프로세스 정렬"
            value={sort}
            onChange={(e) => setSort(e.target.value)}
          >
            <option value="cpu">CPU 순</option>
            <option value="memory">메모리 순</option>
          </select>
        }
      >
        <p className="resource-note">
          관측 {data.process_count ?? 0}개 · 권한 부족{" "}
          {data.denied_processes ?? 0}개 · CPU 100% = 논리 코어 1개. ROS 노드와
          PID의 대응은 확인되지 않습니다.
        </p>
        <div className="resource-table resource-process-table">
          <table>
            <colgroup>
              <col style={{ width: "18%" }} />
              <col style={{ width: "40%" }} />
              <col style={{ width: "18%" }} />
              <col style={{ width: "24%" }} />
            </colgroup>
            <thead>
              <tr>
                <th>PID</th>
                <th>프로세스</th>
                <th>CPU</th>
                <th>메모리 RSS</th>
              </tr>
            </thead>
            <tbody>
              {processes.map((p) => (
                <tr key={p.pid}>
                  <td>{p.pid}</td>
                  <td title={p.name}>{p.name}</td>
                  <td>{percent(p.cpu_percent)}</td>
                  <td>{bytes(p.rss_bytes)}</td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>
      </Panel>
      <Panel title="ROS 발견 노드">
        <p className="resource-note">
          DDS domain에서 발견한 {rosNodes.length}개 노드 · 다른 호스트의 노드가
          포함될 수 있으며 위 프로세스와 자동 대응시키지 않습니다.
        </p>
        <div className="resource-node-list">
          {rosNodes.length ? (
            rosNodes
              .slice(0, 128)
              .map((node, i) => (
                <code key={`${node.name}:${i}`}>{node.name}</code>
              ))
          ) : (
            <p>발견된 ROS 노드가 없습니다.</p>
          )}
        </div>
        {rosNodes.length > 128 && (
          <p>처음 128개만 표시합니다. 전체 목록은 ROS 탐색기에서 확인하세요.</p>
        )}
      </Panel>
      <div className="resource-details">
        <Panel title="코어별 CPU">
          <div className="resource-cores">
            {data.cpu_cores?.map((v: number | null, i: number) => (
              <div key={i}>
                <span>CPU {i}</span>
                <strong>{percent(v)}</strong>
                <progress max="100" value={v ?? 0} aria-label={`CPU ${i}`} />
              </div>
            ))}
          </div>
        </Panel>
        <Panel title="네트워크 인터페이스">
          <div className="resource-table">
            <table>
              <thead>
                <tr>
                  <th>인터페이스</th>
                  <th>수신</th>
                  <th>송신</th>
                </tr>
              </thead>
              <tbody>
                {data.networks?.map((n: Data) => (
                  <tr key={n.name}>
                    <td>{n.name}</td>
                    <td>{bytes(n.rx_bytes_s)}/s</td>
                    <td>{bytes(n.tx_bytes_s)}/s</td>
                  </tr>
                ))}
              </tbody>
            </table>
          </div>
        </Panel>
        <Panel title="온도 · GPU">
          <div className="resource-sensors">
            {data.temperatures?.length ? (
              data.temperatures.map((t: Data) => (
                <div key={t.name}>
                  <span>{t.name}</span>
                  <strong>{t.celsius.toFixed(1)}°C</strong>
                </div>
              ))
            ) : (
              <p>온도 센서 미지원 또는 관측 불가</p>
            )}
            {data.gpu?.length ? (
              data.gpu.map((g: Data) => (
                <div key={g.name}>
                  <span>GPU {g.name}</span>
                  <strong>{percent(g.percent)}</strong>
                </div>
              ))
            ) : (
              <p>GPU 사용률 미지원 · DRM sysfs 제공 장치만 관측</p>
            )}
          </div>
        </Panel>
      </div>
    </div>
  );
}
