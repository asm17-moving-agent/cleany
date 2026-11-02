import { Fragment, useEffect, useState } from "react";
import { ChevronRight } from "lucide-react";
import { Panel } from "./panel";
import { Button } from "./ui/button";
import { type Data, type Row } from "@/lib/monitor";
import "./recording-controls.css";

const labels: Record<string, string> = {
  resources: "호스트 리소스",
  slam: "SLAM 서브맵, 스캔, 그래프",
  motion: "위치, 속도, 미션",
  paths: "계획, 제어 경로",
  costmaps: "Costmap",
  bt: "Nav2 BT",
  safety: "안전 판단",
  logs: "로그",
  lidar: "LiDAR 원본",
  point_cloud: "Depth point cloud 원본",
  voxel_layer: "Nav2 voxel layer 원본",
  voxel_world: "Voxel world, 장애물 메모리",
  camera: "카메라 원본",
};
const heavy = ["lidar", "point_cloud", "voxel_layer", "voxel_world", "camera"];
// Older monitor processes and saved sessions can still send previous labels.
const displayLabel = (value: string) => value.replace(/\s*\u00b7\s*/g, ", ");
export function RecordingControls({ replay }: { replay: boolean }) {
  const [live, setLive] = useState<Data | null>(null);
  const [draft, setDraft] = useState<Data | null>(null);
  const [message, setMessage] = useState("");
  const [busy, setBusy] = useState(false);
  const [available, setAvailable] = useState(false);
  const [expanded, setExpanded] = useState<Record<string, boolean>>({});
  useEffect(() => {
    let stopped = false;
    const refresh = async () => {
      try {
        const response = await fetch("/api/recording/settings");
        if (!response.ok) throw Error(await response.text());
        const data = await response.json();
        if (!stopped) {
          setAvailable(true);
          setLive(data);
          setDraft((value) => value || data.settings);
        }
      } catch (e) {
        if (!stopped) {
          setAvailable(false);
          setMessage(String(e));
        }
      }
    };
    void refresh();
    const timer = setInterval(refresh, 1000);
    return () => {
      stopped = true;
      clearInterval(timer);
    };
  }, []);
  const apply = async (settings: Data, newSession = false) => {
    setBusy(true);
    setMessage("");
    try {
      const response = await fetch(
        newSession ? "/api/recording/session" : "/api/recording/settings",
        {
          method: newSession ? "POST" : "PUT",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify(settings),
        },
      );
      if (!response.ok) throw Error(await response.text());
      const data = await response.json();
      setLive(data);
      setDraft(data.settings);
      setMessage(
        data.reason ||
          (newSession
            ? "새 기록 파일로 녹화를 시작했습니다."
            : live?.settings.active && !settings.active
              ? "녹화를 정지했습니다."
              : "녹화 설정을 적용했습니다."),
      );
    } catch (e) {
      setMessage(String(e));
    } finally {
      setBusy(false);
    }
  };
  const entriesFor = (group: string): Data[] =>
    Object.entries(live?.source_fields || {})
      .filter(([, field]) => (field as Data).group === group)
      .flatMap<Data>(([name, field]) => {
        const spec = field as Data;
        const value =
          draft?.sources?.[name] ?? live?.source_applied?.[name] ?? "";
        return spec.multiple
          ? (Array.isArray(value) ? value : []).map((topic, index) => ({
              id: `${name}:${index}`,
              name,
              index,
              label: `속도 체인 ${index + 1}`,
              value: topic,
              defaultHz: spec.default_hz,
              maximum: spec.maximum,
            }))
          : [
              {
                id: name,
                name,
                label: displayLabel(spec.label),
                value,
                internal: spec.internal,
                defaultHz: spec.default_hz,
                maximum: spec.maximum,
              },
            ];
      });
  const toggleGroup = (group: string, checked: boolean) => {
    if (!draft) return;
    const source_enabled = { ...draft.source_enabled };
    for (const item of entriesFor(group)) source_enabled[item.id] = checked;
    setDraft({
      ...draft,
      enabled: { ...draft.enabled, [group]: checked },
      source_enabled,
    });
  };
  const toggleSource = (group: string, id: string, checked: boolean) => {
    if (!draft) return;
    const items = entriesFor(group);
    const source_enabled = { ...draft.source_enabled };
    if (!draft.enabled[group])
      for (const item of items) source_enabled[item.id] = false;
    source_enabled[id] = checked;
    setDraft({
      ...draft,
      source_enabled,
      enabled: {
        ...draft.enabled,
        [group]: items.some((item) => source_enabled[item.id] ?? true),
      },
    });
  };
  const updateSource = (item: Data, text: string) => {
    if (!draft) return;
    const sources = { ...draft.sources };
    if (item.index !== undefined) {
      const values = [
        ...(sources[item.name] ?? live?.source_applied?.[item.name] ?? []),
      ];
      values[item.index] = text;
      sources[item.name] = values;
    } else sources[item.name] = text;
    setDraft({ ...draft, sources });
  };
  const maximumLabel = (maximum: Data | undefined) => (
    <span
      title={`${(maximum?.estimated ?? true) ? "추정 상한" : "코드, 설정 기준"}: ${displayLabel(maximum?.detail || "토픽 역할 기준 추정")}. 실행 설정, 발행자에 따라 달라질 수 있습니다.`}
    >
      ({(maximum?.estimated ?? true) ? "≈" : ""}
      {maximum?.hz ?? 100} Hz)
    </span>
  );
  const profile = live?.rate_profiles?.[draft?.preset];
  const profileModified =
    draft &&
    profile &&
    (Object.entries(profile.source_hz).some(
      ([key, rate]) => draft.source_hz?.[key] !== rate,
    ) ||
      Object.entries(profile.hz).some(
        ([key, rate]) => draft.hz?.[key] !== rate,
      ));
  const recording = !!live?.settings.active && !live?.reason;
  return (
    <Panel
      title="녹화 설정"
      aside={
        <span>
          {!available
            ? "logger 연결 끊김"
            : live?.reason
              ? "저장 중단"
              : live?.settings.active
                ? "녹화 중"
                : "녹화 정지"}
        </span>
      }
    >
      {live?.source_pending && (
        <p role="status">
          {live.source_live
            ? "기본 관측 토픽 변경 적용 대기 중입니다."
            : "ROS 관측 노드가 연결되지 않아 토픽 설정은 저장만 되었습니다. 관측 노드 실행 시 적용됩니다."}
        </p>
      )}
      {replay && (
        <p role="status">
          기록 재생 중에는 녹화 설정을 변경할 수 없습니다. 실시간으로 전환해
          주세요.
        </p>
      )}
      {draft && (
        <fieldset
          className="recording-options"
          disabled={!available || busy || replay}
        >
          <p>
            가공 데이터는 SQLite에, 선택한 ROS 원본은 rosbag2에 저장합니다.
            카메라, 점군, voxel 원본은 용량이 큽니다. 필요한 토픽과 기록 빈도를
            선택하세요.
          </p>
          <p>
            대시보드를 닫아도 logger는 녹화를 계속합니다.{" "}
            {recording && "설정을 변경하려면 녹화를 정지하세요."}
          </p>
          <div className="recording-toolbar">
            <label>
              주기 프로필{" "}
              <select
                disabled={recording}
                aria-label="주기 프로필"
                value={draft.preset}
                onChange={(e) => {
                  const preset = e.target.value;
                  const profile = live?.rate_profiles?.[preset];
                  if (!profile) return;
                  setDraft({
                    ...draft,
                    preset,
                    rate_defaults_version: 4,
                    source_hz: { ...profile.source_hz },
                    hz: { ...draft.hz, ...profile.hz },
                  });
                }}
              >
                <option value="low">저사양</option>
                <option value="standard">기본</option>
                <option value="precise">정밀</option>
              </select>
            </label>
            {profileModified && <small>주기 수정됨</small>}
            <label>
              세션 용량 제한{" "}
              <input
                disabled={recording}
                aria-label="세션 용량 MiB"
                type="number"
                min={10}
                max={1024}
                value={draft.limit_mib}
                onChange={(e) =>
                  setDraft({ ...draft, limit_mib: Number(e.target.value) })
                }
              />{" "}
              MiB
            </label>
          </div>
          <div className="recording-table">
            <table>
              <thead>
                <tr>
                  <th>녹화 항목</th>
                  <th>기록 빈도</th>
                  <th>관측 데이터 / 원본 토픽</th>
                  <th>현재 기록 상태</th>
                </tr>
              </thead>
              <tbody>
                {Object.entries(labels).map(([key, label]) => {
                  const raw = heavy.includes(key);
                  const rawSelected = heavy.filter(
                    (name) => draft.enabled[name],
                  ).length;
                  const rawHeader = key === "lidar" && (
                    <tr>
                      <td>
                        <div className="recording-group-label">
                          <button
                            type="button"
                            className="recording-expand"
                            aria-label="센서 원본 토픽 펼치기"
                            aria-expanded={!!expanded.raw}
                            onClick={() =>
                              setExpanded({ ...expanded, raw: !expanded.raw })
                            }
                          >
                            <ChevronRight
                              size={16}
                              style={{
                                transform: expanded.raw
                                  ? "rotate(90deg)"
                                  : undefined,
                              }}
                            />
                          </button>
                          <label>
                            <input
                              disabled={recording}
                              type="checkbox"
                              checked={rawSelected === heavy.length}
                              ref={(node) => {
                                if (node)
                                  node.indeterminate =
                                    rawSelected > 0 &&
                                    rawSelected < heavy.length;
                              }}
                              onChange={(e) =>
                                setDraft({
                                  ...draft,
                                  enabled: {
                                    ...draft.enabled,
                                    ...Object.fromEntries(
                                      heavy.map((name) => [
                                        name,
                                        e.target.checked,
                                      ]),
                                    ),
                                  },
                                })
                              }
                            />
                            센서 원본, rosbag2
                          </label>
                        </div>
                      </td>
                      <td>토픽별 지정, 대용량</td>
                      <td />
                      <td>
                        {rawSelected} / {heavy.length}개 선택
                      </td>
                    </tr>
                  );
                  if (raw && !expanded.raw)
                    return <Fragment key={key}>{rawHeader}</Fragment>;
                  const items = entriesFor(key);
                  const selected = draft.enabled[key]
                    ? items.filter(
                        (item) => draft.source_enabled?.[item.id] ?? true,
                      ).length
                    : 0;
                  const partial = selected > 0 && selected < items.length;
                  return (
                    <Fragment key={key}>
                      {rawHeader}
                      <tr className={raw ? "recording-source-row" : undefined}>
                        <td>
                          <div className="recording-group-label">
                            {items.length > 0 ? (
                              <button
                                type="button"
                                className="recording-expand"
                                aria-label={`${label} 토픽 펼치기`}
                                aria-expanded={!!expanded[key]}
                                onClick={() =>
                                  setExpanded({
                                    ...expanded,
                                    [key]: !expanded[key],
                                  })
                                }
                              >
                                <ChevronRight
                                  size={16}
                                  style={{
                                    transform: expanded[key]
                                      ? "rotate(90deg)"
                                      : undefined,
                                  }}
                                />
                              </button>
                            ) : !raw ? (
                              <span className="recording-expand-spacer" />
                            ) : null}
                            <label>
                              <input
                                disabled={recording}
                                type="checkbox"
                                checked={
                                  items.length
                                    ? selected === items.length
                                    : (draft.enabled[key] ?? false)
                                }
                                ref={(node) => {
                                  if (node) node.indeterminate = partial;
                                }}
                                onChange={(e) =>
                                  toggleGroup(key, e.target.checked)
                                }
                              />
                              {label}
                            </label>
                          </div>
                        </td>
                        <td>
                          {!raw && key !== "resources" ? null : (
                            <label className="recording-rate">
                              <input
                                disabled={recording}
                                type="number"
                                min={0.1}
                                max={1000}
                                step={0.1}
                                aria-label={`${label} 기록 Hz`}
                                value={draft.hz[key]}
                                onChange={(e) =>
                                  setDraft({
                                    ...draft,
                                    hz: {
                                      ...draft.hz,
                                      [key]: Number(e.target.value),
                                    },
                                  })
                                }
                              />{" "}
                              Hz / {maximumLabel(live?.rate_maximums?.[key])}
                            </label>
                          )}
                        </td>
                        <td>
                          {heavy.includes(key) ? (
                            <div className="recording-source">
                              <input
                                disabled={recording}
                                aria-label={`${label} 토픽`}
                                value={draft.topics[key]}
                                onChange={(e) =>
                                  setDraft({
                                    ...draft,
                                    topics: {
                                      ...draft.topics,
                                      [key]: e.target.value,
                                    },
                                  })
                                }
                              />
                            </div>
                          ) : null}
                        </td>
                        <td>
                          {live?.status[key]?.state || "확인 중"}
                          {live?.status[key]?.count
                            ? `, ${live.status[key].count}건`
                            : ""}
                          {live?.status[key]?.error && (
                            <small>{live.status[key].error}</small>
                          )}
                          {live?.status[key]?.dropped ? (
                            <small>유실 {live.status[key].dropped}건</small>
                          ) : null}
                        </td>
                      </tr>
                      {expanded[key] &&
                        items.map((item) => (
                          <tr key={item.id} className="recording-source-row">
                            <td>
                              <label>
                                <input
                                  disabled={recording}
                                  type="checkbox"
                                  aria-label={`${item.label} 기록`}
                                  checked={
                                    !!draft.enabled[key] &&
                                    (draft.source_enabled?.[item.id] ?? true)
                                  }
                                  onChange={(e) =>
                                    toggleSource(key, item.id, e.target.checked)
                                  }
                                />
                                {item.label}
                              </label>
                            </td>
                            <td>
                              <label className="recording-rate">
                                <input
                                  disabled={recording}
                                  type="number"
                                  min={0.1}
                                  max={1000}
                                  step={0.1}
                                  aria-label={`${item.label} 기록 Hz`}
                                  placeholder="수신마다"
                                  title="비워두면 수신마다 기록"
                                  value={
                                    (draft.source_hz?.[item.id] === undefined
                                      ? item.defaultHz
                                      : draft.source_hz[item.id]) ?? ""
                                  }
                                  onChange={(e) =>
                                    setDraft({
                                      ...draft,
                                      source_hz: {
                                        ...draft.source_hz,
                                        [item.id]:
                                          e.target.value === ""
                                            ? null
                                            : Number(e.target.value),
                                      },
                                    })
                                  }
                                />{" "}
                                Hz / {maximumLabel(item.maximum)}
                              </label>
                            </td>
                            <td>
                              {item.internal ? null : (
                                <input
                                  disabled={recording}
                                  className="recording-basic-topic"
                                  aria-label={`${item.label} 기본 토픽`}
                                  value={item.value}
                                  onChange={(e) =>
                                    updateSource(item, e.target.value)
                                  }
                                />
                              )}
                            </td>
                            <td>
                              <small>
                                {draft.enabled[key] &&
                                (draft.source_enabled?.[item.id] ?? true)
                                  ? "기록 선택"
                                  : "미녹화"}
                              </small>
                            </td>
                          </tr>
                        ))}
                    </Fragment>
                  );
                })}
              </tbody>
            </table>
          </div>
          <div className="recording-toolbar">
            <Button
              disabled={recording}
              variant="outline"
              onClick={() =>
                void apply({ ...draft, active: !!live?.settings.active })
              }
            >
              설정 적용
            </Button>
            <Button
              variant={recording ? "destructive" : "default"}
              className={recording ? "recording-stop" : "recording-start"}
              onClick={() =>
                void apply({ ...draft, active: !recording }, !recording)
              }
            >
              <span aria-hidden="true">{recording ? "■" : "●"}</span>
              {recording ? "녹화 정지" : "녹화 시작"}
            </Button>
            <small>
              용량 한도는 SQLite와 rosbag2의 합계입니다. 한도에 도달하면 기록을
              중단하며 기존 기록은 보존합니다.
            </small>
          </div>
        </fieldset>
      )}
      {(message || live?.reason) && (
        <p role="status">{message || live?.reason}</p>
      )}
    </Panel>
  );
}
export function RecordingReplayNotice({
  rows,
  sessionId,
  timeAt,
}: {
  rows: Row[];
  sessionId: string;
  timeAt: number;
}) {
  const settings = rows.find((r) => r.kind === "recording_policy")?.data;
  const status =
    rows.find((r) => r.kind === "recording_status")?.data.status || {};
  const archive = rows.find((r) => r.kind === "recording_raw")?.data;
  const raw = archive?.items || [];
  return (
    <details className="recording-replay">
      <summary>녹화 범위, 원본 자료</summary>
      {settings ? (
        <p>
          {!settings.active && "이 시점은 녹화가 정지된 구간입니다. "}미녹화
          항목:{" "}
          {Object.keys(labels)
            .filter((key) => !settings.enabled[key])
            .map((key) => labels[key])
            .join(", ") || "없음"}
        </p>
      ) : (
        <p>이전 형식의 기록: 녹화 항목 설정 정보가 없습니다.</p>
      )}
      {settings?.source_enabled && (
        <p>
          미녹화 개별 항목:{" "}
          {Object.entries(settings.source_enabled)
            .filter(([, enabled]) => !enabled)
            .map(([id]) => {
              const [name, index] = id.split(":");
              const config = rows.find((r) => r.kind === "config")?.data || {};
              const value = settings.sources?.[name] ?? config[name];
              return name === "_computed_motion"
                ? "주행 요약, TF 위치"
                : index !== undefined
                  ? value?.[Number(index)] || id
                  : value || id;
            })
            .join(", ") || "없음"}
        </p>
      )}
      {heavy.map((key) => {
        const items = raw.filter((item: Data) => item.category === key);
        return (
          <div key={key}>
            <strong>{labels[key]}: </strong>
            {items.length ? (
              <>
                {items.reduce((sum: number, item: Data) => sum + item.count, 0)}
                건,{" "}
                <a
                  href={`/api/recordings/${sessionId}/raw?category=${key}&at=${timeAt}`}
                >
                  직전 원본 내려받기
                </a>
              </>
            ) : settings && !settings.enabled[key] ? (
              "미녹화"
            ) : (
              status[key]?.state || "원본 수신 기록 없음"
            )}
            {items.map((item: Data) => (
              <small key={item.topic}>
                {item.topic}, {item.message_type}
              </small>
            ))}
          </div>
        );
      })}
      <p>
        {archive?.format === "rosbag2" ? (
          <>
            원본은 별도 rosbag2 기록입니다. 가공 데이터 SQLite 내보내기에는
            원본이 포함되지 않습니다.{" "}
            {archive.bag_available ? (
              <a href={`/api/recordings/${sessionId}/bag`}>rosbag2 내려받기</a>
            ) : !archive.bag_recording &&
              ["incomplete", "recording"].includes(archive.bag_state) ? (
              "rosbag2가 정상 종료되지 않아 내보낼 수 없습니다."
            ) : (
              "rosbag2 내려받기는 녹화 정지 후 가능합니다."
            )}
          </>
        ) : archive?.format === "legacy_sqlite" ? (
          "이전 형식의 원본은 SQLite 내보내기에 포함됩니다."
        ) : (
          "이 시점까지 원본 수신 기록이 없습니다."
        )}{" "}
        rosbag2 영상, 점군은 RViz(ROS 토픽 재생) 또는 Foxglove에서 확인할 수
        있습니다.
      </p>
    </details>
  );
}
