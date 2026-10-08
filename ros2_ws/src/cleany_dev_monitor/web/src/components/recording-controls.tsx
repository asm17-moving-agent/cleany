import { useEffect, useState } from "react";
import { Panel } from "./panel";
import { Button } from "./ui/button";
import { type Data, type Row } from "@/lib/monitor";
import "./recording-controls.css";

const labels: Record<string, string> = {
  motion: "위치·속도·미션",
  paths: "계획·제어 경로",
  costmaps: "Costmap",
  bt: "Nav2 BT",
  safety: "안전 판단",
  logs: "로그",
  lidar: "LiDAR 원본",
  point_cloud: "Depth point cloud 원본",
  voxel_layer: "Nav2 voxel layer 원본",
  voxel_world: "Voxel world·장애물 메모리",
  camera: "카메라 원본",
};
const heavy = ["lidar", "point_cloud", "voxel_layer", "voxel_world", "camera"];
export function RecordingControls({ replay }: { replay: boolean }) {
  const [live, setLive] = useState<Data | null>(null);
  const [draft, setDraft] = useState<Data | null>(null);
  const [message, setMessage] = useState("");
  const [busy, setBusy] = useState(false);
  useEffect(() => {
    let stopped = false;
    const refresh = async () => {
      try {
        const response = await fetch("/api/recording/settings");
        if (!response.ok) throw Error("녹화 설정을 읽을 수 없습니다.");
        const data = await response.json();
        if (!stopped) {
          setLive(data);
          setDraft((value) => value || data.settings);
        }
      } catch (e) {
        if (!stopped) setMessage(String(e));
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
      setMessage("녹화 설정을 적용했습니다.");
    } catch (e) {
      setMessage(String(e));
    } finally {
      setBusy(false);
    }
  };
  return (
    <Panel
      title="녹화 설정"
      aside={
        <span>
          {live?.reason
            ? "저장 중단"
            : live?.settings.active
              ? "녹화 중"
              : "일시정지"}
        </span>
      }
    >
      <p>
        화면 표시와 별도로 기록할 항목을 선택합니다. 원본 데이터는 공간 표본을
        줄이지 않고 설정한 주기로 저장합니다.
      </p>
      {replay && (
        <p role="status">
          기록 재생 중에는 녹화 설정을 변경할 수 없습니다. 실시간으로 전환해
          주세요.
        </p>
      )}
      {draft && (
        <fieldset className="recording-options" disabled={busy || replay}>
          <div className="recording-toolbar">
            <label>
              프리셋{" "}
              <select
                aria-label="녹화 프리셋"
                value={draft.preset}
                onChange={(e) => {
                  const preset = e.target.value;
                  setDraft({
                    ...draft,
                    preset,
                    enabled:
                      preset === "custom"
                        ? draft.enabled
                        : Object.fromEntries(
                            Object.keys(labels).map((key) => [
                              key,
                              !heavy.includes(key) ||
                                (preset === "obstacles" && key !== "camera"),
                            ]),
                          ),
                  });
                }}
              >
                <option value="basic">기본 주행</option>
                <option value="obstacles">장애물 분석</option>
                <option value="custom">사용자 지정</option>
              </select>
            </label>
            <label>
              세션 용량 제한{" "}
              <input
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
                  <th>주기 / 토픽</th>
                  <th>현재 기록 상태</th>
                </tr>
              </thead>
              <tbody>
                {Object.entries(labels).map(([key, label]) => (
                  <tr key={key}>
                    <td>
                      <label>
                        <input
                          type="checkbox"
                          checked={draft.enabled[key]}
                          onChange={(e) =>
                            setDraft({
                              ...draft,
                              preset: "custom",
                              enabled: {
                                ...draft.enabled,
                                [key]: e.target.checked,
                              },
                            })
                          }
                        />
                        {label}
                      </label>
                    </td>
                    <td>
                      {heavy.includes(key) ? (
                        <div className="recording-source">
                          <label>
                            <input
                              type="number"
                              min={0.1}
                              max={10}
                              step={0.1}
                              aria-label={`${label} 기록 Hz`}
                              value={draft.hz[key]}
                              onChange={(e) =>
                                setDraft({
                                  ...draft,
                                  preset: "custom",
                                  hz: {
                                    ...draft.hz,
                                    [key]: Number(e.target.value),
                                  },
                                })
                              }
                            />{" "}
                            Hz
                          </label>
                          <input
                            aria-label={`${label} 토픽`}
                            value={draft.topics[key]}
                            onChange={(e) =>
                              setDraft({
                                ...draft,
                                preset: "custom",
                                topics: {
                                  ...draft.topics,
                                  [key]: e.target.value,
                                },
                              })
                            }
                          />
                        </div>
                      ) : (
                        "기존 관측 주기"
                      )}
                    </td>
                    <td>
                      {live?.status[key]?.state || "확인 중"}
                      {live?.status[key]?.count
                        ? ` · ${live.status[key].count}건`
                        : ""}
                      {live?.status[key]?.error && (
                        <small>{live.status[key].error}</small>
                      )}
                      {live?.status[key]?.dropped ? (
                        <small>유실 {live.status[key].dropped}건</small>
                      ) : null}
                    </td>
                  </tr>
                ))}
              </tbody>
            </table>
          </div>
          <div className="recording-toolbar">
            <Button onClick={() => void apply(draft)}>설정 적용</Button>
            <Button
              variant="outline"
              onClick={() =>
                void apply({
                  ...draft,
                  active: !live?.settings.active,
                })
              }
            >
              {live?.settings.active ? "녹화 일시정지" : "녹화 재개"}
            </Button>
            <Button variant="outline" onClick={() => void apply(draft, true)}>
              새 녹화 시작
            </Button>
            <small>
              용량 한도에 도달하면 기록을 중단하며 기존 기록은 보존합니다.
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
  const raw = rows.find((r) => r.kind === "recording_raw")?.data.items || [];
  return (
    <details className="recording-replay">
      <summary>녹화 범위 · 원본 자료</summary>
      {settings ? (
        <p>
          {!settings.active && "이 시점은 녹화 일시정지 구간입니다. "}미녹화
          항목:{" "}
          {Object.keys(labels)
            .filter((key) => !settings.enabled[key])
            .map((key) => labels[key])
            .join(", ") || "없음"}
        </p>
      ) : (
        <p>이전 형식의 기록: 녹화 항목 설정 정보가 없습니다.</p>
      )}
      {heavy.map((key) => {
        const items = raw.filter((item: Data) => item.category === key);
        return (
          <div key={key}>
            <strong>{labels[key]}: </strong>
            {items.length ? (
              <>
                {items.reduce((sum: number, item: Data) => sum + item.count, 0)}
                건 ·{" "}
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
                {item.topic} · {item.message_type}
              </small>
            ))}
          </div>
        );
      })}
      <p>
        원본은 전체 세션 내보내기에 포함됩니다. 3D 원본과 카메라 영상의 직접
        재생 뷰어는 아직 제공하지 않습니다.
      </p>
    </details>
  );
}
