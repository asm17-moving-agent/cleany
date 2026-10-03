import { useEffect, useState } from "react";
import { Camera, VideoOff } from "lucide-react";
import { Card, CardContent } from "@/components/ui/card";
import { Badge } from "@/components/ui/badge";
import { Button } from "@/components/ui/button";

type Topic = { name: string; types: string[] };
const imageTypes = new Set([
  "sensor_msgs/msg/Image",
  "sensor_msgs/msg/CompressedImage",
]);

export function CameraView({
  topics,
  replay,
}: {
  topics: Topic[];
  replay: boolean;
}) {
  const cameras = topics.filter(
    (t) => t.types.length === 1 && imageTypes.has(t.types[0]),
  );
  const [topic, setTopic] = useState("");
  const [paused, setPaused] = useState(false);
  const [visible, setVisible] = useState(!document.hidden);
  const [frame, setFrame] = useState<{ url: string; size: string } | null>(
    null,
  );
  const [status, setStatus] = useState("토픽 선택");
  const known = cameras.some((t) => t.name === topic);
  useEffect(() => {
    const update = () => setVisible(!document.hidden);
    document.addEventListener("visibilitychange", update);
    return () => document.removeEventListener("visibilitychange", update);
  }, []);
  useEffect(() => {
    setFrame(null);
    if (!topic || !known || replay || paused || !visible) return;
    let stopped = false;
    let url = "";
    let expires = 0;
    let timer: ReturnType<typeof setTimeout>;
    let controller: AbortController;
    const clear = (message: string) => {
      if (url) URL.revokeObjectURL(url);
      url = "";
      setFrame(null);
      setStatus(message);
    };
    const poll = async () => {
      controller = new AbortController();
      const timeout = setTimeout(() => controller.abort(), 1500);
      try {
        const response = await fetch(
          `/api/camera?topic=${encodeURIComponent(topic)}`,
          { signal: controller.signal, cache: "no-store" },
        );
        if (stopped) return;
        if (response.status === 202) {
          const data = await response.json();
          if (!stopped)
            clear(data.state === "stale" ? "수신 지연" : "수신 대기");
        } else if (
          response.status === 200 &&
          response.headers.get("content-type")?.startsWith("image/jpeg")
        ) {
          const blob = await response.blob();
          if (stopped) return;
          const age = Number(response.headers.get("X-Frame-Age") ?? 2);
          if (!Number.isFinite(age) || age >= 2) {
            clear("수신 지연");
            return;
          }
          if (url) URL.revokeObjectURL(url);
          url = URL.createObjectURL(blob);
          expires = performance.now() + (2 - age) * 1000;
          setFrame({
            url,
            size: `${response.headers.get("X-Image-Width")} × ${response.headers.get("X-Image-Height")}`,
          });
          setStatus("LIVE");
        } else {
          const message =
            response.status === 422
              ? (await response.json()).message
              : response.status === 429
                ? "구독 한도 초과"
                : "연결 실패";
          if (!stopped) clear(message);
        }
      } catch {
        if (!stopped) clear("연결 끊김");
      } finally {
        clearTimeout(timeout);
        if (!stopped) timer = setTimeout(poll, 200);
      }
    };
    setStatus("수신 대기");
    void poll();
    const watchdog = setInterval(() => {
      if (url && performance.now() > expires) clear("수신 지연");
    }, 200);
    return () => {
      stopped = true;
      controller?.abort();
      clearTimeout(timer);
      clearInterval(watchdog);
      if (url) URL.revokeObjectURL(url);
    };
  }, [topic, known, replay, paused, visible]);
  const active = !!topic && known && !replay && !paused && visible;
  const label = replay
    ? "영상 재생 미지원"
    : !topic
      ? "토픽 선택"
      : !known
        ? "토픽 없음"
        : paused
          ? "일시정지"
          : !visible
            ? "일시정지"
            : status;
  return (
    <Card className="panel">
      <CardContent className="camera-content">
        <div className="camera-toolbar">
          <Camera size={18} />
          <select
            aria-label="카메라 토픽"
            value={topic}
            disabled={replay}
            onChange={(e) => {
              setFrame(null);
              setTopic(e.target.value);
              setPaused(false);
            }}
          >
            <option value="">
              {cameras.length ? "토픽 선택" : "카메라 토픽 없음"}
            </option>
            {cameras.map((t) => (
              <option key={t.name} value={t.name}>
                {t.name}
              </option>
            ))}
          </select>
          <Badge
            variant="secondary"
            className={active && frame ? "badge info" : "badge"}
          >
            {label}
          </Badge>
          {active && frame && <span className="camera-size">{frame.size}</span>}
          <Button
            variant="outline"
            size="sm"
            disabled={!topic || replay}
            onClick={() => setPaused(!paused)}
          >
            {paused ? "재개" : "일시정지"}
          </Button>
        </div>
        <div className="camera-viewport">
          {active && frame ? (
            <img
              src={frame.url}
              alt={topic}
              onError={() => {
                setFrame(null);
                setStatus("영상 디코딩 실패");
              }}
            />
          ) : (
            <div className="empty">
              <VideoOff size={28} />
              <p>{label}</p>
            </div>
          )}
        </div>
      </CardContent>
    </Card>
  );
}
