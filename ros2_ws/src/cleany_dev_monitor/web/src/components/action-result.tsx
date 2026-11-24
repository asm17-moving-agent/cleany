import { useState } from "react";
import { Clock3 } from "lucide-react";
import type { Data } from "@/lib/monitor";

export const jobActive = (job?: Data) =>
  job?.state === "RUNNING" || job?.state === "PENDING";

export function JobTime({ job, now }: { job?: Data; now: number }) {
  const end =
    job?.finished_at ?? (jobActive(job) && now > 0 ? now / 1000 : undefined);
  const measured =
    job &&
    Number.isFinite(job.created_at) &&
    Number.isFinite(end) &&
    end >= job.created_at;
  const value = measured
    ? (end - job.created_at).toFixed(1) + "초"
    : job
      ? "시간 미확인"
      : "최근 기록 없음";
  return (
    <small className="job-time" title="Jetson 요청 접수부터 결과 저장까지">
      <Clock3 size={13} aria-hidden="true" />
      {jobActive(job) ? "진행 " : ""}
      {value}
    </small>
  );
}

const names: Record<string, string> = {
  "computer mouse": "마우스",
  pen: "펜",
  box: "상자",
  battery: "배터리",
};
export function InspectionResult({
  job,
  authenticated,
}: {
  job?: Data;
  authenticated: boolean;
}) {
  const [selected, select] = useState<number>();
  const [failedImage, setFailedImage] = useState(false);
  if (!job || jobActive(job))
    return (
      <p role="status">카메라 촬영과 물체 검출 결과를 기다리는 중입니다.</p>
    );
  if (job.state !== "SUCCEEDED")
    return (
      <p role="status">{job.message || "장면 인식 결과를 받지 못했습니다."}</p>
    );
  const result = job.result;
  if (!Array.isArray(result?.objects))
    return <p>검출 목록을 읽을 수 없습니다.</p>;
  const objects: Data[] = result.objects;
  const chosen = objects.find((o) => o.object_id === selected);
  const image = result.image_available && authenticated && !failedImage;
  const width = result.source_width,
    height = result.source_height;
  const box = chosen?.bbox;
  return (
    <section className="inspection-result" aria-label="장면 인식 결과">
      <div className="inspection-heading">
        <h4>검출 물체 {objects.length}개</h4>
        {Number.isFinite(result.captured_at_ns) && (
          <time>
            {new Date(result.captured_at_ns / 1e6).toLocaleTimeString("ko-KR", {
              hour12: false,
            })}{" "}
            촬영
          </time>
        )}
      </div>
      {image ? (
        <div className="inspection-image">
          <img
            src={`/api/control/proposals/${encodeURIComponent(job.id)}/image`}
            alt="촬영 당시 물체 번호와 검출 영역"
            onError={() => setFailedImage(true)}
          />
          {width > 0 &&
            height > 0 &&
            Array.isArray(box) &&
            box.every(Number.isFinite) && (
              <span
                className="inspection-bbox"
                style={{
                  left: (box[0] / width) * 100 + "%",
                  top: (box[1] / height) * 100 + "%",
                  width: ((box[2] - box[0]) / width) * 100 + "%",
                  height: ((box[3] - box[1]) / height) * 100 + "%",
                }}
              />
            )}
        </div>
      ) : (
        <p>
          {result.image_available && !authenticated
            ? "조작 권한을 연결하면 촬영 이미지를 볼 수 있습니다."
            : "이 촬영의 검출 이미지가 없습니다."}
        </p>
      )}
      {result.seconds && <p>장면 인식 {result.seconds.inspection.toFixed(1)}초 · 이미지 처리 {result.seconds.capture.toFixed(1)}초</p>}
      {objects.length > 0 ? (
        <div className="detected-list">
          {objects.map((o) => (
            <button
              key={o.object_id}
              type="button"
              aria-pressed={selected === o.object_id}
              onClick={() =>
                select(selected === o.object_id ? undefined : o.object_id)
              }
            >
              <b>#{o.object_id}</b> {names[o.label] || o.label}{" "}
              <small>
                {Number.isFinite(o.confidence)
                  ? Math.round(o.confidence * 100) + "%"
                  : "—"}
              </small>
            </button>
          ))}
        </div>
      ) : (
        <p>이 장면에서 검출된 물체가 없습니다.</p>
      )}
      {chosen && (
        <p className="inspection-position">
          {chosen.position_valid &&
          Array.isArray(chosen.position) &&
          chosen.position.every(Number.isFinite)
            ? `#${chosen.object_id} 대표점 · ${result.representative_frame} · X ${(chosen.position[0] * 1000).toFixed(0)} / Y ${(chosen.position[1] * 1000).toFixed(0)} / Z ${(chosen.position[2] * 1000).toFixed(0)} mm`
            : `#${chosen.object_id} · 유효한 3D 대표점이 없습니다.`}
        </p>
      )}
    </section>
  );
}
