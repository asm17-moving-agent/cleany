import { Check, RefreshCw, SkipForward } from "lucide-react";
import { Button } from "@/components/ui/button";
import { type Data } from "@/lib/monitor";
import "./task-proposal.css";

export function TaskProposal({
  proposal,
  reason,
  submit,
}: {
  proposal?: Data | null;
  reason: (action: string) => string;
  submit: (action: string, args?: Data) => Promise<void>;
}) {
  const remaining = proposal
    ? Math.max(0, Math.ceil(proposal.expires_at - Date.now() / 1000))
    : 0;
  const ready = proposal?.state === "READY" && remaining > 0;
  const target = proposal?.target;
  const unavailable = !ready ? "새 제안을 받아 확인하세요." : "";
  const approvalReason = reason("approve") || unavailable;
  return (
    <section
      className="task-proposal"
      aria-label="다음 작업 제안"
      data-empty={!proposal}
    >
      <div className="task-proposal-heading">
        <div>
          <h3>다음 작업 제안</h3>
          {proposal && (
            <p>
              한 번 승인하면 한 물체만 실행합니다. 다음 작업은 다시 승인을
              기다립니다.
            </p>
          )}
        </div>
        <span className="task-proposal-badge">
          {proposal ? "규칙 기반 Planner" : "제안 대기"}
        </span>
      </div>
      {!proposal ? (
        <p>‘다음 작업 제안받기’를 눌러 현재 장면과 추천 대상을 확인하세요.</p>
      ) : (
        <>
          <div className="task-proposal-content">
            {proposal.image_available && (
              <img
                src={`/api/control/proposals/${encodeURIComponent(proposal.id)}/image`}
                alt={
                  target
                    ? `추천 물체 ${target.object_id}번 ${target.label}가 초록색 테두리로 표시된 관측`
                    : "제안에 사용한 관측"
                }
              />
            )}
            <div className="task-proposal-description">
              <strong>
                {target
                  ? `${target.object_id}번 · ${target.label}`
                  : "실행 가능한 대상 없음"}
              </strong>
              {target && (
                <p>
                  거리 {Number(target.distance_m).toFixed(2)} m · 검출 신뢰도{" "}
                  {Math.round(target.confidence * 100)}%
                </p>
              )}
              <p className="task-proposal-action">
                {proposal.operation_mode === "pick_return_release"
                  ? "잡기 → 원래 위치에 놓기 → 팔 복귀"
                  : target
                    ? `${proposal.skill_name === "collect_lost_item" ? "분실물" : "쓰레기"} 수거 → ${proposal.destination_id}`
                    : ""}
              </p>
              <p>{proposal.reason}</p>
              <p className="task-proposal-state" role="status">
                {ready
                  ? `승인 대기 · ${remaining}초 남음`
                  : proposal.state === "CONSUMED"
                    ? "이미 처리한 제안 · 실행 결과를 확인하세요."
                    : proposal.state === "NO_TARGET"
                      ? "대상이 없거나 조건을 충족하지 못했습니다. 장면을 확인하세요."
                      : "제안 만료 · 최신 장면으로 다시 제안받으세요."}
              </p>
              {proposal.last_result && (
                <p>
                  이전 실행: {proposal.last_result.status}
                  {proposal.last_result.placement_state === "ASSUMED" &&
                    " · 배치는 추정 상태"}
                  {proposal.last_result.message &&
                    ` · ${proposal.last_result.message}`}
                </p>
              )}
            </div>
          </div>
          <div className="task-proposal-buttons">
            <span title={approvalReason}>
              <Button
                disabled={!!approvalReason}
                onClick={() =>
                  void submit("approve", { proposal_id: proposal.id })
                }
              >
                <Check />
                승인하여 1회 실행
              </Button>
            </span>
            <span title={reason("skip") || unavailable}>
              <Button
                variant="outline"
                disabled={!!reason("skip") || !ready}
                onClick={() =>
                  void submit("skip", { proposal_id: proposal.id })
                }
              >
                <SkipForward />이 물체 건너뛰기
              </Button>
            </span>
            <span title={reason("propose")}>
              <Button
                variant="outline"
                disabled={!!reason("propose")}
                onClick={() => void submit("propose")}
              >
                <RefreshCw />
                다시 제안받기
              </Button>
            </span>
          </div>
          {ready && (
            <p className="task-proposal-hint">
              승인한 이름과 분류를 유지하고, 최신 영상에서 같은 물체인지 확인합니다.
              물체가 이동했거나 확인되지 않거나 승인 시간이 지나면 실행을 중단합니다.
            </p>
          )}
        </>
      )}
    </section>
  );
}
