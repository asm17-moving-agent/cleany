import { useEffect, useRef, useState } from "react";
import { Button } from "@/components/ui/button";
import { Card, CardContent } from "@/components/ui/card";
import { Badge } from "@/components/ui/badge";

type MappingState = { phase: string; message: string; request_id?: string; generation?: number };
const busy = new Set(["queued", "stopping", "waiting_usb", "starting"]);

export function MappingPanel({ held }: { held: boolean }) {
  const [state, setState] = useState<MappingState>({ phase: "unavailable", message: "매핑 관리자 연결 대기" });
  const [error, setError] = useState("");
  const [sending, setSending] = useState(false);
  const requestId = useRef<string | null>(null);
  const token = new URLSearchParams(location.hash.slice(1)).get("token") || "";
  useEffect(() => {
    let disposed = false;
    let pending = false;
    const poll = async () => {
      if (pending) return;
      pending = true;
      try {
        const response = await fetch("/api/mapping/status", { headers: { "X-Control-Token": token }, signal: AbortSignal.timeout(1000) });
        if (!response.ok) throw new Error("매핑 상태를 읽지 못했습니다");
        const value: MappingState = await response.json();
        if (!disposed) {
          setState(value);
          if (value.request_id === requestId.current) requestId.current = null;
        }
      } catch { if (!disposed) setState({ phase: "unavailable", message: "매핑 관리자 연결 대기" }); }
      finally { pending = false; }
    };
    void poll();
    const timer = setInterval(poll, 750);
    return () => { disposed = true; clearInterval(timer); };
  }, [token]);
  const reset = async () => {
    setSending(true); setError("");
    requestId.current ||= Array.from(crypto.getRandomValues(new Uint8Array(16)), (v) => v.toString(16).padStart(2, "0")).join("");
    try {
      const response = await fetch("/api/reset", { method: "POST", headers: { "Content-Type": "application/json", "X-Control-Token": token }, body: JSON.stringify({ request_id: requestId.current }), signal: AbortSignal.timeout(3000) });
      const value = await response.json();
      if (!response.ok) throw new Error(value.error || "초기화 요청 실패");
      setState({ phase: "queued", message: "지도 초기화 요청 접수" });
    } catch (error) { setError(error instanceof Error ? error.message : "초기화 응답 지연"); }
    finally { setSending(false); }
  };
  return <Card className="gap-0 py-0"><CardContent className="space-y-3 p-4">
    <div className="flex items-center justify-between"><span className="text-sm font-medium">지도</span><Badge variant="outline">{state.phase === "running" ? "매핑 중" : state.phase === "failed" ? "확인 필요" : busy.has(state.phase) ? "준비 중" : "대기"}</Badge></div>
    <p className="text-xs text-muted-foreground" aria-live="polite">{state.message}</p>
    <Button type="button" variant="outline" className="w-full" disabled={held || sending || busy.has(state.phase) || state.phase === "unavailable"} onClick={reset}>지도 초기화</Button>
    {error && <p role="alert" className="text-xs">{error}</p>}
  </CardContent></Card>;
}
