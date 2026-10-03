import { ArrowUpRight } from "lucide-react";
import { Button } from "@/components/ui/button";

export function FoxglovePanel() {
  return (
    <div className="integration-link">
      <Button variant="link" size="sm" asChild>
        <a
          href="https://docs.foxglove.dev/docs/visualization/connecting/live"
          target="_blank"
          rel="noreferrer"
        >
          Foxglove 연결 가이드
          <ArrowUpRight size={14} />
        </a>
      </Button>
    </div>
  );
}
