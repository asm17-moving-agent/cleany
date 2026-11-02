import { useEffect, useRef, useState } from "react";
import { Square } from "lucide-react";
import { Badge } from "@/components/ui/badge";
import { Button } from "@/components/ui/button";
import { Card, CardContent } from "@/components/ui/card";
import { Slider } from "@/components/ui/slider";
import { Separator } from "@/components/ui/separator";
import { MappingPanel } from "./MappingPanel";
import { mountRemote } from "./controller.js";

function Readout({ label, id }: { label: string; id: string }) {
  return (
    <div className="min-w-0 space-y-2">
      <div className="text-[11px] text-muted-foreground">{label}</div>
      <strong id={id} className="block text-[11px] font-medium tabular-nums">
        —
      </strong>
    </div>
  );
}

function MotionInput({ rotation = false }: { rotation?: boolean }) {
  return (
    <div className="stick-wrap">
      <div
        id={rotation ? "turnStick" : "moveStick"}
        className={`stick ${rotation ? "rotation" : "movement"}`}
        role="application"
        tabIndex={0}
        aria-label={rotation ? "회전 슬라이더" : "이동 조이스틱"}
        aria-disabled="true"
      >
        {!rotation && (
          <>
            <span className="stick-label north">↑</span>
            <span className="stick-label south">↓</span>
          </>
        )}
        <span className="stick-label west">{rotation ? "↶" : "←"}</span>
        <span className="stick-label east">{rotation ? "↷" : "→"}</span>
        <div className="knob" />
      </div>
      <span>{rotation ? "회전 · 좌우" : "이동 · 앞뒤 / 좌우"}</span>
    </div>
  );
}

export default function App() {
  const [speed, setSpeed] = useState(0.5);
  const speedRef = useRef(speed);
  const [maxSpeed, setMaxSpeed] = useState(10);
  const [speedDisabled, setSpeedDisabled] = useState(false);
  useEffect(
    () =>
      mountRemote({
        getSpeed: () => speedRef.current,
        onSpeedDisabled: setSpeedDisabled,
        onMaxSpeed: setMaxSpeed,
      }),
    [],
  );
  return (
    <main className="mx-auto max-w-[460px] space-y-6 px-5 py-6">
      <header className="flex items-center justify-between">
        <div className="text-xl font-bold tracking-tight">cleany.</div>
        <Badge variant="outline" className="font-normal text-muted-foreground">
          MOBILE REMOTE
        </Badge>
      </header>
      <div
        id="mock"
        hidden
        className="rounded-lg border border-border bg-muted p-3 text-xs"
      >
        미리보기 · 실제 로봇에 연결하지 않습니다
      </div>
      <section
        aria-label="휠 속도"
        className="grid grid-cols-2 gap-4 border-t border-border pt-4"
      >
        <Readout label="전송 목표 · 휠 rad/s" id="target" />
        <Readout label="측정 속도 · 휠 rad/s" id="velocity" />
      </section>
      <section className="space-y-4" aria-label="속도 설정">
        <div className="flex items-center justify-between text-sm">
          <label id="speed-label">최대 휠 속도</label>
          <strong className="font-medium tabular-nums">
            {speed.toFixed(1)} rad/s
          </strong>
        </div>
        <Slider
          id="speed"
          aria-label="최대 휠 속도"
          min={0.1}
          max={maxSpeed}
          step={0.1}
          value={[speed]}
          disabled={speedDisabled}
          onValueChange={([value]) => {
            speedRef.current = value;
            setSpeed(value);
          }}
        />
        <div className="flex justify-between gap-2 text-[11px] text-muted-foreground">
          <span>약 {((speed * 60) / (2 * Math.PI)).toFixed(1)} RPM</span>
          <span>
            웹 최대 {maxSpeed.toFixed(1)} rad/s · 약{" "}
            {Math.round((maxSpeed * 60) / (2 * Math.PI))} RPM
          </span>
        </div>
      </section>
      <section className="joystick-area" aria-label="조이스틱 조종">
        <MotionInput />
        <MotionInput rotation />
      </section>
      <Button
        id="stop"
        type="button"
        aria-label="전체 정지"
        className="h-14 w-full touch-none select-none gap-2 text-base font-bold"
      >
        <Square aria-hidden="true" className="size-4 fill-current" />
        STOP
      </Button>
      <section className="grid grid-cols-2 gap-4 border-t border-border pt-4">
        <Readout label="모터 출력 · PWM %" id="pwm" />
        <Readout label="ESP32 최대 휠 속도" id="mcuLimit" />
      </section>
      <Card className="gap-0 py-0">
        <CardContent className="space-y-2 p-4" aria-live="polite">
          <div className="flex items-center gap-2 text-sm font-medium">
            <span id="dot" className="dot" />
            <span id="connection">ESP32 연결 확인 중</span>
          </div>
          <p id="message" className="text-xs text-muted-foreground">
            노트북의 USB 연결 상태를 확인합니다
          </p>
          <Separator />
          <p id="feedbackAge" className="text-[11px] text-muted-foreground">
            ESP 상태 수신 대기
          </p>
          <p id="stopState" className="text-xs text-muted-foreground">
            ESP 출력 정지 · 확인 중
          </p>
          <p id="inputState" className="text-xs">
            브라우저 입력 · 대기
          </p>
        </CardContent>
      </Card>
      <MappingPanel held={speedDisabled} />
      <div className="space-y-2 text-[11px] text-muted-foreground">
        <p id="lastStop">최근 정지 사유 · 없음</p>
        <p id="lastFault" hidden />
      </div>
    </main>
  );
}
