import { useEffect, useRef } from "react";
import { useReactFlow, useStore } from "@xyflow/react";

export function FitGraphToViewport({ topology = "" }: { topology?: string }) {
  const { fitView } = useReactFlow();
  const fit = useRef(fitView);
  fit.current = fitView;
  const width = useStore((state) => state.width);
  const height = useStore((state) => state.height);
  useEffect(() => {
    const timer = setTimeout(
      () => void fit.current({ padding: 0.18, maxZoom: 1, duration: 0 }),
      100,
    );
    return () => clearTimeout(timer);
  }, [width, height, topology]);
  return null;
}
