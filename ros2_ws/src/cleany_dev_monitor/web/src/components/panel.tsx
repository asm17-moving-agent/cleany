import type { ReactNode } from "react";
import { Card, CardHeader, CardTitle, CardContent } from "@/components/ui/card";

export function Panel({
  title,
  aside,
  children,
  className = "",
}: {
  title: string;
  aside?: ReactNode;
  children: ReactNode;
  className?: string;
}) {
  return (
    <Card className={"panel " + className}>
      <CardHeader className="panel-title">
        <CardTitle>
          <h2>{title}</h2>
        </CardTitle>
        {aside}
      </CardHeader>
      <CardContent className="panel-content">{children}</CardContent>
    </Card>
  );
}
