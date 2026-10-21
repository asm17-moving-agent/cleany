export type Data = Record<string, any>;
export type Row = {
  seq: number;
  kind: string;
  key: string;
  data: Data;
  received_at: number;
  ros_time: number | null;
};
export const time = (t: number) =>
  new Date(t * 1000).toLocaleTimeString("ko-KR", { hour12: false });
