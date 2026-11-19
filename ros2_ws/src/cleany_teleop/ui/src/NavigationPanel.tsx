import { useEffect, useRef, useState } from 'react';
import { Button } from '@/components/ui/button';
import { Card, CardContent } from '@/components/ui/card';
type MapData={map_id:string;width:number;height:number;resolution:number;origin:number[];cells:number[]};
type Localization={state?:string;reason?:string;phase?:string;recovery_reason?:string;score?:{inlier_ratio:number;mean_distance:number}};
type State={localization?:Localization;slip?:{state?:string};traction_recovery?:{phase?:string;reason?:string};map_id?:string;phase:string;ready:boolean;reason:string;pose?:number[];path?:number[][];goal_id?:string;preview_request_id?:string;preview_phase?:string;preview_path?:number[][];preview_error?:string;preview_target?:number[]};
function LocalizationStatus({value}:{value?:Localization}) {
 if(!value)return null;
 const recovering=['waiting_for_stop','searching','verifying_candidate','verifying_amcl'].includes(value.phase||'');
 const label=value.state==='available'?'위치 정합 정상':value.state==='degraded'?'위치 정합 회복 중 · 감속 주행':recovering?'정지 후 위치 복구 중':value.phase==='failed'?'위치 복구 실패 · 초기 위치 확인 필요':'위치 정합 확인 중';
 const score=value.score;
 return <p role="status" className="text-xs">{label}{score&&Number.isFinite(score.inlier_ratio)?` · 일치 ${(score.inlier_ratio*100).toFixed(0)}%`:''}</p>;
}
// Unlike randomUUID, getRandomValues is available on the phone's HTTP LAN URL.
const requestId=()=>Array.from(crypto.getRandomValues(new Uint8Array(16)),b=>b.toString(16).padStart(2,'0')).join('');
export function NavigationPanel(){
 const [map,setMap]=useState<MapData|null>(null),[state,setState]=useState<State>({phase:'unavailable',ready:false,reason:'주행 모드 대기'}),[goal,setGoal]=useState<number[]|null>(null),[initial,setInitial]=useState(false),[showGlobalPath,setShowGlobalPath]=useState(true),[error,setError]=useState('');
 const canvas=useRef<HTMLCanvasElement>(null),start=useRef<number[]|null>(null),previewId=useRef<string|null>(null);
 const token=new URLSearchParams(location.hash.slice(1)).get('token')||'';
 const call=async(route:string,data?:object)=>{const r=await fetch('/api/navigation/'+route,{method:data?'POST':'GET',headers:{'X-Control-Token':token,...(data?{'Content-Type':'application/json'}:{})},body:data?JSON.stringify(data):undefined,signal:AbortSignal.timeout(1500)});const d=await r.json();if(!r.ok)throw Error(d.error||'요청 실패');return d;};
 useEffect(()=>{let done=false,pending=false; const poll=async()=>{if(pending)return;pending=true;try{const s:State=await call('status');if(done)return;setState(s);if(s.goal_id&&['active','preparing'].includes(s.phase))await call('heartbeat',{goal_id:s.goal_id});if(s.map_id&&s.map_id!==map?.map_id){const m=await call('map');if(!done){setMap(m);setGoal(null);}}}catch{if(!done)setState({phase:'unavailable',ready:false,reason:'주행 서버 연결 대기'});}finally{pending=false;}};void poll();const timer=setInterval(poll,500);return()=>{done=true;clearInterval(timer)};},[map?.map_id]);
 const moving=['preparing','active','stopping'].includes(state.phase);
 const previewReady=!!goal&&state.preview_request_id===previewId.current&&state.preview_phase==='ready';
 useEffect(()=>{
  if(!initial&&state.preview_request_id===previewId.current&&state.preview_phase==='ready'&&state.preview_target){
   const target=state.preview_target;
   setGoal(previous=>previous&&previous.every((value,index)=>value===target[index])?previous:target);
  }
 },[initial,state.preview_request_id,state.preview_phase,state.preview_target]);
 const displayedPath=initial?[]:moving?(state.path||[]):goal?(previewReady?(state.preview_path||[]):[]):(state.path||[]);
 const pickGoal=async(g:number[])=>{
  setGoal(g);setError('');
  if(initial)return;
  const id=requestId();previewId.current=id;
  setState(s=>({...s,preview_request_id:id,preview_phase:'planning',preview_path:[],preview_error:''}));
  try{const result:State=await call('preview',{request_id:id,map_id:map?.map_id,x:g[0],y:g[1],yaw:g[2]});if(previewId.current===id)setState(result);}
  catch(e){if(previewId.current===id){setState(s=>({...s,preview_request_id:id,preview_phase:'failed',preview_path:[]}));setError(e instanceof Error?e.message:'경로 계산 실패');}}
 };
 const screen=(p:number[])=>{if(!map)return[0,0];const dx=p[0]-map.origin[0],dy=p[1]-map.origin[1],c=Math.cos(map.origin[2]),s=Math.sin(map.origin[2]);return[(c*dx+s*dy)/map.resolution,map.height-(-s*dx+c*dy)/map.resolution];};
 useEffect(()=>{const c=canvas.current;if(!map||!c)return;c.width=map.width;c.height=map.height;const ctx=c.getContext('2d')!;const im=ctx.createImageData(map.width,map.height);map.cells.forEach((v,i)=>{const y=Math.floor(i/map.width),x=i%map.width,j=((map.height-1-y)*map.width+x)*4;const val=v<0?180:255-Math.round(v*2.55);im.data.set([val,val,val,255],j)});ctx.putImageData(im,0,0);if(showGlobalPath&&displayedPath.length){ctx.lineJoin='round';ctx.lineCap='round';ctx.beginPath();displayedPath.forEach((p,i)=>{const[x,y]=screen(p);if(i)ctx.lineTo(x,y);else ctx.moveTo(x,y)});ctx.lineWidth=Math.max(5,map.width/110);ctx.strokeStyle='#ffffff';ctx.stroke();ctx.lineWidth=Math.max(3,map.width/180);ctx.strokeStyle='#2563eb';ctx.stroke();}for(const [p,color] of [[state.pose,'#16a34a'],[goal,'#dc2626']] as const){if(!p)continue;const[x,y]=screen(p);ctx.strokeStyle=color;ctx.fillStyle=color;ctx.beginPath();ctx.arc(x,y,5,0,Math.PI*2);ctx.fill();ctx.beginPath();ctx.moveTo(x,y);const a=p[2]-map.origin[2];ctx.lineTo(x+18*Math.cos(a),y-18*Math.sin(a));ctx.stroke();}},[map,state.pose,displayedPath,goal,showGlobalPath]);
 const point=(e:React.PointerEvent<HTMLCanvasElement>)=>{const r=e.currentTarget.getBoundingClientRect(),m=map!;const u=(e.clientX-r.left)*m.width/r.width,v=m.height-(e.clientY-r.top)*m.height/r.height,c=Math.cos(m.origin[2]),s=Math.sin(m.origin[2]);return[m.origin[0]+m.resolution*(c*u-s*v),m.origin[1]+m.resolution*(s*u+c*v)];};
 const send=async(op:string)=>{setError('');const target=op==='goal'&&previewReady?state.preview_target:goal;try{await call(op,{map_id:map?.map_id,x:target?.[0],y:target?.[1],yaw:target?.[2],request_id:requestId()});}catch(e){setError(e instanceof Error?e.message:'요청 실패')}};
 return <Card><CardContent className="space-y-3 p-4"><h2 className="text-sm font-semibold">지도에서 이동</h2><LocalizationStatus value={state.localization}/>{state.traction_recovery?.phase&&<p role="status" className="text-xs">{({warming_up:'슬립 감지 준비 중',normal:'슬립 감지 중',braking:'이동 불일치 감지 · 감속 정지 중',longitudinal_retry:'슬립 복구 · 전후진과 회전으로 재시도 중',failed:'슬립 복구 중단 · 정지',succeeded:'이동 완료',idle:'대기'} as Record<string,string>)[state.traction_recovery.phase]||state.traction_recovery.phase} {state.slip?.state==='unknown'?'· 측정 확인 중':state.slip?.state==='suspect'?'· 슬립 의심':''}</p>}<p className="text-xs">{state.reason} · {({idle:'대기',preparing:'준비 중',active:'이동 중',stopping:'정지 확인 중',succeeded:'도착',cancelled:'취소',failed:'실패',unavailable:'연결 대기'} as Record<string,string>)[state.phase]||state.phase}</p>{map?<canvas ref={canvas} className="w-full touch-none border" aria-label="지도에서 목표를 선택하고 드래그로 방향 지정" onPointerDown={e=>{if(moving)return;start.current=point(e);e.currentTarget.setPointerCapture(e.pointerId)}} onPointerUp={e=>{if(!start.current)return;const p=point(e),s=start.current;void pickGoal([s[0],s[1],Math.hypot(p[0]-s[0],p[1]-s[1])>.1?Math.atan2(p[1]-s[1],p[0]-s[0]):state.pose?.[2]||0]);start.current=null}} onPointerCancel={()=>{start.current=null}}/>:<p className="text-xs text-muted-foreground">지도 수신 대기</p>}<p className="text-xs">녹색: 로봇 · 빨강: 목표 · 파랑: Nav2 전역 경로</p><label className="flex items-center gap-2 text-xs"><input type="checkbox" checked={showGlobalPath} onChange={e=>setShowGlobalPath(e.target.checked)}/>Nav2 전역 경로 표시 <span className="text-muted-foreground">{displayedPath.length?`${displayedPath.length}개 경로점`:goal&&!initial&&state.preview_phase==='planning'?'경로 계산 중…':'경로 생성 대기'}</span></label>{state.preview_error&&!initial&&<p role="status" className="text-xs">{state.preview_error}</p>}{goal&&<p className="text-xs">{initial?'선택':'이동 목표'}: ({goal[0].toFixed(2)}, {goal[1].toFixed(2)}) m · 방향 {(goal[2]*180/Math.PI).toFixed(0)}°</p>}{previewReady&&!initial&&<p className="text-xs text-muted-foreground">표시된 빨간 목표가 실제 이동 목표입니다. 지도 격자와 가능한 방향에 맞춰집니다.</p>}<label className="flex gap-2 text-xs"><input type="checkbox" checked={initial} onChange={e=>{setInitial(e.target.checked);setGoal(null);previewId.current=null;}}/>초기 위치 설정 모드</label><div className="flex gap-2"><Button disabled={!goal||moving||(!initial&&(!state.ready||!previewReady))} onClick={()=>void send(initial?'initial-pose':'goal')}>{initial?'현재 위치 지정':'이동'}</Button><Button variant="outline" onClick={()=>void send('cancel')}>이동 취소 / 정지</Button></div>{error&&<p role="alert" className="text-xs">{error}</p>}</CardContent></Card>;
}
