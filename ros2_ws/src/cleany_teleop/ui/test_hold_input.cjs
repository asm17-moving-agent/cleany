const {test}=require('node:test');
const assert=require('node:assert/strict');
const {HoldInput,stickAxes,HeldUpdates}=require('./hold_input.cjs');
function fixture(){
 const events=[];
 const input=new HoldInput({start:value=>events.push(['start',value]),change:value=>events.push(['change',value]),stop:()=>events.push(['stop'])});
 return {input,events};
}
test('touch and mouse release stop synchronously, including duplicate release',()=>{
 const {input,events}=fixture();input.press('forward');input.release();
 assert.equal(input.active,null);assert.deepEqual(events,[['start','forward'],['stop']]);
 input.release();input.cancel();assert.equal(events.length,2);
});
test('unchanged held input and direction updates stay in one session',()=>{
 const {input,events}=fixture();input.press('forward');
 for(let i=0;i<100;i++)input.press('forward');
 input.press('left');assert.deepEqual(events,[['start','forward'],['change','left']]);
});
test('new explicit press after release starts a new session',()=>{
 const {input,events}=fixture();input.press('forward');input.release();input.press('forward');
 assert.deepEqual(events,[['start','forward'],['stop'],['start','forward']]);
});
test('safety cancellation and STOP work immediately even while idle',()=>{
 const {input,events}=fixture();input.press('forward');input.cancel();input.cancel(true);
 assert.deepEqual(events,[['start','forward'],['stop'],['stop']]);
});
test('interruption preserves a stationary touch and recovery starts a fresh session',()=>{
 const events=[];
 const input=new HoldInput({start:v=>events.push(['press',v]),change:()=>{},stop:preserve=>events.push(['stop',preserve])});
 input.press('forward');input.suspend();input.suspend();
 assert.equal(input.active,'forward');assert.equal(input.suspended,true);
 assert.deepEqual(events,[['press','forward'],['stop',true]]);
 assert.equal(input.resume(),true);assert.equal(input.resume(),false);
 assert.deepEqual(events,[['press','forward'],['stop',true],['press','forward']]);
});
test('recovery uses the latest joystick position after dragging while disconnected',()=>{
 const {input,events}=fixture();input.press('forward');input.suspend();input.press('left');
 assert.equal(input.suspended,true);assert.equal(events.filter(e=>e[0]==='start').length,1);
 input.resume();assert.deepEqual(events.at(-1),['start','left']);
});
test('release during an outage cancels automatic recovery and cannot revive on reconnect',()=>{
 const {input,events}=fixture();input.press('forward');input.suspend();input.release();
 assert.equal(input.active,null);assert.equal(input.suspended,false);assert.equal(input.resume(),false);
 assert.equal(events.filter(e=>e[0]==='start').length,1);
});
test('STOP, pointer cancellation and page loss invalidate held recovery immediately',()=>{
 for(const force of [false,true]){
  const {input,events}=fixture();input.press('forward');input.suspend();input.cancel(force);
  assert.equal(input.resume(),false);assert.equal(input.active,null);
  assert.equal(events.filter(e=>e[0]==='start').length,1);
 }
});
test('repeated disconnect and recovery do not require a new finger press',()=>{
 const {input,events}=fixture();input.press('forward');
 for(let i=0;i<10;i++){input.suspend();input.resume()}
 assert.equal(events.filter(e=>e[0]==='start').length,11);
 input.release();assert.equal(input.resume(),false);
});
test('waiting for initial connection does not send an unrelated remote STOP',()=>{
 const {input,events}=fixture();input.press('forward');input.suspend(false);
 assert.equal(input.active,'forward');assert.equal(input.suspended,true);
 assert.equal(events.filter(e=>e[0]==='stop').length,0);
 input.resume();assert.equal(events.filter(e=>e[0]==='start').length,2);
});
test('joystick center dead zone and radial clamping preserve direction',()=>{
 assert.equal(stickAxes(2,2,100).forward,0);
 const half=stickAxes(0,-54,100);assert.ok(Math.abs(half.forward-.5)<1e-12);
 const diagonal=stickAxes(100,-100,100);
 assert.ok(Math.abs(Math.hypot(diagonal.forward,diagonal.left)-1)<1e-12);
 assert.equal(diagonal.forward,-diagonal.left);
});
test('rotation ignores vertical drag and follows left/right direction',()=>{
 const left=stickAxes(-100,200,100,true);
 assert.equal(left.ccw,1);assert.equal(left.forward,0);assert.equal(left.left,0);
 assert.equal(stickAxes(100,0,100,true).ccw,-1);
});
const settle=()=>new Promise(resolve=>setImmediate(resolve));
function pendingPump(){
 const pending=[],replies=[],errors=[];
 const pump=new HeldUpdates({send:()=>new Promise((resolve,reject)=>pending.push({resolve,reject})),
  onReply:value=>replies.push(value),onError:error=>errors.push(error)});
 return {pump,pending,replies,errors};
}
test('held renewals do not wait for slow HTTP responses and bound outstanding requests',async()=>{
 const {pump,pending,replies}=pendingPump();
 for(let i=0;i<10;i++){pump.tick();await settle()}
 assert.equal(pending.length,3);assert.equal(pump.pending,3);
 pending[0].resolve({accepted:true});await settle();assert.equal(replies.length,1);
 pump.tick();await settle();assert.equal(pending.length,4);
 pump.cancel();for(const request of pending)request.resolve({});await settle();
});
test('a delayed older timeout cannot cancel a later successful held update',async()=>{
 const {pump,pending,errors}=pendingPump();pump.tick();pump.tick();await settle();
 pending[1].resolve({accepted:true});await settle();
 pending[0].reject(new Error('older timeout'));await settle();assert.equal(errors.length,0);
});
test('a current failed update still cancels control, and stale request rejection does not',async()=>{
 const {pump,pending,errors}=pendingPump();pump.tick();await settle();
 const stale=new Error('reordered');stale.staleRequest=true;pending[0].reject(stale);await settle();
 assert.equal(errors.length,0);pump.tick();await settle();
 pending[1].reject(new Error('connection lost'));await settle();assert.equal(errors.length,1);
});
test('release invalidates pending callbacks and sends no queued renewal afterward',async()=>{
 const {pump,pending,replies,errors}=pendingPump();pump.tick();await settle();
 pump.cancel();pump.tick();pending[0].resolve({});await settle();
 assert.equal(pending.length,1);assert.equal(replies.length,0);assert.equal(errors.length,0);
 const other=pendingPump();other.pump.tick();other.pump.cancel();await settle();
 assert.equal(other.pending.length,0);
});
test('150 ms response latency still renews the 250 ms lease every 70 ms',async()=>{
 let now=170,lease=250,lastIntent=1;const arrivals=[],responses=[];
 const pump=new HeldUpdates({send:()=>{
  assert.ok(now<lease,'lease must not expire while held');lease=now+250;
  arrivals.push({now,intent:lastIntent});return new Promise(resolve=>responses.push({at:now+150,resolve}));
 },onReply:()=>{},onError:error=>{throw error}});
 // Initial press reply arrived at 170 ms; renewal starts immediately.
 for(let i=0;i<180;i++){
  now=170+i*70;lastIntent=i;
  for(const reply of responses.filter(r=>!r.done&&r.at<=now)){reply.done=true;reply.resolve({})}
  await settle();pump.tick();await settle();
 }
 assert.equal(arrivals.length,180);assert.ok(now>10000);
 assert.ok(arrivals.slice(1).every((r,i)=>r.now-arrivals[i].now===70));
 assert.equal(arrivals.at(-1).intent,179);pump.cancel();
 for(const reply of responses)reply.resolve({});await settle();
});
