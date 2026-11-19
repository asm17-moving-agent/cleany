// Transient interruption preserves the physical hold; release/STOP ends it.
class HoldInput {
  constructor({start,change,stop}) {
    this.start=start;this.change=change;this.stop=stop;this.active=null;this.suspended=false;
  }
  press(value) {
    const previous=this.active;this.active=value;
    if(previous===null){this.suspended=false;this.start(value)}
    else if(previous!==value)this.change(value);
  }
  release() { this.cancel(); }
  suspend(stopRemote=true) {
    if(this.active===null||this.suspended)return;
    this.suspended=true;if(stopRemote)this.stop(true);
  }
  resume() {
    if(this.active===null||!this.suspended)return false;
    this.suspended=false;this.start(this.active);return true;
  }
  cancel(force=false) {
    const wasActive=this.active!==null;this.active=null;this.suspended=false;
    if(wasActive||force)this.stop(false);
  }
}
// Circular clamp and radial dead zone; screen Y points down.
function stickAxes(dx,dy,radius,rotation=false) {
  let x=dx/radius,y=rotation?0:dy/radius;
  const distance=Math.hypot(x,y),bounded=Math.min(1,distance);
  if(distance>1){x/=distance;y/=distance}
  if(bounded<=0.08)return {x:0,y:0,forward:0,left:0,ccw:0};
  const gain=(bounded-0.08)/(0.92*bounded);
  return {x,y,forward:rotation?0:-y*gain,left:rotation?0:-x*gain,ccw:rotation?-x*gain:0};
}
// Fixed-cadence renewal is independent of response latency, with bounded concurrency.
class HeldUpdates {
  constructor({send,onReply,onError,limit=3}) {
    this.send=send;this.onReply=onReply;this.onError=onError;this.limit=limit;
    this.pending=0;this.sequence=0;this.lastSuccess=0;this.cancelled=false;
  }
  tick() {
    if(this.cancelled||this.pending>=this.limit)return;
    const sequence=++this.sequence;this.pending++;
    Promise.resolve().then(()=>this.cancelled?null:this.send()).then(reply=>{
      if(this.cancelled||reply===null)return;
      this.lastSuccess=Math.max(this.lastSuccess,sequence);this.onReply(reply);
    }).catch(error=>{
      if(!this.cancelled&&!error.staleRequest&&sequence>=this.lastSuccess)this.onError(error);
    }).finally(()=>{this.pending--});
  }
  cancel() {this.cancelled=true}
}
if(typeof module!=='undefined')module.exports={HoldInput,stickAxes,HeldUpdates};

if(typeof window!=='undefined')window.CleanyInput={HoldInput,stickAxes,HeldUpdates};
