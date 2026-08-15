const RUN_LIMIT_S = 120;
const DEFAULT_WINDOW_S = 15;
const MIN_WINDOW_S = 5;
const MAX_POINTS = 6000;
const FRAME_INTERVAL_MS = 1000 / 30;
const DISPLAY_SAMPLE_HZ = 50;
const groups = [
  { id:"attitude", title:"Attitude", unit:"deg", series:[
    {id:"pitch",label:"Pitch",path:"attitude.pitch_deg",decimals:2},
    {id:"setpoint",label:"Setpoint",path:"attitude.pitch_setpoint_deg",decimals:2},
  ]},
  { id:"rate", title:"Pitch rate", unit:"deg/s", series:[
    {id:"gyro",label:"Gyro",path:"rate.gyro_pitch_rate_dps",decimals:2},
    {id:"setpoint",label:"Setpoint",path:"rate.rate_setpoint_dps",decimals:2},
  ]},
  { id:"motion", title:"Motion", unit:"steps/s", series:[
    {id:"command",label:"Command",path:"controller.command_sps",decimals:1},
    {id:"measured",label:"Measured",path:"motion.measured_velocity_sps",decimals:1},
  ]},
  { id:"wheel-steps", title:"Wheel position", unit:"steps", series:[
    {id:"left",label:"Left actual",path:"motion.left_actual_steps",decimals:0},
    {id:"right",label:"Right actual",path:"motion.right_actual_steps",decimals:0},
  ]},
  { id:"command", title:"Command", unit:"steps/s", series:[
    {id:"command",label:"Command",path:"controller.command_sps",decimals:1},
    {id:"error",label:"Velocity error",path:"controller.velocity_error",decimals:1},
  ]},
  { id:"controller", title:"Controller", unit:"deg", series:[
    {id:"pitch",label:"Pitch error",path:"controller.pitch_error_deg",decimals:2},
    {id:"damping",label:"Velocity damping",path:"controller.velocity_damping_acceleration_mps2",decimals:2},
    {id:"trim",label:"COM trim",path:"controller.com_trim_deg",decimals:2},
  ]},
];
const paths = [...new Set(groups.flatMap(group => group.series.map(series => series.path)))];
const colors = ["#5db6ff", "#ffbf5b", "#54d6a0", "#ff7f8a", "#cb9bff", "#63d7db"];
const $ = selector => document.querySelector(selector);
const numberAt = (value, path) => {
  const found = path.split(".").reduce((item, key) => item && typeof item === "object" ? item[key] : undefined, value);
  return typeof found === "number" && Number.isFinite(found) ? found : null;
};
const store = {
  samples:[], plotSamples:[], x:[], values:new Map(),
  earliest:null, latest:null, origin:null,
  cacheStart:null, cacheEnd:null, decimated:false,
};
const state = {
  charts:new Map(), observers:[],
  visible:new Set(groups.filter(group => group.id !== "command").map(group => group.id)),
  seconds:DEFAULT_WINDOW_S, end:null, follow:true, source:null,
  events:null, paused:false, displayRun:0,
  navigator:null,
  playbackTimer:0, status:null, busy:false,
  renderQueued:false, lastRenderAt:0, backgroundLoad:0,
  ingestedSamples:0, renderedFrames:0,
};
const sync = uPlot.sync("balancer-timeline");
const icons = {
  activity: '<path d="M3 12h4l3-8 4 16 3-8h4"/>',
  upload: '<path d="M12 16V4m-4 4 4-4 4 4M5 14v5h14v-5"/>',
  radio: '<path d="M5 17a10 10 0 0 1 0-10m14 10a10 10 0 0 0 0-10M8.5 14a5 5 0 0 1 0-4m7 4a5 5 0 0 0 0-4M12 12h.01"/>',
  rocket: '<path d="M14 5c3-2 5-2 5-2s0 2-2 5l-5 5-4-4 6-4ZM8 9l-3 1-2 3 5 1m4-1 1 5-3 2-1-5"/>',
  play: '<path d="m8 5 11 7-11 7V5Z"/>',
  abort: '<path d="m9 9 6 6m0-6-6 6M8 3h8l5 5v8l-5 5H8l-5-5V8l5-5Z"/>',
};
function icon(name) { return `<svg class="ui-icon" viewBox="0 0 24 24" aria-hidden="true">${icons[name]}</svg>`; }

function elapsed(raw) { return raw - (store.origin ?? 0); }
function formatCursorTime(raw) { return raw == null ? "—" : `${elapsed(raw).toFixed(2)} s`; }
function formatNavigatorTime(raw) {
  if (raw == null) return "—";
  const value = Math.max(0, elapsed(raw));
  return state.seconds > 60 ? `${Math.floor(value/60)}:${Math.floor(value%60).toString().padStart(2,"0")}` : `${value.toFixed(1)} s`;
}
function formatAxisTime(raw) {
  const value = Math.max(0, elapsed(raw));
  return state.seconds > 60 ? `${Math.floor(value/60)}:${Math.round(value%60).toString().padStart(2,"0")}` : value.toFixed(1);
}
function isTelemetryGap(previous, current) {
  if(store.decimated)return false;
  const cadence=state.source?.sample_cadence_s??1/DISPLAY_SAMPLE_HZ;
  return current.received_at-previous.received_at>Math.max(.1,cadence*5);
}
function rebuildArrays(samples) {
  store.samples=samples;
  store.plotSamples=[]; store.x=[]; store.values=new Map(paths.map(path=>[path,[]]));
  samples.forEach((sample,index)=>{
    if(index>0&&isTelemetryGap(samples[index-1],sample)){
      store.plotSamples.push(null); store.x.push(sample.received_at-.000001);
      for(const values of store.values.values())values.push(null);
    }
    store.plotSamples.push(sample); store.x.push(sample.received_at);
    for(const path of paths)store.values.get(path).push(numberAt(sample,path));
  });
  store.cacheStart=samples[0]?.received_at??null;
  store.cacheEnd=samples.at(-1)?.received_at??null;
}
function replaceStore(body) {
  store.earliest=body.earliest;
  store.latest=body.latest;
  store.origin=body.earliest;
  store.decimated=body.decimated;
  rebuildArrays(body.samples);
}
function mergeStore(body, replaceDecimation=false) {
  store.earliest=body.earliest??store.earliest;
  store.latest=body.latest??store.latest;
  store.origin=store.origin??body.earliest;
  const bySequence=new Map(store.samples.map(sample=>[sample.sequence,sample]));
  body.samples.forEach(sample=>bySequence.set(sample.sequence,sample));
  const merged=[...bySequence.values()].sort((a,b)=>a.received_at-b.received_at).slice(-MAX_POINTS);
  if(replaceDecimation)store.decimated=body.decimated;
  rebuildArrays(merged);
}
function clearStore() {
  store.earliest=store.latest=store.origin=store.cacheStart=store.cacheEnd=null;
  store.decimated=false;
  rebuildArrays([]);
}
function append(sample) {
  if ((store.samples.at(-1)?.sequence ?? -1) >= sample.sequence) return;
  state.ingestedSamples++;
  if (store.origin == null) store.origin=sample.received_at;
  const previous=store.samples.at(-1);
  store.samples.push(sample);
  if(previous&&isTelemetryGap(previous,sample)){
    store.plotSamples.push(null);store.x.push(sample.received_at-.000001);
    for(const values of store.values.values())values.push(null);
  }
  store.plotSamples.push(sample);store.x.push(sample.received_at);
  for (const path of paths) store.values.get(path).push(numberAt(sample,path));
  store.earliest=store.earliest??sample.received_at;
  store.latest=store.cacheEnd=sample.received_at;
  store.cacheStart=store.samples[0]?.received_at??sample.received_at;
  while (store.samples.length>MAX_POINTS) {
    store.samples.shift(); rebuildArrays(store.samples);
    store.cacheStart=store.samples[0]?.received_at??null;
  }
  updateMetrics(sample);
  scheduleRender();
}
function chartData(group) { return [store.x, ...group.series.map(series=>store.values.get(series.path))]; }
function viewRange() {
  const origin=store.origin??0;
  const latest=store.latest??origin;
  if (state.follow) {
    const end=latest-origin<state.seconds ? origin+state.seconds : latest;
    return [end-state.seconds,end];
  }
  const horizonEnd=state.source?.mode==="live" ? origin+RUN_LIMIT_S : Math.max(origin+MIN_WINDOW_S,store.latest??origin+MIN_WINDOW_S);
  const end=Math.min(horizonEnd,Math.max(origin+state.seconds,state.end??latest));
  return [end-state.seconds,end];
}
function updateMetrics(sample) {
  const status=state.status??{};
  const metrics = [
    ["Pitch",numberAt(sample,"attitude.pitch_deg"),"deg",2],
    ["Velocity",numberAt(sample,"motion.measured_velocity_sps"),"steps/s",1],
    ["IMU age",typeof status.imu_age_ms==="number"?status.imu_age_ms:null,"ms",1],
    ["Feedback age",typeof status.feedback_age_ms==="number"?status.feedback_age_ms:null,"ms",1],
    ["UDP telemetry rate",typeof status.raw_packet_rate_hz==="number"?status.raw_packet_rate_hz:null,"Hz",1],
    ["Plot sample rate",typeof status.display_rate_hz==="number"?status.display_rate_hz:null,"Hz",1],
  ];
  $("#metrics").innerHTML=metrics.map(([name,value,unit,decimals])=>`<div class="metric"><small>${name}</small><strong>${value==null?"—":`${value.toFixed(decimals)} ${unit}`}</strong></div>`).join("");
  $("#events-text").textContent=`controller 0x${sample.flags.controller.toString(16)} · saturation 0x${sample.flags.saturation.toString(16)} · actuator 0x${sample.flags.actuator.toString(16)}`;
}
function render() {
  const [start,end]=viewRange();
  state.renderedFrames++;
  const plots=$("#plots");
  plots.setAttribute("data-view-start",elapsed(start).toFixed(2));
  plots.setAttribute("data-view-end",elapsed(end).toFixed(2));
  plots.setAttribute("data-window-seconds",state.seconds.toFixed(2));
  plots.setAttribute("data-ingested-samples",String(state.ingestedSamples));
  plots.setAttribute("data-rendered-frames",String(state.renderedFrames));
  for (const [id,chart] of state.charts) {
    chart.setData(chartData(groups.find(group=>group.id===id)),false);
    chart.setScale("x",{min:start,max:end});
  }
  renderNavigator();
}
function scheduleRender() {
  if (state.renderQueued) return;
  state.renderQueued=true;
  const frame=now => {
    if (now-state.lastRenderAt<FRAME_INTERVAL_MS) { window.requestAnimationFrame(frame); return; }
    state.renderQueued=false; state.lastRenderAt=now; render();
  };
  window.requestAnimationFrame(frame);
}
function horizon() {
  const start=store.origin??0;
  const duration=state.source?.mode==="live" ? RUN_LIMIT_S : Math.max(MIN_WINDOW_S,(store.latest??start)-start);
  return [start,start+duration];
}
function renderNavigator() {
  const [start,end]=viewRange(), [hStart,hEnd]=horizon(), duration=hEnd-hStart;
  const left=(start-hStart)/duration*100;
  const width=(end-start)/duration*100;
  const windowElement=$("#navigator-window");
  windowElement.style.left=`${Math.max(0,Math.min(100-width,left))}%`;
  windowElement.style.width=`${Math.max(1.5,Math.min(100,width))}%`;
  const filled=Math.max(0,Math.min(100,((store.latest??hStart)-hStart)/duration*100));
  $("#navigator-fill").setAttribute("style",`width:${filled}%`);
  $("#navigator-start").textContent=formatNavigatorTime(start);
  $("#navigator-end").textContent=formatNavigatorTime(end);
  $("#window-duration").textContent=`${state.seconds.toFixed(state.seconds%1?1:0)} s`;
  $("#timeline-state").textContent=state.follow?"Following latest":"History locked";
  $("#follow").toggleAttribute("disabled",state.follow);
}
async function fetchHistory(seconds, end=null) {
  const params=new URLSearchParams({seconds:String(seconds),max_points:String(MAX_POINTS)});
  if(end!=null)params.set("end",String(end));
  const response=await fetch(`/api/history?${params}`);
  if(!response.ok)throw new Error("History request failed.");
  return await response.json();
}
async function loadInitial() {
  const detail=await fetchHistory(DEFAULT_WINDOW_S);
  replaceStore(detail);
  state.displayRun=detail.display_run??state.displayRun;
  if(store.samples.length)updateMetrics(store.samples.at(-1));
  render();
}
async function loadCompleteCache() {
  const token=++state.backgroundLoad;
  const duration=state.source?.mode==="csv" ? Math.max(DEFAULT_WINDOW_S,state.source.duration_s??DEFAULT_WINDOW_S) : RUN_LIMIT_S;
  const body=await fetchHistory(duration);
  if(token!==state.backgroundLoad)return;
  mergeStore(body,true);
  document.body.setAttribute("data-cache-ready","true");
  document.body.setAttribute("data-cache-decimated",String(body.decimated));
  scheduleRender();
}
async function loadDetail(start, end) {
  const body=await fetchHistory(Math.max(MIN_WINDOW_S,end-start),end);
  mergeStore(body);
  scheduleRender();
}
function cursor(group, sample, card) {
  if(!sample)return;
  const values=group.series.map(series=>`<span class="cursor-value"><b>${series.label}</b><output>${(numberAt(sample,series.path)??0).toFixed(series.decimals)}</output></span>`).join("");
  (card.querySelector(".cursor")).innerHTML=`<span class="cursor-time">${formatCursorTime(sample.received_at)}</span><span class="cursor-values">${values}</span>`;
}
function create(group) {
  const card=document.createElement("article"); card.className="plot-card";
  card.innerHTML=`<div class="plot-title"><div><h2>${group.title}</h2><span>${group.unit}</span></div><div class="legend">${group.series.map((series,index)=>`<span><i style="background:${colors[index]}"></i>${series.label}</span>`).join("")}</div></div><div class="chart" aria-label="${group.title} chart"></div><div class="cursor"><span class="cursor-time">—</span><span>Move across chart to compare values.</span></div>`;
  $("#plots").append(card);
  const root=card.querySelector(".chart");
  const chart=new uPlot({
    width:Math.max(root.clientWidth,1),height:278,scales:{x:{time:false}},
    axes:[
      {stroke:"#d5deea",grid:{stroke:"#42566e",width:1},values:(_u,values)=>values.map(formatAxisTime)},
      {stroke:"#d5deea",grid:{stroke:"#42566e",width:1},label:group.unit,labelFont:"12px system-ui"},
    ],
    series:[{},...group.series.map((series,index)=>({label:series.label,stroke:colors[index],width:2}))],
    cursor:{sync,drag:{x:false,y:false}},
    hooks:{setCursor:[plot=>cursor(group,store.plotSamples[plot.cursor.idx??-1]??undefined,card)]},
  },chartData(group),root);
  const observer=new ResizeObserver(()=>chart.setSize({width:Math.max(root.clientWidth,1),height:278}));
  observer.observe(root); state.observers.push(observer); state.charts.set(group.id,chart);
}
function rebuild() {
  state.observers.forEach(observer=>observer.disconnect()); state.observers=[];
  state.charts.forEach(chart=>chart.destroy()); state.charts.clear(); $("#plots").replaceChildren();
  groups.filter(group=>state.visible.has(group.id)).forEach(create); render();
}
function setStatus(source) {
  state.source=source;
  const status=state.status??{}; const active=status.run_active===true;
  const feed=status.telemetry_connected===true; const reachable=status.pi_ready===true||feed;
  const label=source.mode==="csv"?(active?"Playing CSV":"CSV ready"):(active?(feed?"Running":"Starting · waiting for telemetry"):(reachable?"Pi ready":"Pi offline"));
  $("#connection").textContent=label; $("#connection").className=`connection ${(reachable||source.mode==="csv")?"online":"offline"}`;
  $("#source").textContent=source.mode==="csv"?`CSV · ${source.name}`:"Live source · rpi4";
  $("#deploy").toggleAttribute("disabled",source.mode==="csv"||state.busy);
}
function stopPlayback() { if(state.playbackTimer)window.clearInterval(state.playbackTimer); state.playbackTimer=0; }
function startPlayback() {
  stopPlayback(); if(state.source?.mode!=="csv")return;
  state.follow=false; state.seconds=DEFAULT_WINDOW_S; state.end=(store.origin??0)+DEFAULT_WINDOW_S; scheduleRender();
  state.playbackTimer=window.setInterval(()=>{
    state.end=Math.min((store.latest??0),(state.end??0)+.1); scheduleRender();
    if((state.end??0)>=(store.latest??0))stopPlayback();
  },100);
}
function resetForDisplayRun(displayRun) {
  state.displayRun=displayRun; state.seconds=DEFAULT_WINDOW_S; state.follow=true; state.end=null;
  state.ingestedSamples=0; document.body.setAttribute("data-cache-ready","false");
  clearStore(); scheduleRender();
}
function stream() {
  if(state.events||state.paused||document.hidden)return;
  state.events=new EventSource("/api/stream");
  state.events.addEventListener("telemetry",event=>append(JSON.parse(event.data)));
  state.events.addEventListener("status",event=>{
    const status=JSON.parse(event.data); state.status=status;
    if(typeof status.display_run==="number"&&status.display_run!==state.displayRun)resetForDisplayRun(status.display_run);
    if(state.source)setStatus({...state.source,connection_state:String(status.connection_state??""),connection_message:String(status.connection_message??"")});
    if(store.samples.length)updateMetrics(store.samples.at(-1));
  });
  state.events.onerror=()=>{state.events?.close();state.events=null;if(!state.paused&&!document.hidden)window.setTimeout(stream,2000);};
}
async function post(path, body, headers) {
  const response=await fetch(path,{method:"POST",body,headers});
  const result=await response.json();
  if(!response.ok||!result.ok)throw new Error(result.error??"Request failed."); if(result.source)setStatus(result.source); return result;
}
async function operation(id) {
  if(state.busy)return;
  const button=$("#"+id); const controls=[...document.querySelectorAll(".run-buttons button")];
  const messages={deploy:"Deploying to pi@rpi4…",start:state.source?.mode==="csv"?"Starting CSV playback…":"Starting balancer…",abort:state.source?.mode==="csv"?"Stopping CSV playback…":"Stopping balancer…"};
  state.busy=true; controls.forEach(control=>control.disabled=true); button.classList.add("working"); button.setAttribute("aria-busy","true"); $("#operation-result").textContent=messages[id];
  try {
    if(id==="abort"){state.paused=true;stopPlayback();state.events?.close();state.events=null;}
    const response=await post(`/api/${id}`);
    if(id==="start"){
      state.paused=false; resetForDisplayRun(response.display_run??state.displayRun); state.status={...(state.status??{}),run_active:true,telemetry_connected:false}; stream();
      if(state.source?.mode==="csv"){await loadCompleteCache();startPlayback();}
    } else if(id==="abort") state.status={...(state.status??{}),run_active:false,telemetry_connected:false};
    if(state.source)setStatus(state.source); $("#operation-result").textContent=response.message??"Done.";
  } catch(error){$("#operation-result").textContent=`Failed: ${error.message}`;}
  finally {state.busy=false;controls.forEach(control=>control.disabled=false);button.classList.remove("working");button.removeAttribute("aria-busy");if(state.source)setStatus(state.source);}
}
function navigatorPointer(event) {
  const target=event.target; const track=$("#navigator-track"); const [start,end]=viewRange();
  const mode=target.classList.contains("left-handle")?"left":target.classList.contains("right-handle")?"right":"pan";
  state.navigator={mode,x:event.clientX,start,end}; track.setPointerCapture(event.pointerId); track.classList.add("active");
}
function navigatorMove(event) {
  if(!state.navigator)return;
  const track=$("#navigator-track"), [hStart,hEnd]=horizon();
  const delta=(event.clientX-state.navigator.x)/Math.max(1,track.clientWidth)*(hEnd-hStart);
  let [start,end]=[state.navigator.start,state.navigator.end];
  if(state.navigator.mode==="pan") { const width=end-start; start=Math.max(hStart,Math.min(hEnd-width,start+delta)); end=start+width; }
  else if(state.navigator.mode==="left") start=Math.max(hStart,Math.min(end-MIN_WINDOW_S,start+delta));
  else end=Math.min(hEnd,Math.max(start+MIN_WINDOW_S,end+delta));
  state.follow=false; state.seconds=Math.min(RUN_LIMIT_S,Math.max(MIN_WINDOW_S,end-start)); state.end=end; render();
}
function navigatorFinish() {
  if(!state.navigator)return;
  state.navigator=null; $("#navigator-track").classList.remove("active");
  if(store.decimated) { const [start,end]=viewRange(); void loadDetail(start,end); }
}
function layout() {
  $("#app").innerHTML = `<div class="dashboard-shell"><aside class="left-rail"><div class="brand"><div class="brand-mark"><img src="/balancer-mark.svg" alt="Balancer Bot"></div><div><span class="brand-kicker">LOCAL CONSOLE</span><h1>Balancer</h1><p id="source" class="subtle">Live source · rpi4</p></div></div><div class="source-actions"><span class="rail-label">Data source</span><label class="file-button source-file">${icon("upload")}<span>Open CSV capture</span><input id="csv-file" type="file" accept=".csv,text/csv"></label></div><div class="rail-signature"><span></span>LOCAL TELEMETRY</div></aside><main class="dashboard-main"><section id="metrics" class="metrics"></section><section class="workspace"><div class="workspace-head"><div class="workspace-title">${icon("activity")}<h2>Telemetry</h2></div><div class="plot-controls"><div class="tabs">${groups.map(group=>`<button data-group="${group.id}" class="${state.visible.has(group.id)?"active":""}">${group.title}</button>`).join("")}</div><div class="toolbar-divider"></div><div class="ranges"><span id="timeline-state">Following latest</span><button id="follow" class="primary" disabled>Follow latest</button></div></div></div><div class="navigator"><div class="navigator-labels"><span id="navigator-start">0.0 s</span><strong>History window · <em id="window-duration">15 s</em></strong><span id="navigator-end">15.0 s</span></div><div id="navigator-track" class="navigator-track"><div id="navigator-fill" class="navigator-fill"></div><div id="navigator-window" class="navigator-window"><i class="handle left-handle"></i><span>DRAG</span><i class="handle right-handle"></i></div></div></div><div id="plots" class="plot-grid"></div></section></main><aside class="right-rail"><section class="run-controls" aria-label="Run controls"><div class="run-label"><span>${icon("radio")}Run control</span><strong id="connection" class="connection offline">Idle · waiting</strong></div><div class="run-buttons"><button id="deploy" class="secondary">${icon("rocket")}Deploy</button><button id="start" class="primary">${icon("play")}Start</button><button id="abort" class="danger">${icon("abort")}Abort</button></div><output id="operation-result" class="operation-result" aria-live="polite">Ready.</output></section><section class="run-notes"><span class="rail-label">Session</span><p>Start clears the display clock. Abort freezes presentation while capture stays passive.</p></section><section class="events"><strong>Latched flags</strong><span id="events-text">No telemetry received.</span></section></aside></div>`;
  document.querySelectorAll("[data-group]").forEach(button=>button.onclick=()=>{const id=button.dataset.group;state.visible.has(id)?state.visible.delete(id):state.visible.add(id);button.classList.toggle("active");rebuild();});
  $("#follow").addEventListener("click",()=>{state.follow=true;state.end=null;scheduleRender();if(store.decimated){const [start,end]=viewRange();void loadDetail(start,end);}});
  (["deploy","start","abort"]).forEach(id=>$("#"+id).addEventListener("click",()=>void operation(id)));
  const track=$("#navigator-track"); track.addEventListener("pointerdown",navigatorPointer); track.addEventListener("pointermove",navigatorMove); track.addEventListener("pointerup",navigatorFinish); track.addEventListener("pointercancel",navigatorFinish);
  $("#csv-file").addEventListener("change",async event=>{const file=(event.currentTarget).files?.[0];if(!file)return;try{const result=await post("/api/source/csv",file,{"Content-Type":"text/csv","X-Filename":file.name});state.paused=true;stopPlayback();state.events?.close();state.events=null;clearStore();await loadInitial();await loadCompleteCache();$("#operation-result").textContent=result.message??"CSV loaded. Press Start to play.";}catch(error){$("#operation-result").textContent=`Failed: ${error.message}`;}});
  document.addEventListener("visibilitychange",()=>{if(document.hidden){state.events?.close();state.events=null;}else if(!state.paused){void loadCompleteCache();stream();}});
}
layout();
void(async()=>{
  const source=await(await fetch("/api/source")).json(); setStatus(source); state.displayRun=source.display_run??0;
  await loadInitial(); requestAnimationFrame(()=>{rebuild();setTimeout(()=>void loadCompleteCache(),0);}); stream();
})();
