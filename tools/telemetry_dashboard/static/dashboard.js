const RUN_LIMIT_S = 120;
const DEFAULT_WINDOW_S = 15;
const MIN_WINDOW_S = 5;
const MAX_POINTS = 6000;
const TRIM_BATCH_POINTS = 50;
const FRAME_INTERVAL_MS = 1000 / 30;
const DISPLAY_SAMPLE_HZ = 50;
const PERFORMANCE_WINDOW_S = 1.5;
const JOYSTICK_REPEAT_MS = 100;
const PID_FIELDS = [
  ["pitch_gain", "Pitch gain (SPS/rad)"],
  ["pitch_rate_gain", "Pitch-rate gain (SPS/(rad/s))"],
  ["pitch_accel_gain", "Pitch-acceleration gain (SPS/(rad/s²))"],
  ["drive_max_velocity_mps", "Maximum drive velocity (m/s)"],
  ["velocity_gain_per_s", "Velocity feedback gain (1/s)"],
  ["velocity_feedback_cutoff_hz", "Velocity feedback cutoff (Hz)"],
  ["outer_pitch_limit_deg", "Outer pitch limit (deg)"],
  ["fixed_com_trim_deg", "Fixed COM trim (deg)"],
  ["adaptive_com_trim_enabled", "Adaptive COM enabled (0/1)"],
  ["adaptive_com_trim_gain_deg_per_mps_s", "Adaptive COM gain (deg/(m/s·s))"],
  ["adaptive_com_trim_limit_deg", "Adaptive COM limit (deg)"],
  ["turn_max_sps", "Turn limit (steps/s)"],
  ["balance_max_sps", "Balance limit (steps/s)"],
  ["planner_max_acceleration_mps2", "Planner acceleration (m/s²)"],
  ["planner_max_deceleration_mps2", "Planner deceleration (m/s²)"],
  ["planner_max_jerk_mps3", "Planner jerk (m/s³)"],
  ["velocity_i_gain_per_s2", "Velocity I gain (1/s²)"],
  ["velocity_i_leak_time_s", "Velocity I leak time (s)"],
  ["velocity_i_acceleration_limit_mps2", "Velocity I limit (m/s²)"],
];
const groups = [
  { id:"performance", title:"Balance performance", unit:"% reference", help:"1.5 s rolling windows. Pitch error is referenced to 1°, rate to 30°/s, command and velocity to 1000 SPS; lower is quieter.", series:[
    {id:"composite",label:"Composite activity",derived:"performance.composite",decimals:1,width:3},
    {id:"pitch",label:"Pitch error RMS",derived:"performance.pitch",decimals:1},
    {id:"rate",label:"Rate RMS",derived:"performance.rate",decimals:1},
    {id:"command",label:"Command RMS",derived:"performance.command",decimals:1},
    {id:"velocity",label:"Velocity RMS",derived:"performance.velocity",decimals:1},
    {id:"saturation",label:"Saturation",derived:"performance.saturation",decimals:1},
  ]},
  { id:"freeze-diagnostics", title:"Freeze diagnostics", unit:"ms", help:"Opt-in timing view. Sender and receiver gaps separate control/transport pauses from browser append/render work.", series:[
    {id:"receive-gap",label:"Receiver gap",derived:"diagnostics.receive_gap_ms",decimals:1},
    {id:"sender-gap",label:"Sender gap",derived:"diagnostics.sender_gap_ms",decimals:1},
    {id:"sse-gap",label:"Browser SSE gap",derived:"diagnostics.sse_gap_ms",decimals:1},
    {id:"append",label:"Browser append",derived:"diagnostics.append_ms",decimals:1},
    {id:"render",label:"Browser render",derived:"diagnostics.render_ms",decimals:1},
    {id:"sample-age",label:"Browser sample age",derived:"diagnostics.sample_age_ms",decimals:1},
  ]},
  { id:"attitude", title:"Attitude", unit:"deg", help:"Fused body pitch versus the total target", series:[
    {id:"fused",label:"Fused pitch",path:"attitude.fused_pitch_deg",decimals:2},
    {id:"target",label:"Pitch target",path:"attitude.pitch_setpoint_deg",decimals:2},
  ]},
  { id:"rate", title:"Pitch rate / gyro diagnostics", unit:"deg/s", help:"Optional diagnostic view: compare the noisy raw gyro with the filtered control rate.", series:[
    {id:"filtered",label:"Filtered control rate",path:"rate.filtered_pitch_rate_dps",decimals:2},
    {id:"raw",label:"Raw gyro",path:"rate.gyro_pitch_rate_dps",decimals:2},
  ]},
  { id:"contributions", title:"Balance contributions", unit:"SPS", help:"Signed inner-loop terms and final balance command", series:[
    {id:"pitch",label:"Pitch",path:"controller.pitch_feedback_sps",decimals:1},
    {id:"rate",label:"Rate",path:"controller.pitch_rate_feedback_sps",decimals:1},
    {id:"accel",label:"Acceleration",path:"controller.pitch_accel_feedback_sps",decimals:1},
    {id:"command",label:"Command",path:"controller.command_sps",decimals:1},
  ]},
  { id:"outer-loop", title:"Velocity-reference outer loop", unit:"SI", help:"Planner, velocity feedback, acceleration authority, and fixed trim", series:[
    {id:"target",label:"Pitch target",path:"attitude.pitch_setpoint_deg",decimals:2},
    {id:"user-velocity",label:"User velocity",path:"motion.user_velocity_mps",decimals:3},
    {id:"reference-velocity",label:"Reference velocity",path:"motion.reference_velocity_mps",decimals:3},
    {id:"feedback-velocity",label:"Feedback estimate",path:"motion.velocity_feedback_estimate_mps",decimals:3},
    {id:"acceleration",label:"Acceleration command",path:"controller.acceleration_cmd_mps2",decimals:3},
    {id:"drive-pitch",label:"Drive pitch",path:"controller.drive_pitch_target_deg",decimals:2},
    {id:"trim",label:"Fixed/COM trim",path:"controller.com_trim_deg",decimals:2},
  ]},
  { id:"joystick", title:"Joystick command", unit:"normalized", help:"Forward and turn values sent by this dashboard session; unavailable for preloaded history", series:[
    {id:"forward",label:"Forward",derived:"joystick.forward",decimals:2},
    {id:"turn",label:"Turn",derived:"joystick.turn",decimals:2},
  ]},
  { id:"motion", title:"Motion feedback", unit:"steps/s", help:"Commanded motion versus completed-step feedback", series:[
    {id:"command",label:"Command",path:"controller.command_sps",decimals:1},
    {id:"applied-left",label:"Applied left",path:"motion.left_slewed_sps",decimals:1},
    {id:"applied-right",label:"Applied right",path:"motion.right_slewed_sps",decimals:1},
    {id:"corrected",label:"Corrected velocity",path:"motion.corrected_axle_velocity_sps",decimals:1},
    {id:"control-filtered",label:"Control-filtered velocity",path:"motion.velocity_control_sps",decimals:1},
    {id:"completed",label:"Completed steps",path:"motion.raw_completed_velocity_sps",decimals:1},
  ]},
  { id:"wheel-steps", title:"Wheel position", unit:"steps", help:"Completed wheel-step counters", series:[
    {id:"left",label:"Left actual",path:"motion.left_actual_steps",decimals:0},
    {id:"right",label:"Right actual",path:"motion.right_actual_steps",decimals:0},
  ]},
];
const derivedSeries = groups.flatMap(group => group.series.filter(series => series.derived));
const paths = [...new Set(groups.flatMap(group => group.series.map(series => series.path).filter(Boolean)))];
const colors = ["#5db6ff", "#ffbf5b", "#54d6a0", "#ff7f8a", "#cb9bff", "#63d7db"];
const $ = selector => document.querySelector(selector);
const numberAt = (value, path) => {
  const found = path.split(".").reduce((item, key) => item && typeof item === "object" ? item[key] : undefined, value);
  return typeof found === "number" && Number.isFinite(found) ? found : null;
};
const store = {
  samples:[], plotSamples:[], x:[], values:new Map(),
  derived:new Map(derivedSeries.map(series => [series.derived, []])),
  earliest:null, latest:null, origin:null,
  cacheStart:null, cacheEnd:null, decimated:false,
};
const state = {
  charts:new Map(), observers:[],
  visible:new Set(["performance","attitude","contributions","outer-loop"]),
  seconds:DEFAULT_WINDOW_S, end:null, follow:true, source:null,
  events:null, paused:false, displayRun:0,
  navigator:null,
  playbackTimer:0, status:null, busy:false,
  pid:null, joystick:{pad:null,pointerId:null,timer:0,forward:0,turn:0,inFlight:false,pending:null,neutralPending:false},
  renderQueued:false, lastRenderAt:0, backgroundLoad:0,
  ingestedSamples:0, renderedFrames:0,
  lastSseAt:null,
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
function rms(values) {
  return values.length ? Math.sqrt(values.reduce((sum,value) => sum + value * value, 0) / values.length) : null;
}
function performanceValuesAt(index) {
  const sample = store.plotSamples[index];
  if (!sample) return null;
  const end = sample.received_at;
  const pitchErrors = [], rates = [], commands = [], velocities = [];
  let saturationSamples = 0, samples = 0;
  for (let cursor = index; cursor >= 0; cursor--) {
    const candidate = store.plotSamples[cursor];
    if (!candidate) break;
    if (end - candidate.received_at > PERFORMANCE_WINDOW_S) break;
    const pitchError = numberAt(candidate,"controller.pitch_error_deg") ??
      ((numberAt(candidate,"attitude.fused_pitch_deg") ?? 0) -
       (numberAt(candidate,"attitude.pitch_setpoint_deg") ?? 0));
    const rate = numberAt(candidate,"rate.filtered_pitch_rate_dps");
    const command = numberAt(candidate,"controller.command_sps");
    const velocity = numberAt(candidate,"motion.corrected_axle_velocity_sps");
    if (pitchError != null) pitchErrors.push(pitchError);
    if (rate != null) rates.push(rate);
    if (command != null) commands.push(command);
    if (velocity != null) velocities.push(velocity);
    const controllerSaturation = numberAt(candidate,"flags.saturation") ?? 0;
    const actuatorSaturation = numberAt(candidate,"flags.actuator_saturation") ?? 0;
    if (controllerSaturation !== 0 || actuatorSaturation !== 0) saturationSamples++;
    samples++;
  }
  const pitch = rms(pitchErrors);
  const rate = rms(rates);
  const command = rms(commands);
  const velocity = rms(velocities);
  const normalized = [pitch == null ? null : pitch / 1.0 * 100,
    rate == null ? null : rate / 30.0 * 100,
    command == null ? null : command / 1000.0 * 100,
    velocity == null ? null : velocity / 1000.0 * 100].filter(value => value != null);
  return {
    "performance.pitch": pitch == null ? null : pitch / 1.0 * 100,
    "performance.rate": rate == null ? null : rate / 30.0 * 100,
    "performance.command": command == null ? null : command / 1000.0 * 100,
    "performance.velocity": velocity == null ? null : velocity / 1000.0 * 100,
    "performance.composite": normalized.length ? rms(normalized) : null,
    "performance.saturation": samples ? saturationSamples / samples * 100 : null,
  };
}
function previousTelemetryAt(index) {
  for(let cursor=index-1;cursor>=0;cursor--) {
    if(store.plotSamples[cursor]) return store.plotSamples[cursor];
  }
  return null;
}
function diagnosticValuesAt(index) {
  const sample=store.plotSamples[index];
  if(!sample) return null;
  const previous=previousTelemetryAt(index);
  const browser=sample.browserDiagnostics??{};
  const receiveGap=previous ? (sample.received_at-previous.received_at)*1000 : null;
  const sameRun=previous && sample.run_id && previous.run_id===sample.run_id;
  const senderGap=sameRun && sample.sender_monotonic_ns>previous.sender_monotonic_ns
    ? (sample.sender_monotonic_ns-previous.sender_monotonic_ns)/1e6 : null;
  return {
    "diagnostics.receive_gap_ms": receiveGap,
    "diagnostics.sender_gap_ms": senderGap,
    "diagnostics.sse_gap_ms": browser.sse_gap_ms??null,
    "diagnostics.append_ms": browser.append_ms??null,
    "diagnostics.render_ms": browser.render_ms??null,
    "diagnostics.sample_age_ms": browser.sample_age_ms??null,
  };
}
function joystickValuesAt(index) {
  const sample=store.plotSamples[index];
  if(!sample) return null;
  const browser=sample.browserDiagnostics??{};
  if(browser.joystick_command_valid!==true) return null;
  const value=name=>typeof browser[name]==="number"&&Number.isFinite(browser[name])?browser[name]:null;
  return {"joystick.forward":value("joystick_forward"),"joystick.turn":value("joystick_turn")};
}
function derivedValuesAt(index) {
  return {...(performanceValuesAt(index)??{}),...(diagnosticValuesAt(index)??{}),...(joystickValuesAt(index)??{})};
}
function rebuildDerivedArrays() {
  store.derived = new Map(derivedSeries.map(series => [series.derived, []]));
  for (let index = 0; index < store.plotSamples.length; index++) {
    const values = derivedValuesAt(index);
    for (const series of derivedSeries) store.derived.get(series.derived).push(values?.[series.derived] ?? null);
  }
}
function updateDerivedTail() {
  const index=store.plotSamples.length-1;
  if(index<0) return;
  const values=derivedValuesAt(index);
  for(const series of derivedSeries) {
    const array=store.derived.get(series.derived);
    if(array?.length>index) array[index]=values?.[series.derived]??null;
  }
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
  rebuildDerivedArrays();
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
  state.lastSseAt=null;
  rebuildArrays([]);
}
function append(sample) {
  if ((store.samples.at(-1)?.sequence ?? -1) >= sample.sequence) return;
  const appendStarted=performance.now();
  state.ingestedSamples++;
  sample.browserDiagnostics={...(sample.browserDiagnostics??{}),
    joystick_command_valid:true,
    joystick_forward:state.joystick.forward,
    joystick_turn:state.joystick.turn,
  };
  if (store.origin == null) store.origin=sample.received_at;
  const previous=store.samples.at(-1);
  store.samples.push(sample);
  if(previous&&isTelemetryGap(previous,sample)){
    store.plotSamples.push(null);store.x.push(sample.received_at-.000001);
    for(const values of store.values.values())values.push(null);
    for(const values of store.derived.values())values.push(null);
  }
  store.plotSamples.push(sample);store.x.push(sample.received_at);
  for (const path of paths) store.values.get(path).push(numberAt(sample,path));
  const derivedValues = derivedValuesAt(store.plotSamples.length - 1);
  for (const series of derivedSeries) store.derived.get(series.derived).push(derivedValues?.[series.derived] ?? null);
  store.earliest=store.earliest??sample.received_at;
  store.latest=store.cacheEnd=sample.received_at;
  store.cacheStart=store.samples[0]?.received_at??sample.received_at;
  if (store.samples.length>MAX_POINTS+TRIM_BATCH_POINTS) {
    const removeCount=store.samples.length-MAX_POINTS;
    rebuildArrays(store.samples.slice(removeCount));
    store.cacheStart=store.samples[0]?.received_at??null;
  }
  const appendMs=performance.now()-appendStarted;
  sample.browserDiagnostics={...(sample.browserDiagnostics??{}),append_ms:appendMs};
  updateDerivedTail();
  updateMetrics(sample);
  scheduleRender();
}
function chartData(group) {
  return [store.x, ...group.series.map(series => series.derived
    ? store.derived.get(series.derived)
    : store.values.get(series.path))];
}
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
  const statusNumber = name => typeof status[name]==="number" && Number.isFinite(status[name]) ? status[name] : null;
  const sampleOrStatus = (path,statusName) => numberAt(sample,path) ?? statusNumber(statusName);
  const metrics = [
    ["Pitch",numberAt(sample,"attitude.fused_pitch_deg"),"deg",2],
    ["Pitch rate",numberAt(sample,"rate.filtered_pitch_rate_dps"),"deg/s",1],
    ["Balance command",numberAt(sample,"controller.command_sps"),"SPS",0],
    ["Velocity",numberAt(sample,"motion.corrected_axle_velocity_sps"),"steps/s",0],
    ["IMU age",sampleOrStatus("timing.imu_age_ms","imu_age_ms"),"ms",1],
    ["Feedback age",sampleOrStatus("timing.feedback_age_ms","feedback_age_ms"),"ms",1],
  ];
  $("#metrics").innerHTML=metrics.map(([name,value,unit,decimals])=>`<div class="metric"><small>${name}</small><strong>${value==null?"—":`${value.toFixed(decimals)} ${unit}`}</strong></div>`).join("");
  $("#events-text").textContent=sample?.flags
    ? `controller 0x${sample.flags.controller.toString(16)} · saturation 0x${sample.flags.saturation.toString(16)} · actuator 0x${sample.flags.actuator.toString(16)}`
    : "No telemetry received.";
}
function render() {
  const renderStarted=performance.now();
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
  const renderMs=performance.now()-renderStarted;
  const sample=store.samples.at(-1);
  if(sample) {
    sample.browserDiagnostics={...(sample.browserDiagnostics??{}),render_ms:renderMs};
    if(state.lastSseAt!=null) {
      sample.browserDiagnostics.sample_age_ms=Math.max(0,performance.now()-state.lastSseAt);
    }
    updateDerivedTail();
  }
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
function cursor(group, sample, index, card) {
  if(!sample)return;
  const values=group.series.map(series=>{
    const value = series.derived
      ? store.derived.get(series.derived)?.[index]
      : numberAt(sample,series.path);
    return `<span class="cursor-value"><b>${series.label}</b><output>${(value??0).toFixed(series.decimals)}</output></span>`;
  }).join("");
  (card.querySelector(".cursor")).innerHTML=`<span class="cursor-time">${formatCursorTime(sample.received_at)}</span><span class="cursor-values">${values}</span>`;
}
function create(group) {
  const card=document.createElement("article"); card.className="plot-card";
  card.dataset.plotGroup=group.id;
  card.innerHTML=`<div class="plot-title"><div class="plot-heading"><h2>${group.title}</h2><span>${group.unit}</span>${group.help?`<small>${group.help}</small>`:""}</div><div class="legend">${group.series.map((series,index)=>`<span><i style="background:${colors[index]}"></i>${series.label}</span>`).join("")}</div></div><div class="chart" aria-label="${group.title} chart"></div><div class="cursor"><span class="cursor-time">—</span><span>Move across chart to compare values.</span></div>`;
  $("#plots").append(card);
  const root=card.querySelector(".chart");
  const chartHeight=group.id==="performance"?250:278;
  const chart=new uPlot({
    width:Math.max(root.clientWidth,1),height:chartHeight,scales:{x:{time:false}},
    axes:[
      {stroke:"#d5deea",grid:{stroke:"#42566e",width:1},values:(_u,values)=>values.map(formatAxisTime)},
      {stroke:"#d5deea",grid:{stroke:"#42566e",width:1},label:group.unit,labelFont:"12px system-ui"},
    ],
    series:[{},...group.series.map((series,index)=>({label:series.label,stroke:colors[index],width:series.width??2,show:series.show!==false}))],
    cursor:{sync,drag:{x:false,y:false}},
    hooks:{setCursor:[plot=>{
      const index=plot.cursor.idx??-1;
      cursor(group,store.plotSamples[index]??undefined,index,card);
    }]},
  },chartData(group),root);
  const observer=new ResizeObserver(()=>chart.setSize({width:Math.max(root.clientWidth,1),height:chartHeight}));
  observer.observe(root); state.observers.push(observer); state.charts.set(group.id,chart);
}
function rebuild() {
  state.observers.forEach(observer=>observer.disconnect()); state.observers=[];
  state.charts.forEach(chart=>chart.destroy()); state.charts.clear(); $("#plots").replaceChildren();
  groups.filter(group=>state.visible.has(group.id)).forEach(create); render();
}
function liveConnection(status) {
  const telemetryConnected=status.telemetry_connected===true;
  const piReady=status.pi_ready===true;
  if(telemetryConnected)return {label:"Streaming",online:true};
  if(piReady)return {label:"Pi online · telemetry stopped",online:true};
  return {label:"Pi offline",online:false};
}
function setStatus(source) {
  state.source=source;
  const status=state.status??{};
  const connection=source.mode==="csv"
    ? {label:status.run_active===true?"Playing CSV":"CSV ready",online:true}
    : liveConnection(status);
  $("#connection").textContent=connection.label; $("#connection").className=`connection ${connection.online?"online":"offline"}`;
  $("#source").textContent=source.mode==="csv"?`CSV · ${source.name}`:"Live source · rpi4";
  $("#deploy").toggleAttribute("disabled",source.mode==="csv"||state.busy);
  updateJoystickControls();
  updatePidStatus(status.pid_status);
}

function joystickAllowed() {
  return state.source?.mode==="live" && state.status?.run_active===true && !state.busy;
}
function updateJoystickControls() {
  const enabled=joystickAllowed();
  const pad=$("#drive-pad"); if(pad)pad.setAttribute("aria-disabled",String(!enabled));
  const status=$("#joystick-status");
  if(status)status.textContent=enabled?"Drag from center · release to neutral.":"Start a live run to enable joystick commands.";
  if(!enabled&&state.joystick.pad)stopJoystick();
  const pidEnabled=state.source?.mode==="live"&&!state.busy;
  document.querySelectorAll("[data-pid-field], #pid-apply, #pid-load").forEach(control=>control.disabled=!pidEnabled);
}
async function sendJoystick(vector={forward:0,turn:0}, release=false) {
  if(release) {
    state.joystick.neutralPending=true;
    state.joystick.pending=null;
  } else {
    state.joystick.pending={vector:{forward:Number(vector.forward)||0,turn:Number(vector.turn)||0}};
  }
  if(state.joystick.inFlight)return;
  state.joystick.inFlight=true;
  while(state.joystick.neutralPending||state.joystick.pending) {
    const isNeutral=state.joystick.neutralPending;
    const request=isNeutral?{release:true}:state.joystick.pending;
    state.joystick.neutralPending=false;
    if(!isNeutral)state.joystick.pending=null;
    try {
      const response=await fetch("/api/joystick",{
        method:"POST",
        headers:{"Content-Type":"application/json"},
        body:JSON.stringify(isNeutral?{release:true}:request.vector),
        keepalive:isNeutral,
      });
      const result=await response.json();
      if(!response.ok||!result.ok)throw new Error(result.error??"Joystick command failed.");
    } catch(error) {
      if(!isNeutral) {
        stopJoystick(false);
        $("#operation-result").textContent=`Joystick stopped: ${error.message}`;
        state.joystick.pending=null;
        state.joystick.neutralPending=true;
      } else {
        state.joystick.pending=null;
      }
    }
  }
  state.joystick.inFlight=false;
}
function clearJoystickVisuals() {
  if(state.joystick.timer)window.clearInterval(state.joystick.timer);
  state.joystick.timer=0; state.joystick.pad=null; state.joystick.pointerId=null; state.joystick.forward=0; state.joystick.turn=0;
  const pad=$("#drive-pad");
  if(pad) {
    pad.classList.remove("active");
    pad.style.setProperty("--drive-pad-x","50%");
    pad.style.setProperty("--drive-pad-y","50%");
    pad.setAttribute("aria-valuetext","Neutral");
  }
}
function stopJoystick(sendNeutral=true) {
  clearJoystickVisuals();
  if(sendNeutral)void sendJoystick({forward:0,turn:0},true);
}
function clampJoystick(value) { return Math.max(-1,Math.min(1,value)); }
function drivePadVectorAt(pad,event) {
  const rect=pad.getBoundingClientRect();
  const centerX=rect.left+rect.width/2, centerY=rect.top+rect.height/2;
  const x=clampJoystick((event.clientX-centerX)/Math.max(1,rect.width/2));
  const y=clampJoystick((event.clientY-centerY)/Math.max(1,rect.height/2));
  return Math.abs(y)>Math.abs(x)?{forward:-y,turn:0}:{forward:0,turn:x};
}
function updateDrivePad(pad,event) {
  const vector=drivePadVectorAt(pad,event);
  state.joystick.forward=vector.forward; state.joystick.turn=vector.turn;
  pad.style.setProperty("--drive-pad-x",`${50+vector.turn*50}%`);
  pad.style.setProperty("--drive-pad-y",`${50-vector.forward*50}%`);
  pad.setAttribute("aria-valuetext",`Forward ${vector.forward.toFixed(2)}, turn ${vector.turn.toFixed(2)}`);
  void sendJoystick(vector);
}
function startJoystick(pad,event) {
  if(!joystickAllowed()||state.joystick.pad)return;
  state.joystick.pad=pad; state.joystick.pointerId=event.pointerId;
  pad.classList.add("active");
  pad.setPointerCapture?.(event.pointerId);
  updateDrivePad(pad,event);
  state.joystick.timer=window.setInterval(()=>{
    if(!joystickAllowed()||state.joystick.pad!==pad) {stopJoystick();return;}
    void sendJoystick({forward:state.joystick.forward,turn:state.joystick.turn});
  },JOYSTICK_REPEAT_MS);
}
function moveJoystick(pad,event) {
  if(state.joystick.pad!==pad||state.joystick.pointerId!==event.pointerId)return;
  if(event.buttons===0) {stopJoystick();return;}
  updateDrivePad(pad,event);
}
function finishJoystick(pad,event) {
  if(state.joystick.pad!==pad||state.joystick.pointerId!==event.pointerId)return;
  stopJoystick();
}
const PID_MIN_STEP = 0.0001;
function pidSpinnerStep(value) {
  const raw=Math.abs(Number(value))*.1;
  if(!Number.isFinite(raw)||raw<=0)return PID_MIN_STEP;
  const magnitude=10**Math.floor(Math.log10(raw));
  return Math.max(PID_MIN_STEP,Number((Math.round(raw/magnitude)*magnitude).toPrecision(12)));
}
function updatePidSpinnerStep(input) {
  input.step=String(pidSpinnerStep(input.value));
}
function renderPidFields(values) {
  PID_FIELDS.forEach(([name])=>{
    const input=document.querySelector(`[data-pid-field="${name}"]`);
    if(input&&typeof values?.[name]==="number") {
      input.value=String(values[name]);
      updatePidSpinnerStep(input);
    }
  });
}
function updatePidStatus(status) {
  const output=$("#pid-status");
  if(!output||!status)return;
  output.textContent=status.message??"";
  output.className=`pid-status ${status.state??""}`;
}
async function loadPid() {
  try {
    const response=await fetch("/api/pid");
    const result=await response.json();
    if(!response.ok||!result.ok)throw new Error(result.error??"PID config unavailable.");
    state.pid=result; renderPidFields(result.values); updatePidStatus(result.last_status);
  } catch(error) {
    const output=$("#pid-status"); if(output)output.textContent=`PID tuning unavailable: ${error.message}`;
  }
}
async function applyPid() {
  const values={};
  for(const [name] of PID_FIELDS) {
    const value=Number(document.querySelector(`[data-pid-field="${name}"]`)?.value);
    if(!Number.isFinite(value)) {$("#pid-status").textContent=`Enter a finite value for ${name}.`;return;}
    values[name]=value;
  }
  try {
    const result=await post("/api/pid",{values});
    if(state.pid) {state.pid.values=result.values;state.pid.pending=result.pending;}
    $("#pid-status").textContent=result.message??"PID override sent.";
  } catch(error) { $("#pid-status").textContent=`PID update failed: ${error.message}`; }
}
function loadPidBaseline() {
  if(state.pid?.baseline) {
    renderPidFields(state.pid.baseline);
    void applyPid();
  }
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
  state.ingestedSamples=0; state.lastSseAt=null; document.body.setAttribute("data-cache-ready","false");
  clearStore(); scheduleRender();
}
function stream() {
  if(state.events||state.paused||document.hidden)return;
  state.events=new EventSource("/api/stream");
  state.events.addEventListener("telemetry",event=>{
    const now=performance.now();
    const sseGapMs=state.lastSseAt==null?null:now-state.lastSseAt;
    state.lastSseAt=now;
    const sample=JSON.parse(event.data);
    sample.browserDiagnostics={sse_gap_ms:sseGapMs,sample_age_ms:0};
    append(sample);
  });
  state.events.addEventListener("status",event=>{
    const status=JSON.parse(event.data); state.status=status;
    if(status.pid_status) {
      if(state.pid) state.pid.last_status=status.pid_status;
      updatePidStatus(status.pid_status);
    }
    if(typeof status.display_run==="number"&&status.display_run!==state.displayRun)resetForDisplayRun(status.display_run);
    if(state.source)setStatus({...state.source,connection_state:String(status.connection_state??""),connection_message:String(status.connection_message??"")});
    updateMetrics(store.samples.at(-1)??null);
  });
  state.events.onerror=()=>{state.events?.close();state.events=null;if(!state.paused&&!document.hidden)window.setTimeout(stream,2000);};
}
async function post(path, body, headers) {
  const isBinary=typeof body==="string"||(typeof Blob!=="undefined"&&body instanceof Blob);
  const response=await fetch(path,{method:"POST",body:isBinary?body:JSON.stringify(body),headers:isBinary?headers:{"Content-Type":"application/json",...(headers??{})}});
  const result=await response.json();
  if(!response.ok||!result.ok)throw new Error(result.error??"Request failed."); if(result.source)setStatus(result.source); return result;
}
async function operation(id) {
  if(state.busy)return;
  const button=$("#"+id); const controls=[...document.querySelectorAll(".run-buttons button")];
  const messages={deploy:"Deploying to pi@rpi4…",start:state.source?.mode==="csv"?"Starting CSV playback…":"Starting balancer…",abort:state.source?.mode==="csv"?"Stopping CSV playback…":"Stopping balancer…"};
  state.busy=true; updateJoystickControls(); controls.forEach(control=>control.disabled=true); button.classList.add("working"); button.setAttribute("aria-busy","true"); $("#operation-result").textContent=messages[id];
  try {
    if(id==="abort"){stopJoystick();state.paused=true;stopPlayback();state.events?.close();state.events=null;}
    const response=await post(`/api/${id}`);
    if(id==="start"){
      state.paused=false; resetForDisplayRun(response.display_run??state.displayRun); state.status={...(state.status??{}),run_active:true,telemetry_connected:false}; stream();
      if(state.source?.mode==="csv"){await loadCompleteCache();startPlayback();}
    } else if(id==="abort") {state.status={...(state.status??{}),run_active:false,telemetry_connected:false};stopJoystick(false);}
    if(state.source)setStatus(state.source); $("#operation-result").textContent=response.message??"Done.";
  } catch(error){$("#operation-result").textContent=`Failed: ${error.message}`;}
  finally {state.busy=false;controls.forEach(control=>control.disabled=false);button.classList.remove("working");button.removeAttribute("aria-busy");if(state.source)setStatus(state.source);updateJoystickControls();}
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
  $("#app").innerHTML = `<div class="dashboard-shell"><aside class="left-rail"><div class="brand"><div class="brand-mark"><img src="/balancer-mark.svg" alt="Balancer Bot"></div><div><span class="brand-kicker">LOCAL CONSOLE</span><h1>Balancer</h1><p id="source" class="subtle">Live source · rpi4</p></div></div><div class="source-actions"><span class="rail-label">Data source</span><label class="file-button source-file">${icon("upload")}<span>Open CSV capture</span><input id="csv-file" type="file" accept=".csv,text/csv"></label></div><div class="rail-signature"><span></span>LOCAL TELEMETRY</div></aside><main class="dashboard-main"><section id="metrics" class="metrics"></section><section class="workspace"><div class="workspace-head"><div class="workspace-title">${icon("activity")}<h2>Telemetry</h2></div><div class="plot-controls"><div class="tabs">${groups.map(group=>`<button data-group="${group.id}" class="${state.visible.has(group.id)?"active":""}">${group.title}</button>`).join("")}</div><div class="toolbar-divider"></div><div class="ranges"><span id="timeline-state">Following latest</span><button id="follow" class="primary" disabled>Follow latest</button></div></div></div><div class="navigator"><div class="navigator-labels"><span id="navigator-start">0.0 s</span><strong>History window · <em id="window-duration">15 s</em></strong><span id="navigator-end">15.0 s</span></div><div id="navigator-track" class="navigator-track"><div id="navigator-fill" class="navigator-fill"></div><div id="navigator-window" class="navigator-window"><i class="handle left-handle"></i><span>DRAG</span><i class="handle right-handle"></i></div></div></div><div id="plots" class="plot-grid"></div></section></main><aside class="right-rail"><section class="run-controls" aria-label="Run controls"><div class="run-label"><span>${icon("radio")}Run control</span><strong id="connection" class="connection offline">Idle · waiting</strong></div><div class="run-buttons"><button id="deploy" class="secondary">${icon("rocket")}Deploy</button><button id="start" class="primary">${icon("play")}Start</button><button id="abort" class="danger">${icon("abort")}Abort</button></div><output id="operation-result" class="operation-result" aria-live="polite">Ready.</output></section><section class="run-notes"><span class="rail-label">Session</span><p>Start begins a new raw capture file and clears the display clock. Abort freezes presentation while capture stays passive.</p></section><section class="events"><strong>Latched flags</strong><span id="events-text">No telemetry received.</span></section></aside></div>`;
  updateMetrics(null);
  document.querySelectorAll("[data-group]").forEach(button=>button.onclick=()=>{const id=button.dataset.group;state.visible.has(id)?state.visible.delete(id):state.visible.add(id);button.classList.toggle("active");rebuild();});
  $("#follow").addEventListener("click",()=>{state.follow=true;state.end=null;scheduleRender();if(store.decimated){const [start,end]=viewRange();void loadDetail(start,end);}});
  (["deploy","start","abort"]).forEach(id=>$("#"+id).addEventListener("click",()=>void operation(id)));
  const track=$("#navigator-track"); track.addEventListener("pointerdown",navigatorPointer); track.addEventListener("pointermove",navigatorMove); track.addEventListener("pointerup",navigatorFinish); track.addEventListener("pointercancel",navigatorFinish);
  $("#csv-file").addEventListener("change",async event=>{const file=(event.currentTarget).files?.[0];if(!file)return;try{const result=await post("/api/source/csv",file,{"Content-Type":"text/csv","X-Filename":file.name});state.paused=true;stopPlayback();state.events?.close();state.events=null;clearStore();await loadInitial();await loadCompleteCache();$("#operation-result").textContent=result.message??"CSV loaded. Press Start to play.";}catch(error){$("#operation-result").textContent=`Failed: ${error.message}`;}});
  document.addEventListener("visibilitychange",()=>{if(document.hidden){stopJoystick();state.events?.close();state.events=null;}else if(!state.paused){void loadCompleteCache();stream();}});
}
function installControlPanels() {
  const fields=PID_FIELDS.map(([name,label])=>`<label class="pid-field"><span>${label}</span><input data-pid-field="${name}" type="number" step="${PID_MIN_STEP}" inputmode="decimal"></label>`).join("");
  $(".right-rail").insertAdjacentHTML("beforeend",`
    <section class="joystick-controls" aria-label="Joystick commands">
      <div class="panel-heading"><span class="rail-label">Joystick</span><strong>Drag for direction and speed; release to stop</strong></div>
      <div id="drive-pad" class="drive-pad" data-drive-pad role="group" tabindex="0" aria-label="Forward and turn joystick" aria-disabled="true" aria-valuetext="Neutral" style="--drive-pad-x:50%;--drive-pad-y:50%">
        <span class="drive-pad-label top">Forward</span><span class="drive-pad-label bottom">Reverse</span>
        <span class="drive-pad-label left">Left</span><span class="drive-pad-label right">Right</span>
        <span class="drive-pad-center">Neutral</span><i class="drive-pad-thumb"></i>
      </div>
      <small id="joystick-status" class="panel-status">Start a live run to enable joystick commands.</small>
    </section>
    <details class="pid-controls">
      <summary>Session PID tuning</summary>
      <p class="panel-help">Changes apply to the current balancer process only and are never written to pid.conf.</p>
      <div class="pid-actions"><button id="pid-apply" class="primary">Apply</button><button id="pid-load" class="secondary">Load</button></div>
      <div class="pid-fields">${fields}</div>
      <output id="pid-status" class="pid-status" aria-live="polite">Loading PID values…</output>
    </details>`);
  const pad=$("#drive-pad");
  pad.addEventListener("pointerdown",event=>{
      event.preventDefault();
      startJoystick(pad,event);
  });
  pad.addEventListener("pointermove",event=>{event.preventDefault();moveJoystick(pad,event);});
  pad.addEventListener("pointerup",event=>finishJoystick(pad,event));
  pad.addEventListener("pointercancel",()=>{if(state.joystick.pad===pad)stopJoystick();});
  pad.addEventListener("lostpointercapture",()=>{if(state.joystick.pad===pad)stopJoystick();});
  $("#pid-apply").addEventListener("click",()=>void applyPid());
  $("#pid-load").addEventListener("click",loadPidBaseline);
  document.querySelectorAll("[data-pid-field]").forEach(input=>input.addEventListener("input",()=>updatePidSpinnerStep(input)));
  window.addEventListener("blur",()=>stopJoystick());
  window.addEventListener("beforeunload",()=>stopJoystick());
  loadPid();
}
layout();
installControlPanels();
void(async()=>{
  const source=await(await fetch("/api/source")).json(); setStatus(source); state.displayRun=source.display_run??0;
  await loadInitial(); requestAnimationFrame(()=>{rebuild();setTimeout(()=>void loadCompleteCache(),0);}); stream();
})();
