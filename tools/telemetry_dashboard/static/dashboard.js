"use strict";

const COLORS = ["#59b4ff", "#ffca58", "#8ee38e", "#ff8383", "#d795ff", "#74d7cf"];
const LIVE_WINDOW_SECONDS = 30;
const state = {
  catalog: null,
  charts: new Map(),
  samples: [],
  historySeconds: 600,
  paused: false,
  hidden: document.hidden,
  events: null,
  pending: null,
  renderPending: false,
  liveFollow: true,
  rangeSeconds: LIVE_WINDOW_SECONDS,
  xRange: null,
  visible: JSON.parse(localStorage.getItem("balancer-visible") || "{}"),
  syncing: false,
};

function get(obj, path) {
  return path.split(".").reduce((value, key) => value == null ? undefined : value[key], obj);
}

function visible(group, id) {
  return state.visible[`${group.id}.${id}`] !== false;
}

function setVisible(group, id, value) {
  state.visible[`${group.id}.${id}`] = value;
  localStorage.setItem("balancer-visible", JSON.stringify(state.visible));
}

function format(value) {
  return Number.isFinite(value) ? Number(value).toFixed(2) : "—";
}

function retainedRange() {
  if (!state.samples.length) return [0, LIVE_WINDOW_SECONDS];
  return [state.samples[0].received_at, state.samples[state.samples.length - 1].received_at];
}

function currentRange() {
  const [first, last] = retainedRange();
  if (state.liveFollow) return [Math.max(first, last - state.rangeSeconds), last];
  return state.xRange || [Math.max(first, last - state.rangeSeconds), last];
}

function clampRange(min, max) {
  const [first, last] = retainedRange();
  const available = Math.max(0.001, last - first);
  const width = Math.min(available, Math.max(0.05, max - min));
  const start = Math.max(first, Math.min(last - width, min));
  return [start, start + width];
}

function chartData(group) {
  const xs = state.samples.map(sample => sample.received_at);
  return [xs, ...group.series.map(series => state.samples.map(sample => {
    const value = get(sample, series[2]);
    return Number.isFinite(value) ? value : null;
  }))];
}

function updateViewState() {
  const label = state.paused ? "Paused" : state.liveFollow ? "Live" : "Exploring history";
  document.querySelector("#view-state").textContent = label;
  document.querySelector("#pause").textContent = state.paused ? "Resume" : "Pause";
  document.querySelectorAll("[data-range]").forEach(button => {
    button.classList.toggle("active", state.liveFollow && Number(button.dataset.range) === state.rangeSeconds);
  });
}

function renderLegend(group, chart) {
  const card = chart.root.closest(".plot-card");
  const index = chart.cursor.idx;
  if (index == null || !state.samples[index]) {
    card.querySelector(".cursor").textContent = "";
    card.querySelector(".legend").textContent = "";
    return;
  }
  const sample = state.samples[index];
  card.querySelector(".cursor").textContent = `t ${format(sample.t_sec)} s`;
  card.querySelector(".legend").textContent = group.series
    .filter(series => visible(group, series[0]))
    .map(series => `${series[1]} ${format(get(sample, series[2]))} ${group.unit}`)
    .join(" · ");
}

function applyRange(min, max, {manual = true} = {}) {
  const range = clampRange(min, max);
  if (manual) {
    state.liveFollow = false;
    state.xRange = range;
  }
  state.syncing = true;
  state.charts.forEach(({chart}) => chart.setScale("x", {min: range[0], max: range[1]}));
  state.syncing = false;
  updateViewState();
}

function updateCharts() {
  const [start, end] = currentRange();
  state.syncing = true;
  state.charts.forEach(({group, chart}) => {
    chart.setData(chartData(group));
    chart.setScale("x", {min: start, max: end});
  });
  state.syncing = false;
  document.querySelector("#samples").textContent = state.samples.length;
}

function scheduleRender() {
  if (state.renderPending || state.hidden || state.paused) return;
  state.renderPending = true;
  requestAnimationFrame(() => {
    state.renderPending = false;
    updateCharts();
  });
}

function queueSample(sample) {
  state.pending = sample;
  if (state.renderPending || state.hidden || state.paused) return;
  state.renderPending = true;
  requestAnimationFrame(() => {
    state.renderPending = false;
    const latest = state.pending;
    state.pending = null;
    if (latest) appendSample(latest);
  });
}

function appendSample(sample) {
  const previous = state.samples[state.samples.length - 1];
  if (previous && sample.sequence <= previous.sequence) return;
  state.samples.push(sample);
  const cutoff = sample.received_at - state.historySeconds;
  while (state.samples.length && state.samples[0].received_at < cutoff) state.samples.shift();
  updateCards(sample);
  updateCharts();
}

function updateCards(sample) {
  const fields = [
    ["Pitch", sample.attitude.pitch_deg, "deg"], ["Rate", sample.rate.filtered_pitch_rate_dps, "dps"],
    ["Command", sample.controller.command_sps, "sps"], ["Velocity", sample.motion.measured_velocity_sps, "sps"],
    ["IMU age", sample.timing.imu_age_ms, "ms"], ["Feedback age", sample.timing.feedback_age_ms, "ms"],
  ];
  document.querySelector("#values").innerHTML = fields
    .map(([name, value, unit]) => `<div>${name}<br><strong>${format(value)} ${unit}</strong></div>`).join("");
}

function updateStatus(status) {
  const connected = document.querySelector("#connection");
  connected.textContent = status.connected ? "Connected" : "Disconnected";
  connected.className = status.connected ? "online" : "offline";
  document.querySelector("#pi").textContent = status.pi;
  document.querySelector("#raw-rate").textContent = `${status.raw_packet_rate_hz} Hz`;
  document.querySelector("#display-rate").textContent = `${status.display_rate_hz} Hz`;
  document.querySelector("#last-packet").textContent = status.last_packet_age_ms == null ? "—" : `${status.last_packet_age_ms.toFixed(0)} ms`;
  document.querySelector("#malformed").textContent = status.malformed_packets;
  for (const [key, id] of [["controller", "controller-flag"], ["saturation", "saturation-flag"], ["actuator", "actuator-flag"]]) {
    document.querySelector(`#${id}`).textContent = `0x${status.latched_flags[key].toString(16)}`;
  }
}

function attachNavigation(chart) {
  chart.root.addEventListener("wheel", event => {
    event.preventDefault();
    const [start, end] = currentRange();
    const rect = chart.root.getBoundingClientRect();
    const ratio = Math.max(0, Math.min(1, (event.clientX - rect.left) / rect.width));
    const pivot = start + (end - start) * ratio;
    const width = (end - start) * (event.deltaY < 0 ? 0.75 : 1.35);
    applyRange(pivot - width * ratio, pivot + width * (1 - ratio));
  }, {passive: false});

  let pan = null;
  chart.root.addEventListener("pointerdown", event => {
    if (event.button !== 1 && !event.shiftKey) return;
    event.preventDefault();
    chart.root.setPointerCapture(event.pointerId);
    pan = {x: event.clientX, range: currentRange()};
  });
  chart.root.addEventListener("pointermove", event => {
    if (!pan) return;
    const width = chart.root.getBoundingClientRect().width || 1;
    const shift = (event.clientX - pan.x) / width * (pan.range[1] - pan.range[0]);
    applyRange(pan.range[0] - shift, pan.range[1] - shift);
  });
  chart.root.addEventListener("pointerup", () => { pan = null; });
  chart.root.addEventListener("dblclick", () => setLiveFollow());
}

function buildPlots() {
  const root = document.querySelector("#plots");
  const template = document.querySelector("#plot-template");
  root.textContent = "";
  state.catalog.groups.forEach(group => {
    const node = template.content.firstElementChild.cloneNode(true);
    node.dataset.group = group.id;
    node.querySelector("h2").textContent = `${group.title} (${group.unit})`;
    const controls = node.querySelector(".series");
    group.series.forEach(([id, label], index) => {
      const check = document.createElement("input");
      check.type = "checkbox";
      check.checked = visible(group, id);
      const wrap = document.createElement("label");
      wrap.style.color = COLORS[index % COLORS.length];
      wrap.append(check, ` ${label}`);
      check.addEventListener("change", () => {
        setVisible(group, id, check.checked);
        state.charts.get(group.id).chart.setSeries(index + 1, {show: check.checked});
      });
      controls.append(wrap);
    });
    root.append(node);
    const chart = new uPlot({
      width: node.querySelector(".chart").clientWidth || 420,
      height: 260,
      scales: {x: {time: false}, y: {auto: true}},
      axes: [
        {stroke: "#9eabb9", grid: {stroke: "#263342"}, values: (_u, values) => {
          const origin = state.samples[0] ? state.samples[0].received_at : 0;
          return values.map(value => `+${(value - origin).toFixed(1)} s`);
        }},
        {stroke: "#9eabb9", grid: {stroke: "#263342"}, label: group.unit},
      ],
      series: [
        {}, ...group.series.map((series, index) => ({label: series[1], stroke: COLORS[index % COLORS.length], width: 1.5, show: visible(group, series[0])})),
      ],
      cursor: {sync: {key: "balancer-telemetry", setScale: false}, drag: {x: true, y: false, setScale: true}},
      hooks: {
        setCursor: [plot => renderLegend(group, plot)],
        setScale: [plot => {
          if (!state.syncing && plot.scales.x.min != null) applyRange(plot.scales.x.min, plot.scales.x.max);
        }],
      },
    }, chartData(group), node.querySelector(".chart"));
    attachNavigation(chart);
    state.charts.set(group.id, {group, chart});
  });
  window.addEventListener("resize", () => state.charts.forEach(({chart}) => chart.setSize({width: chart.root.parentElement.clientWidth, height: 260})));
}

async function loadHistory(seconds = state.rangeSeconds) {
  const history = await (await fetch(`/api/history?seconds=${encodeURIComponent(seconds)}`)).json();
  state.historySeconds = history.history_seconds;
  state.samples = history.samples;
  if (state.samples.length) updateCards(state.samples[state.samples.length - 1]);
  if (!state.hidden && !state.paused) updateCharts();
}

function closeStream() {
  if (state.events) state.events.close();
  state.events = null;
}

function openStream() {
  if (state.events || state.paused || state.hidden) return;
  state.events = new EventSource("/api/stream");
  state.events.addEventListener("telemetry", event => queueSample(JSON.parse(event.data)));
  state.events.addEventListener("status", event => updateStatus(JSON.parse(event.data)));
}

async function setPaused(paused) {
  state.paused = paused;
  if (paused) closeStream();
  else {
    await loadHistory(state.rangeSeconds);
    setLiveFollow();
    openStream();
  }
  updateViewState();
}

function setLiveFollow(seconds = state.rangeSeconds) {
  state.rangeSeconds = seconds;
  state.liveFollow = true;
  state.xRange = null;
  updateCharts();
  updateViewState();
}

async function selectRange(seconds) {
  state.rangeSeconds = seconds;
  await loadHistory(seconds);
  setLiveFollow(seconds);
}

async function setupDeploy() {
  const info = await (await fetch("/api/deploy-info")).json();
  const panel = document.querySelector("#deploy-panel");
  const result = document.querySelector("#deploy-result");
  if (!info.enabled) {
    panel.hidden = true;
    return;
  }
  panel.hidden = false;
  const action = (path, label, afterSuccess) => async () => {
    result.textContent = `${label}…`;
    const response = await fetch(path, {method: "POST"});
    const body = await response.json();
    result.textContent = body.ok ? body.message : `Failed: ${body.error}`;
    if (body.ok && afterSuccess) await afterSuccess();
  };
  document.querySelector("#deploy-current").onclick = action("/api/deploy", "Deploying");
  document.querySelector("#start-bot").onclick = action("/api/start", "Starting", () => setPaused(false));
  document.querySelector("#abort-bot").onclick = action("/api/abort", "Aborting", () => setPaused(true));
  if (!info.binary_exists) result.textContent = `Build missing: ${info.binary}`;
}

async function initialize() {
  state.catalog = await (await fetch("signal_catalog.json")).json();
  buildPlots();
  await loadHistory(LIVE_WINDOW_SECONDS);
  updateViewState();
  openStream();
  document.querySelector("#pause").onclick = () => setPaused(!state.paused);
  document.querySelector("#clear").onclick = () => {
    state.samples = [];
    fetch("/api/clear-flags");
    updateCharts();
  };
  document.querySelector("#reset").onclick = () => setLiveFollow();
  document.querySelectorAll("[data-range]").forEach(button => {
    button.onclick = () => selectRange(Number(button.dataset.range));
  });
  document.addEventListener("visibilitychange", async () => {
    state.hidden = document.hidden;
    if (state.hidden) closeStream();
    else if (!state.paused) {
      await loadHistory(state.rangeSeconds);
      openStream();
    }
  });
  setupDeploy().catch(() => {});
}

initialize().catch(error => {
  document.querySelector("#connection").textContent = `Dashboard failed: ${error.message}`;
});
