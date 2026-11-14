<script setup lang="ts">
import { onBeforeUnmount, onMounted, ref, watch } from "vue";
import { useI18n } from "vue-i18n";
import uPlot from "uplot";
import { onSample, type Sample } from "../lib/store";
import { theme } from "../theme";

const { t, locale } = useI18n();

/** 5 minutes at the 5 Hz poll rate. */
const MAX_POINTS = 1500;
const HEIGHT = 170;

const voltageEl = ref<HTMLDivElement>();
const currentEl = ref<HTMLDivElement>();
const hasData = ref(false);

// Column buffers shared by both plots (uPlot's AlignedData layout).
const times: number[] = [];
const volts: number[] = [];
const amps: number[] = [];

let plots: uPlot[] = [];
let resizeObserver: ResizeObserver | undefined;
let pendingFrame = 0;
const syncKey = `pdpower-${Math.random().toString(36).slice(2)}`;

function cssVar(name: string): string {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim();
}

function makeOptions(label: string, unit: string, color: string, width: number): uPlot.Options {
  const muted = cssVar("--pico-muted-color");
  const grid = cssVar("--pico-muted-border-color");
  return {
    width,
    height: HEIGHT,
    cursor: { sync: { key: syncKey }, drag: { x: false, y: false } },
    scales: {
      x: { time: true },
      y: {
        // Always include 0 so a small ripple doesn't look like a cliff.
        range: (_u, min, max) => [Math.min(0, min), max > 0 ? max * 1.1 : 1],
      },
    },
    axes: [
      {
        stroke: muted,
        space: 70,
        incrs: [1, 2, 5, 10, 15, 30, 60, 120, 300],
        values: (_u, ticks) => ticks.map((s) => new Date(s * 1000).toLocaleTimeString()),
        grid: { stroke: grid, width: 1 },
        ticks: { stroke: grid, width: 1 },
      },
      {
        stroke: muted,
        size: 72,
        grid: { stroke: grid, width: 1 },
        ticks: { stroke: grid, width: 1 },
        values: (_u, ticks) => ticks.map((v) => `${v.toFixed(2)} ${unit}`),
      },
    ],
    series: [
      {},
      {
        label,
        stroke: color,
        width: 2,
        points: { show: false },
        value: (_u, v) => (v == null ? "—" : `${v.toFixed(3)} ${unit}`),
      },
    ],
  };
}

function build() {
  destroy();
  if (!voltageEl.value || !currentEl.value) return;
  const width = voltageEl.value.clientWidth || 600;
  plots = [
    new uPlot(
      makeOptions(t("chart.voltage"), "V", cssVar("--viz-voltage"), width),
      [times, volts],
      voltageEl.value,
    ),
    new uPlot(
      makeOptions(t("chart.current"), "A", cssVar("--viz-current"), width),
      [times, amps],
      currentEl.value,
    ),
  ];
}

function destroy() {
  plots.forEach((p) => p.destroy());
  plots = [];
}

function redraw() {
  pendingFrame = 0;
  if (plots.length !== 2) return;
  plots[0].setData([times, volts]);
  plots[1].setData([times, amps]);
}

function push(s: Sample) {
  times.push(s.t);
  volts.push(s.voltage);
  amps.push(s.current);
  if (times.length > MAX_POINTS) {
    times.shift();
    volts.shift();
    amps.shift();
  }
  hasData.value = true;
  if (!pendingFrame) pendingFrame = requestAnimationFrame(redraw);
}

function clear() {
  times.length = volts.length = amps.length = 0;
  hasData.value = false;
  redraw();
}

function exportCsv() {
  const rows = ["time,voltage_V,current_A"];
  for (let i = 0; i < times.length; i++) {
    rows.push(`${new Date(times[i] * 1000).toISOString()},${volts[i].toFixed(3)},${amps[i].toFixed(3)}`);
  }
  const url = URL.createObjectURL(new Blob([rows.join("\n")], { type: "text/csv" }));
  const a = document.createElement("a");
  a.href = url;
  a.download = `pdpower-${new Date().toISOString().replace(/[:.]/g, "-")}.csv`;
  a.click();
  URL.revokeObjectURL(url);
}

let unsubscribe: (() => void) | undefined;

onMounted(() => {
  build();
  unsubscribe = onSample(push);
  resizeObserver = new ResizeObserver(() => {
    const width = voltageEl.value?.clientWidth;
    if (width) plots.forEach((p) => p.setSize({ width, height: HEIGHT }));
  });
  resizeObserver.observe(voltageEl.value!);
});

watch([locale, theme], build);

onBeforeUnmount(() => {
  unsubscribe?.();
  resizeObserver?.disconnect();
  if (pendingFrame) cancelAnimationFrame(pendingFrame);
  destroy();
});
</script>

<template>
  <article>
    <header class="panel-header">
      <strong>{{ t("chart.title") }}</strong>
      <div role="group" class="chart-actions">
        <button class="secondary outline" :disabled="!hasData" @click="clear">{{ t("chart.clear") }}</button>
        <button class="secondary outline" :disabled="!hasData" @click="exportCsv">{{ t("chart.export") }}</button>
      </div>
    </header>
    <div ref="voltageEl" class="chart"></div>
    <div ref="currentEl" class="chart"></div>
  </article>
</template>
