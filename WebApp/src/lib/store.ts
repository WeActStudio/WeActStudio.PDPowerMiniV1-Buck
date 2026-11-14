import { reactive, readonly } from "vue";
import { PdPowerDevice, USB_FILTER, type ConnectOptions } from "./device";
import { InputState, versionAtLeast, type OutputMode } from "./protocol";
import { i18n } from "../i18n";

const t = (key: string, named?: Record<string, unknown>) =>
  named ? i18n.global.t(key, named) : i18n.global.t(key);

/** Hardware variant, identified by the first two characters of the serial number. */
export type Model = "V1" | "V1.1";

export const MODEL_LIMITS: Record<Model, { voltage: [number, number]; current: [number, number]; pd: [number, number] }> = {
  V1: { voltage: [0.5, 20], current: [0.005, 3], pd: [8, 24] },
  "V1.1": { voltage: [0.5, 24], current: [0.005, 3], pd: [8, 28] },
};

/** Firmware that added INPUT_STATE / PD voltage commands. */
const INPUT_STATE_MIN_FW: Record<Model, string> = { V1: "1.0.2.0", "V1.1": "1.0.1.0" };

export const PRESET_COUNT = 5;
const POLL_INTERVAL_MS = 200;
const MAX_MISSED_POLLS = 10;

export interface Sample {
  t: number; // unix seconds
  voltage: number;
  current: number;
}

export interface Notice {
  id: number;
  kind: "error" | "info";
  text: string;
}

const state = reactive({
  supported: typeof navigator !== "undefined" && "serial" in navigator,
  connected: false,
  connecting: false,
  /** True while the device has stopped answering polls. */
  stale: false,
  info: { name: "", version: "", serial: "" },
  model: "V1" as Model,
  output: { enabled: false, mode: "OK" as OutputMode },
  measured: { voltage: 0, current: 0 },
  presets: Array.from({ length: PRESET_COUNT }, () => ({ voltage: 0, current: 0 })),
  activePreset: 0,
  supportsInput: false,
  input: null as null | { state: number; voltage: number; pdVoltage: number },
  settings: {
    ocp: null as boolean | null,
    calibration: null as boolean | null,
    discharge: null as boolean | null,
    brightness: null as number | null,
  },
  notices: [] as Notice[],
});

export const store = readonly(state);

const device = new PdPowerDevice();
const sampleListeners = new Set<(s: Sample) => void>();
let pollGeneration = 0;
let noticeId = 0;

export function onSample(fn: (s: Sample) => void): () => void {
  sampleListeners.add(fn);
  return () => sampleListeners.delete(fn);
}

export function notify(text: string, kind: Notice["kind"] = "error"): void {
  const id = ++noticeId;
  state.notices.push({ id, kind, text });
  if (kind === "info") setTimeout(() => dismiss(id), 4000);
}

export function dismiss(id: number): void {
  const i = state.notices.findIndex((n) => n.id === id);
  if (i >= 0) state.notices.splice(i, 1);
}

const errorText = (err: unknown) => (err instanceof Error ? err.message : String(err));

/** Run a device action, surfacing failures as a notice instead of throwing. */
async function guarded<T>(fn: () => Promise<T>): Promise<T | undefined> {
  if (!state.connected) {
    notify(t("errors.notOpen"));
    return undefined;
  }
  try {
    return await fn();
  } catch (err) {
    notify(errorText(err));
    return undefined;
  }
}

/** Resolve to undefined instead of rejecting — for optional reads older firmware may not answer. */
const optional = <T>(p: Promise<T>) => p.catch(() => undefined);

function resetState(): void {
  state.connected = false;
  state.stale = false;
  state.info = { name: "", version: "", serial: "" };
  state.output = { enabled: false, mode: "OK" };
  state.measured = { voltage: 0, current: 0 };
  state.supportsInput = false;
  state.input = null;
  state.settings = { ocp: null, calibration: null, discharge: null, brightness: null };
}

device.onDisconnect = () => {
  pollGeneration++;
  resetState();
  notify(t("errors.disconnected"));
};

// ---- Connection ------------------------------------------------------------------------------

export async function connect(opts: ConnectOptions & { anyPort: boolean }): Promise<void> {
  if (state.connected || state.connecting) return;
  let port: SerialPort;
  try {
    port = await navigator.serial.requestPort(opts.anyPort ? {} : { filters: [USB_FILTER] });
  } catch {
    return; // user dismissed the picker
  }
  state.connecting = true;
  try {
    await device.open(port, opts);
    state.connected = true;
    await refreshAll();
    startPolling();
  } catch (err) {
    notify(`${t("errors.openFailed")}: ${errorText(err)}`);
    await device.close();
    resetState();
  } finally {
    state.connecting = false;
  }
}

export async function disconnect(): Promise<void> {
  pollGeneration++;
  await device.close();
  resetState();
}

/** Read everything the UI shows. Mirrors the original tool's startup sequence. */
export async function refreshAll(): Promise<void> {
  await guarded(async () => {
    state.info.name = await device.whoAmI();
    state.info.version = await device.firmwareVersion();
    state.info.serial = await device.serialNumber();
    state.model = state.info.serial.startsWith("B3") ? "V1.1" : "V1";
    state.supportsInput = versionAtLeast(state.info.version, INPUT_STATE_MIN_FW[state.model]);

    state.output = await device.outputState();
    for (let id = 0; id < PRESET_COUNT; id++) await readPreset(id);
    state.activePreset = await device.activePreset();
    await readMeasured();
    if (state.supportsInput) await readInput();

    state.settings.ocp = (await optional(device.ocpEnabled())) ?? null;
    state.settings.calibration = (await optional(device.calibrationEnabled())) ?? null;
    state.settings.brightness = (await optional(device.brightness())) ?? null;
  });
}

async function readPreset(id: number): Promise<void> {
  const p = await device.preset(id);
  if (p.id < PRESET_COUNT) state.presets[p.id] = { voltage: p.mV / 1000, current: p.mA / 1000 };
}

/** Selected preset plus its set-points (both can be changed with the device's own buttons). */
async function readActivePreset(): Promise<void> {
  const id = await device.activePreset();
  if (id >= PRESET_COUNT) return;
  state.activePreset = id;
  await readPreset(id);
}

async function readMeasured(): Promise<void> {
  const m = await device.measuredOutput();
  state.measured = { voltage: m.mV / 1000, current: m.mA / 1000 };
  const sample: Sample = { t: Date.now() / 1000, ...state.measured };
  sampleListeners.forEach((fn) => fn(sample));
}

async function readInput(): Promise<void> {
  const i = await device.inputState();
  state.input = { state: i.state, voltage: i.mV / 1000, pdVoltage: i.pdVolts };
}

function startPolling(): void {
  const generation = ++pollGeneration;
  let tick = 0;
  let missed = 0;
  const loop = async () => {
    while (generation === pollGeneration && device.isOpen) {
      const started = performance.now();
      try {
        await readMeasured(); // every 200 ms
        if (tick % 2 === 0) state.output = await device.outputState(); // every 400 ms
        if (state.supportsInput && tick % 5 === 0) await readInput(); // every 1 s
        if (tick % 5 === 3) await readActivePreset(); // every 1 s: picks up changes made on the device
        missed = 0;
        state.stale = false;
      } catch {
        if (generation !== pollGeneration) return;
        if (++missed >= MAX_MISSED_POLLS) state.stale = true;
      }
      tick++;
      await new Promise((r) => setTimeout(r, Math.max(0, POLL_INTERVAL_MS - (performance.now() - started))));
    }
  };
  void loop();
}

// ---- Output & presets ------------------------------------------------------------------------

export async function setOutputEnabled(on: boolean): Promise<void> {
  await guarded(async () => {
    await device.setOutputEnabled(on);
    state.output = await device.outputState();
  });
}

export async function selectPreset(id: number): Promise<void> {
  if (state.output.enabled) {
    notify(t("errors.outputOnPreset"));
    return;
  }
  await guarded(async () => {
    await device.setActivePreset(id);
    state.activePreset = await device.activePreset();
  });
}

export async function writePreset(id: number, voltage: number, current: number): Promise<boolean> {
  const limits = MODEL_LIMITS[state.model];
  for (const [value, [min, max]] of [
    [voltage, limits.voltage],
    [current, limits.current],
  ] as const) {
    if (!Number.isFinite(value) || value < min || value > max) {
      notify(t("errors.outOfRange", { min, max }));
      return false;
    }
  }
  const ok = await guarded(async () => {
    await device.setPreset(id, Math.round(voltage * 1000), Math.round(current * 1000));
    await readPreset(id);
    return true;
  });
  return ok ?? false;
}

export async function setPdVoltage(volts: number): Promise<void> {
  const [min, max] = MODEL_LIMITS[state.model].pd;
  if (!Number.isFinite(volts) || volts < min || volts > max) {
    notify(t("errors.outOfRange", { min, max }));
    return;
  }
  if (state.output.enabled || state.measured.voltage >= 5) {
    notify(t("errors.pdVoltage"));
    return;
  }
  await guarded(async () => {
    await device.setPdVoltage(volts);
    await readInput();
  });
}

export const isPdInput = () => state.input?.state === InputState.PD;

// ---- Settings --------------------------------------------------------------------------------

export async function setOcp(on: boolean): Promise<void> {
  await guarded(async () => {
    await device.setOcpEnabled(on);
    state.settings.ocp = (await optional(device.ocpEnabled())) ?? on;
  });
}

export async function setCalibration(on: boolean): Promise<void> {
  await guarded(async () => {
    await device.setCalibrationEnabled(on);
    state.settings.calibration = (await optional(device.calibrationEnabled())) ?? on;
  });
}

export async function setDischarge(on: boolean): Promise<void> {
  await guarded(async () => {
    await device.setDischargeEnabled(on);
    state.settings.discharge = on; // no read command for this setting
  });
}

export async function setBrightness(percent: number): Promise<void> {
  await guarded(async () => {
    await device.setBrightness(percent);
    state.settings.brightness = (await optional(device.brightness())) ?? percent;
  });
}

export async function saveConfig(): Promise<void> {
  const ok = await guarded(async () => (await device.saveConfig(), true));
  if (ok) notify(t("messages.saved"), "info");
}

export async function factoryReset(): Promise<void> {
  await guarded(async () => {
    await device.factoryReset();
    await refreshAll();
  });
}

export async function reboot(): Promise<void> {
  await guarded(() => device.reboot());
}
