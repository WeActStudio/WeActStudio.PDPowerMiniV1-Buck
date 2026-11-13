<script setup lang="ts">
import { computed, ref, watch } from "vue";
import { useI18n } from "vue-i18n";
import {
  store,
  setOcp,
  setCalibration,
  setDischarge,
  setBrightness,
  factoryReset,
  reboot,
  setPdVoltage,
  isPdInput,
  MODEL_LIMITS,
} from "../lib/store";

const { t } = useI18n();

type Setter = (on: boolean) => Promise<void>;
const toggles: { key: "ocp" | "calibration" | "discharge"; set: Setter }[] = [
  { key: "ocp", set: setOcp },
  { key: "calibration", set: setCalibration },
  { key: "discharge", set: setDischarge },
];

async function onToggle(e: Event, key: (typeof toggles)[number]["key"], set: Setter) {
  const el = e.target as HTMLInputElement;
  await set(el.checked);
  el.checked = store.settings[key] ?? false;
  el.indeterminate = store.settings[key] === null;
}

const brightness = ref(50);
watch(
  () => store.settings.brightness,
  (v) => {
    if (v !== null) brightness.value = v;
  },
  { immediate: true },
);

const pdLimits = computed(() => MODEL_LIMITS[store.model].pd);

// Seed the PD draft from the device once per connection, then leave it to the user.
const pdVoltage = ref(9);
let seeded = false;
watch(
  () => store.input?.pdVoltage,
  (v) => {
    if (v === undefined) seeded = false;
    else if (!seeded) {
      pdVoltage.value = v;
      seeded = true;
    }
  },
  { immediate: true },
);

function confirmThen(message: string, action: () => Promise<void>) {
  if (window.confirm(message)) void action();
}
</script>

<template>
  <article>
    <header class="panel-header">
      <strong>{{ t("settings.title") }}</strong>
    </header>

    <div class="settings-grid">
      <div>
        <div v-for="item in toggles" :key="item.key" class="setting">
          <label>
            <input
              type="checkbox"
              role="switch"
              :checked="store.settings[item.key] === true"
              :indeterminate="store.connected && store.settings[item.key] === null"
              :disabled="!store.connected"
              @change="onToggle($event, item.key, item.set)"
            />
            {{ t(`settings.${item.key}`) }}
            <span v-if="store.connected && store.settings[item.key] === null" class="muted">
              ({{ t("settings.unknown") }})
            </span>
          </label>
          <small>{{ t(`settings.${item.key}Hint`) }}</small>
        </div>
      </div>

      <div>
        <label class="setting">
          {{ t("settings.brightness") }}:
          <output>{{ store.connected && store.settings.brightness === null ? "—" : `${brightness}%` }}</output>
          <input
            v-model.number="brightness"
            type="range"
            min="1"
            max="100"
            :disabled="!store.connected"
            @change="setBrightness(brightness)"
          />
        </label>

        <form
          v-if="store.connected && isPdInput()"
          class="setting"
          @submit.prevent="setPdVoltage(Number(pdVoltage))"
        >
          <label for="pd-voltage">{{ t("info.pdVoltage") }}</label>
          <fieldset role="group">
            <input
              id="pd-voltage"
              v-model.number="pdVoltage"
              type="number"
              inputmode="decimal"
              step="0.1"
              :min="pdLimits[0]"
              :max="pdLimits[1]"
              required
            />
            <button type="submit">{{ t("info.set") }}</button>
          </fieldset>
          <small>{{ t("info.pdHint") }}</small>
        </form>
      </div>
    </div>

    <footer>
      <small class="muted">{{ t("settings.danger") }}</small>
      <div role="group">
        <button
          class="secondary outline"
          :disabled="!store.connected"
          @click="confirmThen(t('settings.rebootConfirm'), reboot)"
        >
          {{ t("settings.reboot") }}
        </button>
        <button
          class="contrast outline"
          :disabled="!store.connected"
          @click="confirmThen(t('settings.factoryResetConfirm'), factoryReset)"
        >
          {{ t("settings.factoryReset") }}
        </button>
      </div>
    </footer>
  </article>
</template>
