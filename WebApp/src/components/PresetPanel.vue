<script setup lang="ts">
import { computed, ref, watch } from "vue";
import { useI18n } from "vue-i18n";
import {
  store,
  selectPreset,
  writePreset,
  refreshAll,
  saveConfig,
  MODEL_LIMITS,
  PRESET_COUNT,
} from "../lib/store";

const { t } = useI18n();

const limits = computed(() => MODEL_LIMITS[store.model]);
const active = computed(() => store.presets[store.activePreset]);

// Editable draft of the active preset; re-synced whenever the device reports new values.
const voltage = ref(0);
const current = ref(0);
watch(
  () => [store.activePreset, active.value.voltage, active.value.current],
  () => {
    voltage.value = active.value.voltage;
    current.value = active.value.current;
  },
  { immediate: true },
);

const dirty = computed(
  () => voltage.value !== active.value.voltage || current.value !== active.value.current,
);

function apply() {
  void writePreset(store.activePreset, Number(voltage.value), Number(current.value));
}
</script>

<template>
  <article>
    <header class="panel-header">
      <strong>{{ t("presets.title") }}</strong>
    </header>

    <div role="group" class="preset-buttons">
      <button
        v-for="id in PRESET_COUNT"
        :key="id"
        :class="{ outline: store.activePreset !== id - 1 }"
        :aria-pressed="store.activePreset === id - 1"
        :disabled="!store.connected || (store.output.enabled && store.activePreset !== id - 1)"
        @click="selectPreset(id - 1)"
      >
        M{{ id - 1 }}
      </button>
    </div>
    <small v-if="store.connected && store.output.enabled">{{ t("presets.lockedHint") }}</small>

    <form class="grid preset-form" @submit.prevent="apply">
      <label>
        {{ t("presets.voltage") }}
        <input
          v-model.number="voltage"
          type="number"
          inputmode="decimal"
          step="0.01"
          :min="limits.voltage[0]"
          :max="limits.voltage[1]"
          :disabled="!store.connected"
          required
        />
      </label>
      <label>
        {{ t("presets.current") }}
        <input
          v-model.number="current"
          type="number"
          inputmode="decimal"
          step="0.001"
          :min="limits.current[0]"
          :max="limits.current[1]"
          :disabled="!store.connected"
          required
        />
      </label>
      <button type="submit" :disabled="!store.connected || !dirty">{{ t("presets.apply") }}</button>
    </form>

    <footer class="actions">
      <small>{{ t("presets.volatileHint") }}</small>
      <div role="group">
        <button class="secondary outline" :disabled="!store.connected" @click="refreshAll">
          {{ t("presets.refresh") }}
        </button>
        <button :disabled="!store.connected" @click="saveConfig">{{ t("presets.save") }}</button>
      </div>
    </footer>
  </article>
</template>
