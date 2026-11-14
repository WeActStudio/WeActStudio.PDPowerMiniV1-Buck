<script setup lang="ts">
import { computed } from "vue";
import { useI18n } from "vue-i18n";
import { store, setOutputEnabled } from "../lib/store";

const { t } = useI18n();

const power = computed(() => store.measured.voltage * store.measured.current);
/** Set-points of the selected preset (M0..M4). */
const setpoint = computed(() => store.presets[store.activePreset]);

async function onToggle(e: Event) {
  const el = e.target as HTMLInputElement;
  await setOutputEnabled(el.checked);
  el.checked = store.output.enabled; // snap back if the device didn't follow
}
</script>

<template>
  <article>
    <header class="panel-header">
      <strong>{{ t("output.title") }}</strong>
      <span v-if="store.stale" class="badge oc">{{ t("output.stale") }}</span>
      <span
        v-else-if="store.connected && store.output.enabled"
        class="badge"
        :class="store.output.mode.toLowerCase()"
      >{{ t(`output.mode.${store.output.mode}`) }}</span>
    </header>

    <div class="readouts">
      <div>
        <small>{{ t("output.voltage") }}</small>
        <div class="setpoint">{{ t("output.set") }} {{ store.connected ? `${setpoint.voltage.toFixed(3)} V` : "—" }}</div>
        <div class="readout">{{ store.measured.voltage.toFixed(3) }}<span>V</span></div>
      </div>
      <div>
        <small>{{ t("output.current") }}</small>
        <div class="setpoint">{{ t("output.set") }} {{ store.connected ? `${setpoint.current.toFixed(3)} A` : "—" }}</div>
        <div class="readout">{{ store.measured.current.toFixed(3) }}<span>A</span></div>
      </div>
      <div>
        <small>{{ t("output.power") }}</small>
        <div class="readout secondary-readout">{{ power.toFixed(2) }}<span>W</span></div>
      </div>
    </div>

    <footer>
      <label class="output-switch">
        <input
          type="checkbox"
          role="switch"
          :checked="store.output.enabled"
          :disabled="!store.connected"
          @change="onToggle"
        />
        {{ t("output.enable") }}
      </label>
    </footer>
  </article>
</template>
