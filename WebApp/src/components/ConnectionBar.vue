<script setup lang="ts">
import { computed, ref } from "vue";
import { useI18n } from "vue-i18n";
import { store, connect, disconnect } from "../lib/store";
import { INPUT_STATE_KEYS } from "../lib/protocol";
import { DEFAULT_BAUD_RATE, UART_BAUD_RATES } from "../lib/device";
import { LANGUAGES, setLanguage, type Language } from "../i18n";
import logoUrl from "../assets/logo.png";
import { theme, toggleTheme } from "../theme";

const { t, locale } = useI18n();

const baudRate = ref<number>(DEFAULT_BAUD_RATE);
const crc = ref(false);
const anyPort = ref(false);

/** Compact "name · firmware · SN · input" line shown under the title while connected. */
const deviceLine = computed(() => {
  const { name, version, serial } = store.info;
  const parts = [name, version, serial && `SN ${serial}`];
  if (store.input) {
    const key = INPUT_STATE_KEYS[store.input.state];
    parts.push(`${key ? t(`inputStates.${key}`) : "?"} ${store.input.voltage.toFixed(2)} V`);
  }
  return parts.filter(Boolean).join(" · ");
});

const optionsEl = ref<HTMLDetailsElement>();

function onConnectClick() {
  if (optionsEl.value) optionsEl.value.open = false;
  if (store.connected) void disconnect();
  else void connect({ baudRate: baudRate.value, crc: crc.value, anyPort: anyPort.value });
}
</script>

<template>
  <header class="container">
    <nav>
      <ul>
        <li class="brand">
          <img :src="logoUrl" alt="" width="32" height="32" />
          <span>
            <strong>{{ t("app.title") }}</strong>
            <small v-if="store.connected && deviceLine" class="device-line">{{ deviceLine }}</small>
            <small v-else class="subtitle">{{ t("app.subtitle") }}</small>
          </span>
        </li>
      </ul>
      <ul>
        <li>
          <select
            :aria-label="t('connection.language')"
            :value="locale"
            @change="setLanguage(($event.target as HTMLSelectElement).value as Language)"
          >
            <option v-for="l in LANGUAGES" :key="l.code" :value="l.code">{{ l.label }}</option>
          </select>
        </li>
        <li>
          <button
            class="secondary outline theme-toggle"
            :aria-label="theme === 'dark' ? t('theme.toLight') : t('theme.toDark')"
            :title="theme === 'dark' ? t('theme.toLight') : t('theme.toDark')"
            @click="toggleTheme"
          >
            <!-- sun (shown in dark mode) -->
            <svg v-if="theme === 'dark'" viewBox="0 0 24 24" aria-hidden="true">
              <circle cx="12" cy="12" r="4" />
              <path d="M12 2v2M12 20v2M4.93 4.93l1.41 1.41M17.66 17.66l1.41 1.41M2 12h2M20 12h2M4.93 19.07l1.41-1.41M17.66 6.34l1.41-1.41" />
            </svg>
            <!-- moon (shown in light mode) -->
            <svg v-else viewBox="0 0 24 24" aria-hidden="true">
              <path d="M21 12.79A9 9 0 1 1 11.21 3 7 7 0 0 0 21 12.79z" />
            </svg>
          </button>
        </li>
        <li v-if="!store.connected && store.supported">
          <details ref="optionsEl" class="dropdown connection-dropdown">
            <summary role="button" class="secondary outline" :title="t('connection.options')">{{ t("connection.optionsShort") }}</summary>
            <ul>
              <li>
                <label>
                  {{ t("connection.baudRate") }}
                  <select v-model.number="baudRate">
                    <option v-for="b in UART_BAUD_RATES" :key="b" :value="b">{{ b }}</option>
                  </select>
                </label>
              </li>
              <li>
                <label>
                  <input v-model="anyPort" type="checkbox" role="switch" />
                  {{ t("connection.anyPort") }}
                </label>
                <small>{{ t("connection.anyPortHint") }}</small>
              </li>
              <li>
                <label>
                  <input v-model="crc" type="checkbox" role="switch" />
                  {{ t("connection.crc") }}
                </label>
                <small>{{ t("connection.crcHint") }}</small>
              </li>
            </ul>
          </details>
        </li>
        <li>
          <button
            :class="{ secondary: store.connected }"
            :disabled="!store.supported || store.connecting"
            :aria-busy="store.connecting"
            @click="onConnectClick"
          >
            {{
              store.connecting
                ? t("connection.connecting")
                : store.connected
                  ? t("connection.disconnect")
                  : t("connection.connect")
            }}
          </button>
        </li>
      </ul>
    </nav>

  </header>
</template>
