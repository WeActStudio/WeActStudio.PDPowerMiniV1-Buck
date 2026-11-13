<script setup lang="ts">
import { useI18n } from "vue-i18n";
import { store } from "./lib/store";
import ConnectionBar from "./components/ConnectionBar.vue";
import NoticeList from "./components/NoticeList.vue";
import OutputPanel from "./components/OutputPanel.vue";
import PresetPanel from "./components/PresetPanel.vue";
import LiveChart from "./components/LiveChart.vue";
import SettingsPanel from "./components/SettingsPanel.vue";

const { t } = useI18n();
</script>

<template>
  <ConnectionBar />
  <main class="container">
    <article v-if="!store.supported" class="unsupported">
      <strong>{{ t("unsupported.title") }}</strong>
      <p>{{ t("unsupported.body") }}</p>
    </article>

    <NoticeList />

    <div class="panels">
      <OutputPanel />
      <PresetPanel />
    </div>
    <LiveChart />
    <SettingsPanel />
  </main>
</template>
