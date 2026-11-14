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

const REPO_URL = "https://github.com/Brisk4t/WeActStudio.PDPowerMiniV1-Buck";
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
  <footer class="container site-footer">
    <small>
      {{ t("footer.about") }}
      <a :href="REPO_URL" target="_blank" rel="noopener">{{ t("footer.source") }}</a>
    </small>
  </footer>
</template>
