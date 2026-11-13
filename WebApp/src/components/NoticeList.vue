<script setup lang="ts">
import { useI18n } from "vue-i18n";
import { store, dismiss } from "../lib/store";

const { t } = useI18n();
</script>

<template>
  <div class="notices" aria-live="polite">
    <div v-for="n in store.notices" :key="n.id" class="notice" :class="n.kind" role="status">
      <span>{{ n.text }}</span>
      <button class="outline secondary" :aria-label="t('common.dismiss')" @click="dismiss(n.id)">×</button>
    </div>
  </div>
</template>
