import { createApp } from "vue";
import "@picocss/pico/css/pico.min.css";
import "uplot/dist/uPlot.min.css";
import "./style.css";
import App from "./App.vue";
import { i18n } from "./i18n";

document.documentElement.lang = i18n.global.locale.value;
createApp(App).use(i18n).mount("#app");
