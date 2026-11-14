import { defineConfig, type Plugin } from "vite";
import vue from "@vitejs/plugin-vue";

/**
 * Public URL the app is served from, used for the canonical/Open Graph/JSON-LD tags.
 * CI sets SITE_URL from the GitHub Pages deployment; this default is for local builds.
 */
const SITE_URL = (process.env.SITE_URL || "https://brisk4t.github.io/WeActStudio.PDPowerMiniV1-Buck/").replace(
  /\/*$/,
  "/",
);

/** Fills `__SITE_URL__` (canonical, Open Graph and JSON-LD tags) in index.html. */
function siteUrl(): Plugin {
  return {
    name: "pdpower-site-url",
    transformIndexHtml: (html) => html.replaceAll("__SITE_URL__", SITE_URL),
  };
}

export default defineConfig({
  // Relative base so the built `dist/` works from any sub-path (GitHub Pages, a NAS, etc.).
  base: "./",
  plugins: [vue(), siteUrl()],
});
