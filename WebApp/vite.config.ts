import { defineConfig } from "vite";
import vue from "@vitejs/plugin-vue";

export default defineConfig({
  // Relative base so the built `dist/` works from any sub-path (GitHub Pages, a NAS, etc.).
  base: "./",
  plugins: [vue()],
});
