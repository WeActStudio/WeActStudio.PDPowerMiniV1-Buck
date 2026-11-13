import { ref } from "vue";

export type Theme = "light" | "dark";

const STORAGE_KEY = "pdpower.theme";
const systemDark = window.matchMedia("(prefers-color-scheme: dark)");

function savedTheme(): Theme | null {
  try {
    const v = localStorage.getItem(STORAGE_KEY);
    return v === "light" || v === "dark" ? v : null;
  } catch {
    return null;
  }
}

/** The user's explicit choice; null means "follow the OS". */
let chosen = savedTheme();

/** The theme currently in effect. */
export const theme = ref<Theme>(chosen ?? (systemDark.matches ? "dark" : "light"));

function apply(): void {
  // Pico CSS (and style.css) switch palettes on <html data-theme>.
  if (chosen) document.documentElement.dataset.theme = chosen;
  else delete document.documentElement.dataset.theme;
}

apply();

systemDark.addEventListener("change", (e) => {
  if (!chosen) theme.value = e.matches ? "dark" : "light";
});

export function toggleTheme(): void {
  chosen = theme.value === "dark" ? "light" : "dark";
  apply();
  theme.value = chosen;
  try {
    localStorage.setItem(STORAGE_KEY, chosen);
  } catch {
    /* storage blocked */
  }
}
