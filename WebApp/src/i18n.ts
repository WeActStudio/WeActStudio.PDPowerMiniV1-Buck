import { createI18n } from "vue-i18n";
import en from "./locales/en";
import zh from "./locales/zh";
import ja from "./locales/ja";
import ru from "./locales/ru";

export const LANGUAGES = [
  { code: "en", label: "English" },
  { code: "zh", label: "中文" },
  { code: "ja", label: "日本語" },
  { code: "ru", label: "Русский" },
] as const;

export type Language = (typeof LANGUAGES)[number]["code"];

const STORAGE_KEY = "pdpower.language";

function initialLanguage(): Language {
  let saved: string | null = null;
  try {
    saved = localStorage.getItem(STORAGE_KEY);
  } catch {
    /* storage blocked */
  }
  const candidates = [saved, ...navigator.languages.map((l) => l.slice(0, 2))];
  return (candidates.find((c) => LANGUAGES.some((l) => l.code === c)) as Language) ?? "en";
}

export const i18n = createI18n({
  legacy: false,
  locale: initialLanguage(),
  fallbackLocale: "en",
  messages: { en, zh, ja, ru },
});

export function setLanguage(lang: Language): void {
  i18n.global.locale.value = lang;
  document.documentElement.lang = lang;
  try {
    localStorage.setItem(STORAGE_KEY, lang);
  } catch {
    /* storage blocked */
  }
}
