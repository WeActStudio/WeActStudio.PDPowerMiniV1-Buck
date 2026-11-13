# PD Power Mini — Web App

A static, browser-only replacement for the *PD Power Communication Tool* (`Tools/`) and the
`com_pdpower.py` script. It talks to the WeAct Studio PD Power Mini V1 / V1.1 directly over
[Web Serial](https://developer.mozilla.org/docs/Web/API/Web_Serial_API). You don't need to install anything.

## Features

- Live voltage, current and power readout with the CC/OC mode indicator, plus an output on/off switch
- Live voltage and current charts (last 5 minutes) with CSV export
- Output presets M0–M4: select, edit voltage and current limit, save to flash
- Device info (name, firmware, serial number, model, input type and voltage)
- PD request voltage (firmware ≥ 1.0.2.0 on V1, ≥ 1.0.1.0 on V1.1)
- Device settings: over-current protection, calibration offsets, over-current discharge, display brightness
- Restart and factory reset
- English, 中文, 日本語 and Русский, plus light and dark themes
- UART mode: pick a baud rate, turn on CRC-8 framing and list every serial port to use a USB-UART adapter

## Browser support

Web Serial works in **desktop Chromium browsers** (Chrome, Edge, Opera, etc.). The page must be served over
**HTTPS or `http://localhost`**. Firefox and Safari don't support Web Serial.

On Linux, your user needs access to the serial device (usually membership of the `dialout`
or `uucp` group).

## Development

```sh
npm install
npm run dev        # http://localhost:5173
npm test           # protocol + transport unit tests
npm run build      # typecheck + static bundle in dist/
npm run preview    # serve dist/ locally
```

`dist/` uses relative paths, so you can host it on any static server or sub-path (GitHub Pages, a NAS, etc.).

## Stack

| Concern | Library |
|---|---|
| UI framework | [Vue 3](https://vuejs.org) |
| Styling / components | [Pico CSS](https://picocss.com) |
| Charts | [uPlot](https://github.com/leeoniya/uPlot) |
| Translations | [vue-i18n](https://vue-i18n.intlify.dev) |
| Build / test | [Vite](https://vite.dev), [Vitest](https://vitest.dev) |
| Serial | Browser-native Web Serial API (no library) |

## Layout

```
src/
  lib/protocol.ts   framing, CRC-8, incremental frame parser (pure, unit-tested)
  lib/device.ts     PdPowerDevice: Web Serial transport, request queue, typed commands
  lib/store.ts      app state, polling loop, validated actions used by the components
  components/       one small component per panel
  locales/          en / zh / ja / ru strings
```

## Protocol notes

These are implemented from `Communication protocol/*.xlsx` and `com_pdpower.py`:

- Frames are `[cmd, ...data, 0x0A]`. With CRC framing, the trailer is a CRC-8 (poly 0x31, init 0xFF) instead.
  Read requests set bit 7 (`cmd | 0x80`). Writes are not acknowledged.
- Data bytes can equal `0x0A`, so the parser checks fixed-size replies by length *and* trailer.
  It doesn't treat the first `0x0A` as the end of a frame.
- Enable flags use `1 = enabled`, which matches the Python script and the official tool. The spreadsheet says the opposite.
- USB filter: VID `0x1A86`, PID `0xFE0C`. The serial number prefix gives the model: `AC…` is V1 and `B3…` is V1.1.
- Polling: measured output every 200 ms, output state every 400 ms, input state every 1 s.
