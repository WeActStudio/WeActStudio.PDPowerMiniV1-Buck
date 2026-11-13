/**
 * WeAct PD Power Mini V1.x serial protocol.
 *
 * Source of truth: `Communication protocol/*.xlsx` and `com_pdpower.py` in this repo.
 *
 * Every frame is `[command, ...data, trailer]`. The trailer is `0x0A` in normal (USB) mode,
 * or a CRC-8 of the preceding bytes when CRC framing is enabled (UART mode option).
 * Reads set bit 7 of the command byte (`cmd | 0x80`); the device answers with the same header.
 * Writes are not acknowledged by the device.
 */

export const Cmd = {
  WHO_AM_I: 0x01,
  OUTPUT_EN_STATE: 0x02,
  OUTPUT_ID: 0x03,
  OUTPUT_DATA: 0x04,
  OUTPUT_DISPLAY: 0x05,
  OUTPUT_OCP_EN: 0x06,
  OUTPUT_OFFSET_EN: 0x07,
  BRIGHTNESS: 0x08,
  OUTPUT_DISCHARGE_EN: 0x09,
  INPUT_STATE: 0x0a,
  SYSTEM_RESET: 0x40,
  SYSTEM_VERSION: 0x42,
  SYSTEM_SERIAL_NUM: 0x43,
  SYSTEM_CONFIG_SAVE: 0x44,
  SYSTEM_FACTORY_RESET: 0x45,
  SYSTEM_LCD_PANEL_TYPE: 0x46,
  SYSTEM_FACTORY_DATA: 0x47,
} as const;

export const READ_FLAG = 0x80;
export const END = 0x0a;

/** Replies whose payload is a variable-length ASCII string. */
const STRING_REPLIES: ReadonlySet<number> = new Set([
  Cmd.WHO_AM_I,
  Cmd.SYSTEM_VERSION,
  Cmd.SYSTEM_SERIAL_NUM,
]);

/** Total length (header + data + trailer) of each fixed-size read reply. */
const FIXED_REPLY_LENGTH: Readonly<Record<number, number>> = {
  [Cmd.OUTPUT_EN_STATE]: 3,
  [Cmd.OUTPUT_ID]: 3,
  [Cmd.OUTPUT_DATA]: 7,
  [Cmd.OUTPUT_DISPLAY]: 6,
  [Cmd.OUTPUT_OCP_EN]: 3,
  [Cmd.OUTPUT_OFFSET_EN]: 3,
  [Cmd.BRIGHTNESS]: 3,
  [Cmd.INPUT_STATE]: 7,
  [Cmd.SYSTEM_LCD_PANEL_TYPE]: 3,
  [Cmd.SYSTEM_FACTORY_DATA]: 66,
};

/** Longest string reply we will wait for before deciding a header byte was noise. */
const MAX_STRING_REPLY = 128;

/** CRC-8, poly 0x31 (x^8 + x^5 + x^4 + 1), init 0xFF, no reflection, no final XOR. */
export function crc8(data: ArrayLike<number>): number {
  let crc = 0xff;
  for (let i = 0; i < data.length; i++) {
    crc ^= data[i];
    for (let b = 0; b < 8; b++) {
      crc = crc & 0x80 ? ((crc << 1) ^ 0x31) & 0xff : (crc << 1) & 0xff;
    }
  }
  return crc;
}

export function encodeFrame(body: readonly number[], useCrc: boolean): Uint8Array {
  const bytes = body.map((b) => b & 0xff);
  bytes.push(useCrc ? crc8(bytes) : END);
  return Uint8Array.from(bytes);
}

export const u16le = (n: number): [number, number] => [n & 0xff, (n >> 8) & 0xff];
export const readU16le = (p: Uint8Array, offset: number): number =>
  p[offset] | (p[offset + 1] << 8);

export interface Frame {
  /** Command id without the read flag. */
  cmd: number;
  /** Data bytes between header (and length byte, for CRC string replies) and trailer. */
  payload: Uint8Array;
}

/**
 * Incremental parser for device replies. Feed it raw chunks from the serial port; it returns
 * every complete, validated frame and keeps any partial tail for the next chunk.
 *
 * Fixed-size replies are validated by length *and* trailer, because data bytes may legitimately
 * be 0x0A (e.g. a voltage low byte). On a mismatch the parser drops one byte and resyncs.
 */
export class FrameParser {
  private buf = new Uint8Array(0);

  constructor(public useCrc = false) {}

  reset(): void {
    this.buf = new Uint8Array(0);
  }

  push(chunk: Uint8Array): Frame[] {
    const merged = new Uint8Array(this.buf.length + chunk.length);
    merged.set(this.buf, 0);
    merged.set(chunk, this.buf.length);
    const buf = merged;

    const frames: Frame[] = [];
    let i = 0;
    while (i < buf.length) {
      const head = buf[i];
      const cmd = head & 0x7f;
      const isString = STRING_REPLIES.has(cmd);
      if (!(head & READ_FLAG) || (!isString && FIXED_REPLY_LENGTH[cmd] === undefined)) {
        i++;
        continue;
      }

      let len: number;
      if (!isString) {
        len = FIXED_REPLY_LENGTH[cmd];
      } else if (this.useCrc) {
        // [header, length, ...chars, crc]
        if (i + 1 >= buf.length) break;
        len = buf[i + 1] + 3;
      } else {
        // [header, ...chars, 0x0A]
        const end = buf.indexOf(END, i + 1);
        if (end < 0) {
          if (buf.length - i > MAX_STRING_REPLY) {
            i++;
            continue;
          }
          break;
        }
        len = end - i + 1;
      }

      if (i + len > buf.length) break; // wait for the rest of the frame

      const frame = buf.subarray(i, i + len);
      const trailer = frame[len - 1];
      const valid = this.useCrc ? crc8(frame.subarray(0, len - 1)) === trailer : trailer === END;
      if (!valid) {
        i++;
        continue;
      }

      const start = isString && this.useCrc ? 2 : 1;
      frames.push({ cmd, payload: frame.slice(start, len - 1) });
      i += len;
    }

    this.buf = buf.slice(i);
    return frames;
  }
}

export type OutputMode = "OK" | "CC" | "OC";

/** OUTPUT_EN_STATE reply: bit0 = enabled, bits 2..1 = 00 normal / 01 CC / 10 OC. */
export function decodeOutputState(x: number): { enabled: boolean; mode: OutputMode } {
  const m = (x >> 1) & 0x03;
  return { enabled: (x & 0x01) === 1, mode: m === 1 ? "CC" : m === 2 ? "OC" : "OK" };
}

export const InputState = {
  WAIT: 0,
  WAIT_PD_OK: 1,
  WAIT_QC_OK: 2,
  ERR: 3,
  QC: 4,
  PD: 5,
  DC: 6,
} as const;

export const INPUT_STATE_KEYS = ["WAIT", "WAIT_PD_OK", "WAIT_QC_OK", "ERR", "QC", "PD", "DC"] as const;

/** Compare firmware version strings like "V1.0.2.0" or "V1.0.2.0_xyz". Returns a >= b. */
export function versionAtLeast(version: string, target: string): boolean {
  const parse = (s: string) =>
    s.trim().replace(/^V/i, "").split("_")[0].split(".").map((n) => Number.parseInt(n, 10) || 0);
  const a = parse(version);
  const b = parse(target);
  for (let i = 0; i < Math.max(a.length, b.length); i++) {
    const x = a[i] ?? 0;
    const y = b[i] ?? 0;
    if (x !== y) return x > y;
  }
  return true;
}
