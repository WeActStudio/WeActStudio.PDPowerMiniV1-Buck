import { describe, expect, it } from "vitest";
import { Cmd, FrameParser, crc8, decodeOutputState, encodeFrame, versionAtLeast } from "./protocol";

const bytes = (...b: number[]) => Uint8Array.from(b);
const ascii = (s: string) => [...new TextEncoder().encode(s)];

describe("crc8", () => {
  it("matches the CRC-8/NRSC-5 check value (poly 0x31, init 0xFF)", () => {
    expect(crc8(ascii("123456789"))).toBe(0xf7);
  });
});

describe("encodeFrame", () => {
  it("appends 0x0A in plain mode", () => {
    expect([...encodeFrame([0x84, 2], false)]).toEqual([0x84, 2, 0x0a]);
  });
  it("appends CRC in CRC mode", () => {
    expect([...encodeFrame([0x81], true)]).toEqual([0x81, crc8([0x81])]);
  });
});

describe("FrameParser (plain)", () => {
  it("parses a string reply", () => {
    const p = new FrameParser();
    expect(p.push(bytes(0x81, ...ascii("PDPowerMini"), 0x0a))).toEqual([
      { cmd: Cmd.WHO_AM_I, payload: bytes(...ascii("PDPowerMini")) },
    ]);
  });

  it("handles data bytes equal to 0x0A inside fixed frames", () => {
    // 0x0A0A mV = 2570 mV, 0x000A mA = 10 mA
    const p = new FrameParser();
    const [f] = p.push(bytes(0x85, 0x0a, 0x0a, 0x0a, 0x00, 0x0a));
    expect(f.cmd).toBe(Cmd.OUTPUT_DISPLAY);
    expect([...f.payload]).toEqual([0x0a, 0x0a, 0x0a, 0x00]);
  });

  it("reassembles frames split across chunks", () => {
    const p = new FrameParser();
    expect(p.push(bytes(0x84, 0x01, 0x10))).toEqual([]);
    expect(p.push(bytes(0x27, 0xe8, 0x03, 0x0a))).toEqual([
      { cmd: Cmd.OUTPUT_DATA, payload: bytes(0x01, 0x10, 0x27, 0xe8, 0x03) },
    ]);
  });

  it("skips noise and resyncs after a bad trailer", () => {
    const p = new FrameParser();
    const frames = p.push(bytes(0x00, 0x33, 0x82, 0x01, 0x99, 0x82, 0x03, 0x0a));
    expect(frames).toEqual([{ cmd: Cmd.OUTPUT_EN_STATE, payload: bytes(0x03) }]);
  });

  it("parses several frames in one chunk", () => {
    const p = new FrameParser();
    const frames = p.push(bytes(0xc2, ...ascii("V1.0.2.0"), 0x0a, 0x83, 0x02, 0x0a));
    expect(frames.map((f) => f.cmd)).toEqual([Cmd.SYSTEM_VERSION, Cmd.OUTPUT_ID]);
  });
});

describe("FrameParser (CRC)", () => {
  it("parses length-prefixed string replies", () => {
    const body = [0xc3, 4, ...ascii("AC12")];
    const p = new FrameParser(true);
    expect(p.push(bytes(...body, crc8(body)))).toEqual([
      { cmd: Cmd.SYSTEM_SERIAL_NUM, payload: bytes(...ascii("AC12")) },
    ]);
  });

  it("rejects frames with a bad CRC", () => {
    const body = [0x88, 50];
    const p = new FrameParser(true);
    expect(p.push(bytes(...body, crc8(body) ^ 1))).toEqual([]);
    expect(p.push(bytes(...body, crc8(body)))).toEqual([{ cmd: Cmd.BRIGHTNESS, payload: bytes(50) }]);
  });
});

describe("decodeOutputState", () => {
  it("decodes enable bit and mode bits", () => {
    expect(decodeOutputState(0b000)).toEqual({ enabled: false, mode: "OK" });
    expect(decodeOutputState(0b011)).toEqual({ enabled: true, mode: "CC" });
    expect(decodeOutputState(0b101)).toEqual({ enabled: true, mode: "OC" });
  });
});

describe("versionAtLeast", () => {
  it("compares dotted versions with V prefix and suffix", () => {
    expect(versionAtLeast("V1.0.2.0", "1.0.2.0")).toBe(true);
    expect(versionAtLeast("V1.0.10.0_beta", "1.0.2.0")).toBe(true);
    expect(versionAtLeast("V1.0.1.0", "1.0.2.0")).toBe(false);
  });
});
