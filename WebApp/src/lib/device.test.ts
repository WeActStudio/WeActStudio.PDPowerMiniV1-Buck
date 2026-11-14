import { describe, expect, it } from "vitest";
import { PdPowerDevice, TimeoutError } from "./device";
import { Cmd, END, READ_FLAG } from "./protocol";

/** Minimal SerialPort stand-in that answers like the firmware. */
function fakePort(respond: (frame: Uint8Array) => number[] | null) {
  let push!: (b: Uint8Array) => void;
  const written: number[][] = [];
  const readable = new ReadableStream<Uint8Array>({
    start(c) {
      push = (b) => c.enqueue(b);
    },
  });
  const writable = new WritableStream<Uint8Array>({
    write(frame) {
      written.push([...frame]);
      const reply = respond(frame);
      // Deliver the reply in two chunks to exercise reassembly.
      if (reply) {
        const mid = Math.ceil(reply.length / 2);
        setTimeout(() => push(Uint8Array.from(reply.slice(0, mid))), 1);
        setTimeout(() => push(Uint8Array.from(reply.slice(mid))), 2);
      }
    },
  });
  const port = {
    readable,
    writable,
    open: async () => {},
    close: async () => {},
  } as unknown as SerialPort;
  return { port, written };
}

const ascii = (s: string) => [...new TextEncoder().encode(s)];

function firmware(frame: Uint8Array): number[] | null {
  const [head, arg] = frame;
  switch (head) {
    case Cmd.WHO_AM_I | READ_FLAG:
      return [head, ...ascii("PDPowerMiniV1"), END];
    case Cmd.SYSTEM_SERIAL_NUM | READ_FLAG:
      return [head, ...ascii("ac0123"), END];
    case Cmd.OUTPUT_DATA | READ_FLAG:
      return [head, arg, 0x88, 0x13, 0xe8, 0x03, END]; // 5000 mV, 1000 mA
    case Cmd.OUTPUT_DISPLAY | READ_FLAG:
      return [head, 0x0a, 0x0a, 0x0a, 0x00, END]; // 2570 mV, 10 mA
    default:
      return null; // writes and unsupported reads: silence
  }
}

describe("PdPowerDevice", () => {
  it("matches replies to queued requests", async () => {
    const { port } = fakePort(firmware);
    const dev = new PdPowerDevice();
    await dev.open(port, { baudRate: 115200, crc: false });

    const [name, serial, preset, measured] = await Promise.all([
      dev.whoAmI(),
      dev.serialNumber(),
      dev.preset(3),
      dev.measuredOutput(),
    ]);
    expect(name).toBe("PDPowerMiniV1");
    expect(serial).toBe("AC0123");
    expect(preset).toEqual({ id: 3, mV: 5000, mA: 1000 });
    expect(measured).toEqual({ mV: 2570, mA: 10 });
    await dev.close();
  });

  it("encodes writes per the protocol sheet", async () => {
    const { port, written } = fakePort(firmware);
    const dev = new PdPowerDevice();
    await dev.open(port, { baudRate: 115200, crc: false });
    await dev.setPreset(1, 12000, 1500);
    await dev.setPdVoltage(9);
    await dev.setOutputEnabled(true);
    expect(written).toEqual([
      [0x04, 1, 0xe0, 0x2e, 0xdc, 0x05, END],
      [0x0a, 90, 0, END],
      [0x02, 1, END],
    ]);
    await dev.close();
  });

  it("times out on unanswered reads without blocking the queue", async () => {
    const { port } = fakePort(firmware);
    const dev = new PdPowerDevice();
    await dev.open(port, { baudRate: 115200, crc: false });
    await expect(dev.brightness()).rejects.toBeInstanceOf(TimeoutError);
    expect(await dev.whoAmI()).toBe("PDPowerMiniV1");
    await dev.close();
  });
});
