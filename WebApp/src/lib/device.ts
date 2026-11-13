import {
  Cmd,
  FrameParser,
  READ_FLAG,
  decodeOutputState,
  encodeFrame,
  readU16le,
  u16le,
  type OutputMode,
} from "./protocol";

/** USB IDs reported by the device (WCH VID 0x1A86, PID 0xFE0C). */
export const USB_FILTER: SerialPortFilter = { usbVendorId: 0x1a86, usbProductId: 0xfe0c };

export const UART_BAUD_RATES = [9600, 19200, 38400, 57600, 115200, 230400, 460800] as const;
export const DEFAULT_BAUD_RATE = 115200;

export interface ConnectOptions {
  baudRate: number;
  crc: boolean;
}

export class TimeoutError extends Error {
  constructor(cmd: number) {
    super(`No reply to command 0x${(cmd | READ_FLAG).toString(16)}`);
    this.name = "TimeoutError";
  }
}

interface Pending {
  cmd: number;
  resolve: (payload: Uint8Array) => void;
  reject: (err: unknown) => void;
}

const REPLY_TIMEOUT_MS = 500;
/** Gap after an un-acknowledged write so the firmware can process it before the next frame. */
const WRITE_GAP_MS = 20;

const sleep = (ms: number) => new Promise<void>((r) => setTimeout(r, ms));
const decodeText = (p: Uint8Array) => new TextDecoder().decode(p).replace(/\0+$/, "").trim();

/**
 * One PD Power Mini on a Web Serial port.
 *
 * All traffic goes through a single FIFO queue, so exactly one request is in flight at a time
 * and replies are matched to the request that caused them.
 */
export class PdPowerDevice {
  private port: SerialPort | null = null;
  private reader: ReadableStreamDefaultReader<Uint8Array> | null = null;
  private readLoopDone: Promise<void> = Promise.resolve();
  private parser = new FrameParser();
  private useCrc = false;
  private queue: Promise<unknown> = Promise.resolve();
  private pending: Pending | null = null;
  private closing = false;

  /** Called once when the port goes away without `close()` (unplugged, fatal error). */
  onDisconnect: ((error?: unknown) => void) | null = null;

  get isOpen(): boolean {
    return this.port !== null;
  }

  async open(port: SerialPort, opts: ConnectOptions): Promise<void> {
    if (this.port) throw new Error("Already connected");
    await port.open({ baudRate: opts.baudRate, bufferSize: 4096 });
    this.port = port;
    this.closing = false;
    this.useCrc = opts.crc;
    this.parser = new FrameParser(opts.crc);
    this.queue = Promise.resolve();
    this.readLoopDone = this.readLoop(port);
  }

  async close(): Promise<void> {
    const port = this.port;
    if (!port) return;
    this.closing = true;
    this.pending?.reject(new Error("Port closed"));
    await this.reader?.cancel().catch(() => {});
    await this.readLoopDone;
    await port.close().catch(() => {});
    this.port = null;
  }

  private async readLoop(port: SerialPort): Promise<void> {
    let lastError: unknown;
    // Non-fatal errors (framing, overrun) replace `port.readable`; fatal ones null it.
    while (port.readable && !this.closing) {
      const reader = port.readable.getReader();
      this.reader = reader;
      try {
        for (;;) {
          const { value, done } = await reader.read();
          if (done) break;
          if (value) this.onBytes(value);
        }
      } catch (err) {
        lastError = err;
      } finally {
        reader.releaseLock();
        this.reader = null;
      }
    }
    if (!this.closing) {
      this.pending?.reject(lastError ?? new Error("Port disconnected"));
      await port.close().catch(() => {});
      this.port = null;
      this.onDisconnect?.(lastError);
    }
  }

  private onBytes(chunk: Uint8Array): void {
    for (const frame of this.parser.push(chunk)) {
      if (this.pending && frame.cmd === this.pending.cmd) this.pending.resolve(frame.payload);
    }
  }

  private async writeRaw(bytes: Uint8Array): Promise<void> {
    const writable = this.port?.writable;
    if (!writable) throw new Error("Port not open");
    const writer = writable.getWriter();
    try {
      await writer.write(bytes);
    } finally {
      writer.releaseLock();
    }
  }

  private enqueue<T>(job: () => Promise<T>): Promise<T> {
    const run = this.queue.then(job);
    this.queue = run.catch(() => {});
    return run;
  }

  /** Send a read request and resolve with the reply payload. */
  request(cmd: number, args: number[] = [], timeoutMs = REPLY_TIMEOUT_MS): Promise<Uint8Array> {
    return this.enqueue(async () => {
      this.parser.reset();
      const reply = new Promise<Uint8Array>((resolve, reject) => {
        const timer = setTimeout(() => {
          this.pending = null;
          reject(new TimeoutError(cmd));
        }, timeoutMs);
        const settle = () => {
          clearTimeout(timer);
          this.pending = null;
        };
        this.pending = {
          cmd,
          resolve: (p) => (settle(), resolve(p)),
          reject: (e) => (settle(), reject(e)),
        };
      });
      try {
        await this.writeRaw(encodeFrame([cmd | READ_FLAG, ...args], this.useCrc));
      } catch (err) {
        this.pending?.reject(err);
      }
      return reply;
    });
  }

  /** Send a write command (no reply expected). */
  command(cmd: number, args: number[] = []): Promise<void> {
    return this.enqueue(async () => {
      await this.writeRaw(encodeFrame([cmd, ...args], this.useCrc));
      await sleep(WRITE_GAP_MS);
    });
  }

  // ---- Device info -------------------------------------------------------------------------

  async whoAmI(): Promise<string> {
    return decodeText(await this.request(Cmd.WHO_AM_I));
  }

  async firmwareVersion(): Promise<string> {
    return decodeText(await this.request(Cmd.SYSTEM_VERSION));
  }

  async serialNumber(): Promise<string> {
    return decodeText(await this.request(Cmd.SYSTEM_SERIAL_NUM)).toUpperCase();
  }

  // ---- Output ------------------------------------------------------------------------------

  async outputState(): Promise<{ enabled: boolean; mode: OutputMode }> {
    const [x] = await this.request(Cmd.OUTPUT_EN_STATE);
    return decodeOutputState(x);
  }

  setOutputEnabled(on: boolean): Promise<void> {
    return this.command(Cmd.OUTPUT_EN_STATE, [on ? 1 : 0]);
  }

  async activePreset(): Promise<number> {
    const [id] = await this.request(Cmd.OUTPUT_ID);
    return id;
  }

  setActivePreset(id: number): Promise<void> {
    return this.command(Cmd.OUTPUT_ID, [id]);
  }

  /** Preset (M0..M4) set-points in millivolts / milliamps. */
  async preset(id: number): Promise<{ id: number; mV: number; mA: number }> {
    const p = await this.request(Cmd.OUTPUT_DATA, [id]);
    return { id: p[0], mV: readU16le(p, 1), mA: readU16le(p, 3) };
  }

  setPreset(id: number, mV: number, mA: number): Promise<void> {
    return this.command(Cmd.OUTPUT_DATA, [id, ...u16le(mV), ...u16le(mA)]);
  }

  /** Measured output in millivolts / milliamps. */
  async measuredOutput(): Promise<{ mV: number; mA: number }> {
    const p = await this.request(Cmd.OUTPUT_DISPLAY);
    return { mV: readU16le(p, 0), mA: readU16le(p, 2) };
  }

  // ---- Input (firmware >= 1.0.2.0 on V1, >= 1.0.1.0 on V1.1) ------------------------------

  /** Input state, input voltage (mV) and requested PD voltage (volts). */
  async inputState(): Promise<{ state: number; mV: number; pdVolts: number }> {
    const p = await this.request(Cmd.INPUT_STATE);
    return { state: p[0], mV: readU16le(p, 1), pdVolts: readU16le(p, 3) / 10 };
  }

  /** Only takes effect while the output is off and below 5 V. */
  setPdVoltage(volts: number): Promise<void> {
    return this.command(Cmd.INPUT_STATE, u16le(Math.round(volts * 10)));
  }

  // ---- Settings ----------------------------------------------------------------------------

  async ocpEnabled(): Promise<boolean> {
    return (await this.request(Cmd.OUTPUT_OCP_EN))[0] === 1;
  }

  setOcpEnabled(on: boolean): Promise<void> {
    return this.command(Cmd.OUTPUT_OCP_EN, [on ? 1 : 0]);
  }

  async calibrationEnabled(): Promise<boolean> {
    return (await this.request(Cmd.OUTPUT_OFFSET_EN))[0] === 1;
  }

  setCalibrationEnabled(on: boolean): Promise<void> {
    return this.command(Cmd.OUTPUT_OFFSET_EN, [on ? 1 : 0]);
  }

  /** Over-current discharge. Write-only in the protocol. */
  setDischargeEnabled(on: boolean): Promise<void> {
    return this.command(Cmd.OUTPUT_DISCHARGE_EN, [on ? 1 : 0]);
  }

  async brightness(): Promise<number> {
    return (await this.request(Cmd.BRIGHTNESS))[0];
  }

  setBrightness(percent: number): Promise<void> {
    return this.command(Cmd.BRIGHTNESS, [Math.min(100, Math.max(1, Math.round(percent)))]);
  }

  // ---- System ------------------------------------------------------------------------------

  /** Persist preset, OCP, calibration, brightness and PD voltage settings to flash. */
  saveConfig(): Promise<void> {
    return this.command(Cmd.SYSTEM_CONFIG_SAVE);
  }

  factoryReset(): Promise<void> {
    return this.command(Cmd.SYSTEM_FACTORY_RESET);
  }

  reboot(): Promise<void> {
    return this.command(Cmd.SYSTEM_RESET);
  }
}
