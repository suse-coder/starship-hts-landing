/**
 * Pre-computed mission recording loader.
 *
 * `scripts/snapshot.ts` writes `public/recording.bin` (the same columnar recording the worker
 * produces). A static GitHub Pages visitor should play that file directly instead of re-running the
 * physics in a Web Worker: the boot tries this first and only falls back to the worker when the
 * snapshot is missing (dev without `bun run snapshot`, or a forced `?resim`).
 *
 * Binary layout (little-endian): u32 header length, header JSON padded to an 8-byte boundary
 * ({ format, n, cells, predK, ch, events, meta }), Float64 data n×ch, Float32 heat n×cells,
 * Float32 pred n×predK×3, Uint8 predHas n.
 */
import type { Recording } from "@ship/sim"
import { SIM_VERSION } from "virtual:sim-version"

interface SnapshotHeader {
  readonly format: number
  readonly version?: string
  readonly n: number
  readonly cells: number
  readonly predK: number
  readonly ch: number
  /** the wind the recording was flown in (absent in recordings made before it was recorded) */
  readonly wind?: { readonly scale: number; readonly turbulence: number; readonly seed: number }
  readonly events: Recording["events"]
  readonly meta: Recording["meta"]
}

export const loadSnapshot = async (want: { readonly windScale: number; readonly turbulence: number; readonly seed: number }): Promise<Recording | null> => {
  try {
    const base = import.meta.env.BASE_URL || "/"
    const res = await fetch(`${base}recording.bin`, { cache: "force-cache" })
    if (!res.ok) return null
    const buf = await res.arrayBuffer()
    if (buf.byteLength < 12) return null
    const headLen = new DataView(buf).getUint32(0, true)
    if (headLen < 2 || 4 + headLen > buf.byteLength) return null
    const text = new TextDecoder().decode(new Uint8Array(buf, 4, headLen))
    const header = JSON.parse(text.slice(0, text.lastIndexOf("}") + 1)) as SnapshotHeader
    // Only a recording made from exactly this code may be played. In dev any physics edit changes
    // SIM_VERSION, so the committed snapshot is ignored and the app re-simulates; on Pages a stale
    // committed snapshot is likewise ignored rather than shown as if it were this build.
    if (header.format !== 1 || header.version !== SIM_VERSION) {
      console.info(`precomputed recording is for ${header.version ?? "an unknown version"}, code is ${SIM_VERSION} — re-simulating`)
      return null
    }
    // … and only for the wind it was flown in (a no-wind recording is not a windy mission)
    const w = header.wind ?? { scale: 0, turbulence: 0, seed: want.seed }
    if (w.scale !== want.windScale || w.turbulence !== want.turbulence || w.seed !== want.seed) {
      console.info(`precomputed recording was flown with wind ${w.scale}/${w.turbulence}, this run asks for ${want.windScale}/${want.turbulence} — re-simulating`)
      return null
    }
    if (!(header.n > 0) || !(header.ch > 0)) return null
    const { n, cells, predK, ch } = header
    let off = 4 + headLen
    const data: Float64Array[] = []
    for (let c = 0; c < ch; c++) data.push(new Float64Array(buf, off + c * n * 8, n))
    off += ch * n * 8
    const heat = new Float32Array(buf, off, n * cells)
    off += n * cells * 4
    const pred = new Float32Array(buf, off, n * predK * 3)
    off += n * predK * 3 * 4
    const predHas = new Uint8Array(buf, off, n)
    return { n, data, heat, pred, predK, predHas, cells, events: header.events, meta: header.meta }
  } catch (e) {
    console.warn("precomputed recording unavailable; falling back to the simulation worker", e)
    return null
  }
}
