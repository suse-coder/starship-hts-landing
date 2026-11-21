/// <reference lib="webworker" />
/**
 * Simulation worker: builds the world (real-gas tables, aero/MHD tables, Flight 15 targeting) and
 * runs the whole mission once, streaming progress; the recording is transferred (not copied).
 */
import { runMission, type ToWorker, type FromWorker } from "@ship/sim"

const post = (m: FromWorker, transfer: Transferable[] = []): void => (self as unknown as DedicatedWorkerGlobalScope).postMessage(m, transfer)

self.onmessage = (ev: MessageEvent<ToWorker>) => {
  const msg = ev.data
  if (msg.kind !== "run") return
  try {
    const { recording } = runMission(msg.design, (progress) => post({ kind: "progress", progress }))
    const transfer: Transferable[] = [...recording.data.map((a) => a.buffer), recording.heat.buffer, recording.pred.buffer, recording.predHas.buffer]
    post({ kind: "done", recording }, transfer)
  } catch (e) {
    post({ kind: "error", message: e instanceof Error ? `${e.message}\n${e.stack ?? ""}` : String(e) })
  }
}
