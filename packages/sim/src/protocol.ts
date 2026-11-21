import type { DesignParams } from "@ship/craft"
import type { Recording } from "./recording"
import type { RunProgress } from "./run"

/** Messages between the browser main thread and the simulation worker. */
export type ToWorker = { readonly kind: "run"; readonly design: DesignParams }
export type FromWorker =
  | { readonly kind: "progress"; readonly progress: RunProgress }
  | { readonly kind: "done"; readonly recording: Recording }
  | { readonly kind: "error"; readonly message: string }
