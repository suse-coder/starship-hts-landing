/** @ship/gas — equilibrium high-temperature air, real-gas shocks, plasma transport. */
import type { ShockTable } from "./shockTable"
import table from "./shock-table.json"
export * from "./air"
export * from "./plasma"
export * from "./shockTable"
/** Pre-generated real-gas shock/stagnation table (scripts/gen-table.ts). */
export const SHOCK_TABLE: ShockTable = table as unknown as ShockTable
