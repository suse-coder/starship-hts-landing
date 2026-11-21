/** Generates src/physics/shock-table.json (real-gas shock/stagnation table). Run: bun scripts/gen-tables.ts */
import { buildShockTable, DEFAULT_GRID, SHOCK_FIELDS } from "../src/shockTable"
const t0 = performance.now()
const t = buildShockTable(DEFAULT_GRID.hs, DEFAULT_GRID.us)
const round = (x: number) => Number(x.toPrecision(6))
const out = { hs: t.hs, us: t.us, fields: Object.fromEntries(SHOCK_FIELDS.map((f) => [f, t.fields[f].map(round)])) }
await Bun.write(new URL("../src/shock-table.json", import.meta.url), JSON.stringify(out))
console.log(`shock table ${t.hs.length}x${t.us.length} in ${((performance.now() - t0) / 1000).toFixed(1)} s`)
