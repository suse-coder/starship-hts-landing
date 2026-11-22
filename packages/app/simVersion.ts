/**
 * Simulation version: SHA-256 over the source of every package that influences the recording
 * (physics, guidance, simulation loop), comments and whitespace stripped so doc-only edits do not
 * count. Used by the Vite plugin (`virtual:sim-version`), the snapshot writer and the app boot:
 * a precomputed recording is played only when its version equals the current code — otherwise the
 * app re-simulates (in dev after any physics edit, and on Pages after a stale committed snapshot).
 */
import { createHash } from "node:crypto"
import { readdirSync, readFileSync, statSync } from "node:fs"
import { join } from "node:path"

const PHYSICS_PACKAGES = ["math", "earth", "gas", "craft", "aero", "heat", "mhd", "tower", "gnc", "sim"]

const listTs = (dir: string): string[] =>
  readdirSync(dir).flatMap((name) => {
    const p = join(dir, name)
    return statSync(p).isDirectory() ? listTs(p) : p.endsWith(".ts") ? [p] : []
  })

/** strip block/line comments and collapse whitespace (approximate, but deterministic) */
const normalise = (src: string): string =>
  src.replace(/\/\*[\s\S]*?\*\//g, "").replace(/(^|[^:"'`])\/\/.*$/gm, "$1").replace(/\s+/g, " ").trim()

/** `packagesDir` is the repository's `packages/` directory */
export const physicsVersion = (packagesDir: string): string => {
  const h = createHash("sha256")
  const dirs = PHYSICS_PACKAGES.map((p) => join(packagesDir, p, "src"))
  for (const file of dirs.flatMap(listTs).sort()) {
    h.update(file.slice(packagesDir.length))
    h.update(normalise(readFileSync(file, "utf8")))
  }
  return h.digest("hex").slice(0, 16)
}
