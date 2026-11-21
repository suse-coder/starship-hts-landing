import { createHash } from "node:crypto"
import { readdirSync, readFileSync, statSync } from "node:fs"
import { join, resolve } from "node:path"
import { defineConfig, type Plugin } from "vite"

/**
 * Simulation version = SHA-256 over the source of every package that influences the recording
 * (physics, guidance, simulation loop), with comments and whitespace stripped so that doc-only edits
 * do not count. The browser caches the recording under this key (IndexedDB, app/src/simCache.ts):
 * view/app edits reload instantly from the cache; a physics change produces a new key → one new run.
 */
const PHYSICS_PACKAGES = ["math", "earth", "gas", "craft", "aero", "heat", "mhd", "tower", "gnc", "sim"]
const ROOT = resolve(__dirname, "..")
const VIRTUAL = "virtual:sim-version"
const RESOLVED = "\0" + VIRTUAL

const listTs = (dir: string): string[] =>
  readdirSync(dir).flatMap((name) => {
    const p = join(dir, name)
    return statSync(p).isDirectory() ? listTs(p) : p.endsWith(".ts") ? [p] : []
  })

/** strip block/line comments and collapse whitespace (approximate, but deterministic) */
const normalise = (src: string): string =>
  src.replace(/\/\*[\s\S]*?\*\//g, "").replace(/(^|[^:"'`])\/\/.*$/gm, "$1").replace(/\s+/g, " ").trim()

const physicsDirs = PHYSICS_PACKAGES.map((p) => join(ROOT, p, "src"))

const computeVersion = (): string => {
  const h = createHash("sha256")
  for (const file of physicsDirs.flatMap(listTs).sort()) {
    h.update(file.slice(ROOT.length))
    h.update(normalise(readFileSync(file, "utf8")))
  }
  return h.digest("hex").slice(0, 16)
}

const simVersion = (): Plugin => {
  let version = computeVersion()
  return {
    name: "sim-version",
    resolveId: (id) => (id === VIRTUAL ? RESOLVED : null),
    load: (id) => (id === RESOLVED ? `export const SIM_VERSION = ${JSON.stringify(version)}\n` : null),
    handleHotUpdate({ file, server }) {
      if (!physicsDirs.some((d) => file.startsWith(d))) return
      const next = computeVersion()
      if (next === version) return // comment/whitespace-only edit: the cached recording stays valid
      version = next
      const mod = server.moduleGraph.getModuleById(RESOLVED)
      if (mod) server.moduleGraph.invalidateModule(mod)
      server.config.logger.info(`[sim-version] physics changed → ${version} (next load re-simulates)`, { timestamp: true })
    },
  }
}

export default defineConfig({
  base: "./",
  plugins: [simVersion()],
  build: { target: "es2022", outDir: "dist", chunkSizeWarningLimit: 2000 },
  worker: { format: "es" },
})
