import { resolve } from "node:path"
import { defineConfig, type Plugin } from "vite"
import { physicsVersion } from "./simVersion"

/**
 * Simulation version = SHA-256 over the source of every package that influences the recording
 * (physics, guidance, simulation loop), with comments and whitespace stripped so that doc-only edits
 * do not count. The browser caches the recording under this key (IndexedDB, app/src/simCache.ts):
 * view/app edits reload instantly from the cache; a physics change produces a new key → one new run.
 */
const ROOT = resolve(__dirname, "..")
const VIRTUAL = "virtual:sim-version"
const RESOLVED = "\0" + VIRTUAL
const physicsDirs = ["math", "earth", "gas", "craft", "aero", "heat", "mhd", "tower", "gnc", "sim"].map((p) => resolve(ROOT, p, "src"))
const computeVersion = (): string => physicsVersion(ROOT)

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
