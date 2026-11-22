declare module "virtual:sim-version" {
  /** hash of the physics/guidance/simulation sources (see app/vite.config.ts) */
  export const SIM_VERSION: string
}

interface ImportMetaEnv {
  /** Vite base path (the GitHub Pages repository sub-path) */
  readonly BASE_URL: string
}
interface ImportMeta {
  readonly env: ImportMetaEnv
}
