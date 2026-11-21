import { PHASES } from "@ship/gnc"
export { runMission, CH } from "@ship/sim"
export const PHASES_OF = (i: number): string => PHASES[Math.round(i)]?.id ?? "?"
