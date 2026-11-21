/** Print the high-α bank planner internals at every update. */
import { runMission } from "@ship/sim"
import { DEFAULT_DESIGN } from "@ship/craft"
;(globalThis as { PLAN_DEBUG?: (s: string) => void }).PLAN_DEBUG = (s) => console.log(s)
runMission(DEFAULT_DESIGN)
