# bench/00 — Machine and run budget (development environment)

Measured 2026-11-21 on the devcontainer that runs the OpenCode server, the Vite dev server, the tests and
the headless browser checks. This is the development machine, not the user's device. The browser app's own
budget is in spec/04 (A-series); the per-phase run time (bench/01) is planned under T07.

## 1. Measurements

| Resource | Value | Source |
|---|---|---|
| CPU | 12 vCPU; load average 12.0 (1 min), 12.9 (5 min), 14.7 (15 min) | `nproc`, `/proc/loadavg` |
| CPU pressure | some avg60 20.7 %, avg300 20.1 % (tasks waiting for a core) | `/proc/pressure/cpu` |
| Memory | 40 GB total, 8.9 GB used, 31 GB available; swap unused | `free -m` |
| Memory pressure | some and full 0.0; cgroup `oom_kill` 0 | `/proc/pressure/memory`, `memory.events` |
| Server restarts | 20 "Event stream stalled" and 2 "Timed out connecting to server" disconnects today; about 12 server restarts since 06:00 | `~/.local/share/opencode/log/opencode.log` |
| MCP fan-out | `rea mcp` ×5 (one per project directory), about 275 MB RSS each, 25–40 s to connect; `browse mcp` ×1 | `ps`, the log |
| Storage | `opencode.db` 828 MB (+ 17 MB WAL), worktree 423 MB, log 51 MB, snapshot 50 MB | `du` |

## 2. Diagnosis

- **Memory is not the cause.** No OOM kill, no memory pressure, 31 GB free.
- **CPU starvation is the cause of the stalls.** The client declares "Event stream stalled" when the server
  stops answering its event stream, then restarts `opencode serve --service`. The restart adds load (five MCP
  servers start at once and answer 503 meanwhile), which can trigger the next stall.
- Stalls cluster with the heaviest work: software-rendered Chromium (WebGL via SwiftShader uses all cores),
  mission runs (one core each for 1–2 min), and parallel dev-server, test and screenshot jobs.
- Not measured: whether synchronous reads of the 828 MB session database block the event loop. This is
  plausible. The test is to archive old sessions and count the stalls again.

## 3. Run budget (mandatory for every agent and every check)

1. Heavy commands run under `nice -n 19`: the mission, the profile script, headless-browser runs, and whole-repo
   `bun test` runs longer than a minute.
2. One mission or profile run at a time. Check first with `pgrep -fa 'scripts/(mission|profile)'`.
3. One headless browser at a time, viewport 1280×720 unless a test needs more, closed in a `finally` block. A
   `timeout` on the node process does not stop its Chromium children: check with `pgrep -fa chrome-linux64`.
4. At most two subagents run heavy jobs at the same time.
5. Never `pkill -f vite` (it kills its own shell). Stop a process by its PID.

## 4. Measured run times (Bun on this machine, shared CPU)

| Run | Time | Target |
|---|---|---|
| `bun test packages` | 2–11 s | – |
| Full mission in Bun (`scripts/bench.ts`, `nice -n 19`) | **35.4 s** (35.3 s sim + 78 ms planning; 72,148 steps, 491 µs/step) | G4: ≤ 30 s — **not met** (18 % over), see bench/01 |
| Full mission in the browser worker | 26–125 s; the lower end is an idle machine. T07 did not measure the worker (no exported recording). | – |

## 5. Recommendations (need the user's decision)

- Restrict the `rea` MCP server to the one project that uses it (five instances → one). This saves about 1.1 GB
  of RSS and the restart burst.
- Archive old sessions in `opencode.db`, then count the stalls again.
- The VS Code remote helpers show CPU bursts above 100 %. Close idle terminals and extensions that scan ports.
