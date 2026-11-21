/**
 * Recording cache (IndexedDB). Key = simulation version (hash of the physics sources, vite.config.ts)
 * + hash of the design parameters. Only the latest entry is kept. Typed arrays are stored by
 * structured clone (≈ 20 MB per recording), so a reload after a view/app edit skips the simulation.
 */
import type { Recording } from "@ship/sim"

const DB = "starship-sim"
const STORE = "recordings"

const open = (): Promise<IDBDatabase> =>
  new Promise((res, rej) => {
    const r = indexedDB.open(DB, 1)
    r.onupgradeneeded = () => r.result.createObjectStore(STORE)
    r.onsuccess = () => res(r.result)
    r.onerror = () => rej(r.error)
  })

const tx = async <T>(mode: IDBTransactionMode, fn: (s: IDBObjectStore) => IDBRequest<T> | void): Promise<T | undefined> => {
  const db = await open()
  return new Promise((res, rej) => {
    const t = db.transaction(STORE, mode)
    const req = fn(t.objectStore(STORE))
    t.oncomplete = () => { db.close(); res(req ? req.result : undefined) }
    t.onerror = () => { db.close(); rej(t.error) }
  })
}

/** FNV-1a 32-bit of a string (design parameters → short key part) */
export const fnv1a = (s: string): string => {
  let h = 0x811c9dc5
  for (let i = 0; i < s.length; i++) { h ^= s.charCodeAt(i); h = Math.imul(h, 0x01000193) >>> 0 }
  return h.toString(16).padStart(8, "0")
}

export const loadCached = async (key: string): Promise<Recording | null> => {
  try {
    const rec = await tx<Recording>("readonly", (s) => s.get(key) as IDBRequest<Recording>)
    return rec ?? null
  } catch { return null }
}

export const storeCached = async (key: string, rec: Recording): Promise<void> => {
  try {
    await tx("readwrite", (s) => { s.clear(); s.put(rec, key) })
  } catch (e) { console.warn("recording cache: store failed", e) }
}

export const clearCached = async (): Promise<void> => {
  try { await tx("readwrite", (s) => { s.clear() }) } catch { /* ignore */ }
}
