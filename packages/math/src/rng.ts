/**
 * Counter-based (stateless) random numbers: the value is a pure function of (seed, counter),
 * so the simulation stays referentially transparent and replayable.
 */
const hash32 = (x0: number): number => {
  let x = x0 | 0
  x = Math.imul(x ^ (x >>> 16), 0x7feb352d)
  x = Math.imul(x ^ (x >>> 15), 0x846ca68b)
  return (x ^ (x >>> 16)) >>> 0
}

/** uniform in (0,1) */
export const uniform = (seed: number, k: number, stream = 0): number =>
  (hash32(hash32(seed * 0x9e3779b1 + stream) + k) + 0.5) / 4294967296

/** standard normal via Box-Muller from two uniforms of the same counter */
export const gauss = (seed: number, k: number, stream = 0): number => {
  const u1 = uniform(seed, 2 * k, stream)
  const u2 = uniform(seed, 2 * k + 1, stream)
  return Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u2)
}
