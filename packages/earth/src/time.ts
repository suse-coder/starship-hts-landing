/** Time scales. UTC is used as UT1 (|UT1-UTC| < 0.9 s is irrelevant here). */
export const julianDate = (isoUtc: string): number => Date.parse(isoUtc) / 86400000 + 2440587.5

/** Greenwich mean sidereal time, rad (IAU 1982 expression). */
export const gmst = (jd: number): number => {
  const d = jd - 2451545.0
  const T = d / 36525
  const deg = 280.46061837 + 360.98564736629 * d + 0.000387933 * T * T - (T * T * T) / 38710000
  const r = ((deg % 360) + 360) % 360
  return (r * Math.PI) / 180
}

/** Flight 15 timeline anchors (Wikipedia "Starship flight 15", read 2026-11-20). */
export const FLIGHT15 = {
  liftoffUtc: "2026-11-13T22:47:36Z",
  /** caught at T+22:32:30 → 2026-11-14 21:20:06 UTC (3:20 pm CST) */
  catchUtc: "2026-11-14T21:20:06Z",
  orbitAltitudeKm: 275,
} as const

export const formatUtc = (jd: number): string => new Date((jd - 2440587.5) * 86400000).toISOString().replace(".000", "")
