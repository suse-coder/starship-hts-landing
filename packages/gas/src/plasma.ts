import { E_CHARGE, EV, H_PLANCK, K_B, M_E } from "@ship/earth"
import { S, type AirState } from "./air"

/** Electron–neutral momentum-transfer cross sections at ~0.3–1 eV, m² (Itikawa compilations, rounded). */
const Q_EN: Readonly<Record<number, number>> = {
  [S.N2]: 1.0e-19,
  [S.O2]: 0.6e-19,
  [S.NO]: 1.0e-19,
  [S.N]: 0.6e-19,
  [S.O]: 0.45e-19,
}
export const Q_CS = 3e-18 // e–Cs, m²
const I_CS = 3.894 // eV

export const electronSpeed = (T: number): number => Math.sqrt((8 * K_B * T) / (Math.PI * M_E))

/** electron–neutral collision frequency per electron, s^-1 */
export const nuElectronNeutral = (s: AirState): number => {
  let sum = 0
  for (const k of [S.N2, S.O2, S.NO, S.N, S.O]) sum += s.n[k] * Q_EN[k]
  return sum * electronSpeed(s.T)
}

export const coulombLog = (ne: number, T: number): number => {
  const TeV = (K_B * T) / EV
  const necm = Math.max(ne * 1e-6, 1)
  return Math.max(2, 23 - Math.log(Math.sqrt(necm) * TeV ** -1.5))
}

/** electron–ion collision frequency giving the Spitzer conductivity, s^-1 */
export const nuElectronIon = (ne: number, T: number): number => {
  const TeV = (K_B * T) / EV
  return 1.47e-12 * ne * coulombLog(ne, T) * TeV ** -1.5
}

/** Saha ionisation of a caesium seed at (T) into a background electron density neAir. Returns n(Cs+). */
export const cesiumIonisation = (T: number, nCs: number, neAir: number): number => {
  if (nCs <= 0) return 0
  const K = ((2 * Math.PI * M_E * K_B * T) / (H_PLANCK * H_PLANCK)) ** 1.5 * Math.exp((-I_CS * EV) / (K_B * T))
  const b = neAir + K
  // x² + b x − K nCs = 0, stable form
  return (2 * K * nCs) / (b + Math.sqrt(b * b + 4 * K * nCs))
}

export interface PlasmaProps {
  readonly ne: number
  readonly nu: number // total momentum-transfer collision frequency
  readonly sigma: number // scalar conductivity S/m
  readonly fp: number // plasma frequency Hz
}

export const plasmaProps = (ne: number, nuEn: number, T: number): PlasmaProps => {
  const nu = nuEn + nuElectronIon(ne, T) + 1
  const sigma = (ne * E_CHARGE * E_CHARGE) / (M_E * nu)
  return { ne, nu, sigma, fp: 8.98 * Math.sqrt(Math.max(ne, 0)) }
}

export const hallParameter = (B: number, nu: number): number => (E_CHARGE * B) / (M_E * nu)
export const cyclotronFreq = (B: number): number => (E_CHARGE * B) / (2 * Math.PI * M_E)
/** critical (cut-off) electron density for an unmagnetised plasma at frequency f */
export const criticalDensity = (f: number): number => 1.2404e-2 * f * f
