#!/usr/bin/env python3
"""
Dual-Slope IADC Integrator
"""

import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

# Parameter aus Spec/Bericht
R = 100e3
C = 204.8e-12               # Bericht
RC = R * C                  # 
A_NOM = 10000               # Spec

V_DD = 1.2
V_CM = 0.75                 # Spec
V_REF_P = 1.0
V_REF_N = 0.5
V_REF_DIFF = V_REF_P - V_REF_N  # +0.5 V

T_CLK = 20e-9
N_ACC = 1024
T_ACC = N_ACC * T_CLK       # entspricht RC
V_LSB = V_REF_DIFF / N_ACC



class DiffIntegrator:
    """Diff-Integrator (Finite Gain). Siehe Gl. 32. Bericht"""
    
    def __init__(self, A=A_NOM, Vos=0.0):
        self.A = A
        self.Vos = Vos
        self.tau = RC
        self.tau_eff = A * RC
        self.Vdiff = 0.0
    
    def reset(self):
        self.Vdiff = 0.0
    
    def get_outputs(self):
        Vp = V_CM + self.Vdiff / 2
        Vn = V_CM - self.Vdiff / 2
        return Vp, Vn, self.Vdiff
    
    def step_euler(self, Vin_diff, dt):
        """Euler-Schritt. dV/dt = -V/(A*RC) - Vin/RC"""
        vin_eff = Vin_diff + self.Vos
        dV = (-self.Vdiff / self.tau_eff - vin_eff / self.tau) * dt
        self.Vdiff += dV
        # print(f"  step: vin_eff={vin_eff:.4f} dV={dV:.6f} Vdiff={self.Vdiff:.4f}")
        return self.get_outputs()
    
    def ramp_exact(self, Vin_diff, t):
        """Exakte Loesung fuer konstanten Eingang."""
        vin_eff = Vin_diff + self.Vos
        t = np.asarray(t)
        exp_term = np.exp(-t / self.tau_eff)
        Vdiff = -self.A * vin_eff * (1 - exp_term)
        return Vdiff
    
    def ramp_ideal(self, Vin_diff, t):
        """Idealer Integrator (A -> inf)"""
        vin_eff = Vin_diff + self.Vos
        return -vin_eff * np.asarray(t) / self.tau

def run_dual_slope(Vin_diff, A=A_NOM, Vos=0.0, dt=T_CLK):
    """
    Simuliert kompletten Zyklus.
    Phase 1: Integration
    Phase 2: Deintegration bis Zero-Crossing
    """
    integ = DiffIntegrator(A=A, Vos=Vos)
    
    t_list, vp_list, vn_list, vdiff_list = [], [], [], []
    t = 0.0
    
    def record():
        Vp, Vn, Vd = integ.get_outputs()
        t_list.append(t)           # aktuelle Zeit
        vp_list.append(Vp)         # Vout_p nach dem letzten Update
        vn_list.append(Vn)         # Vout_n nach dem letzten Update
        vdiff_list.append(Vd)      # Vdiff nach dem letzten Update
    record()
    
    # Integration
    for _ in range(N_ACC):
        integ.step_euler(Vin_diff, dt)
        t += dt
        record()

    V_peak = integ.Vdiff
    t_acc_end = t
    # print(f"DEBUG: V_peak={V_peak:.4f}")  # bei Bedarf einkommentieren
    
    # Deintegration
    # neg. V_peak -> neg. V_deint damit Vout steigt
    if V_peak < 0:
        V_deint = -V_REF_DIFF
    else:
        V_deint = V_REF_DIFF
    # print(f"Deint: V_peak={V_peak:.4f} -> V_deint={V_deint}")
    
    Z = 0 
    max_cycles = 2 * N_ACC  # sollte nie erreicht werden!
    
    for _ in range(max_cycles):
        v_prev = integ.Vdiff
        integ.step_euler(V_deint, dt)
        t += dt
        Z += 1
        record()  # Vout_p, Vout_n, Vdiff für den aktuellen Integratorschritt speichern
        if v_prev * integ.Vdiff<= 0:
            # print(f"ZC @ cycle {Z}: v_prev={v_prev:.5f} -> {integ.Vdiff:.5f}")
            break 
    return {
        't': np.array(t_list),
        'Vout_p': np.array(vp_list),
        'Vout_n': np.array(vn_list),
        'Vdiff': np.array(vdiff_list),
        'Z': Z,
        'V_peak': V_peak,
        't_acc_end': t_acc_end
    }


def save_dat(fpath, **columns):
    """Speichert Spalten als DAT fuer pgfplots."""
    keys = list(columns.keys())
    data = np.column_stack([columns[k] for k in keys])
    hdr = " ".join(keys)
    with open(fpath, 'w') as f:
        f.write("# %s\n" % hdr)
        for row in data:
            f.write(" ".join(f"{v:.7e}" for v in row) + "\n")


def generate_plots(out_dir):
    out_dir = Path(out_dir)
    out_dir.mkdir(exist_ok=True)
    
    print("IADC Integrator Simulation:")
    print("R=%.0fkOhm, C=%.1fpF, RC=%.2fus" % (R/1e3, C*1e12, RC*1e6))
    print(f"V_CM={V_CM}V, V_ref={V_REF_DIFF}V, A={A_NOM}")
    print("T_acc=%.2fus (%d cycles)\n" % (T_ACC*1e6, N_ACC))
    # Plot 1: Dual-Slope Vdiff
    print("[1] Dual-Slope Zyklen (Vdiff)")
    for Vin in [0.1, 0.25, 0.4, 0.5]:
        res = run_dual_slope(-Vin)
        label = f"{int(Vin*1000)}mV"
        save_dat(out_dir / f"dualslope_{label}.dat",
                 t_us=res['t']*1e6,
                 Vdiff=res['Vdiff'],
                 Vout_p=res['Vout_p'],
                 Vout_n=res['Vout_n'])
        
        Z_expected = int(round(Vin / V_REF_DIFF * N_ACC))
        print(f"   Vin={Vin}V: Z={res['Z']} (erwartet: {Z_expected})") 
    # Plot 2: Single-ended
    print("\n[2] Single-ended Outputs")
    for Vin in [0.1, 0.25, 0.4, 0.5]:
        res = run_dual_slope(-Vin)
        label = f"{int(Vin*1000)}mV"
        save_dat(out_dir / f"singleended_{label}.dat",
                 t_us=res['t']*1e6,
                 Vout_p=res['Vout_p'],
                 Vout_n=res['Vout_n'])
        print("   Vin={}V: Vout_p=[{:.3f}, {:.3f}]V".format(Vin, min(res['Vout_p']), max(res['Vout_p'])))
    make_png(out_dir)
    print(f"\nDaten gespeichert in: {out_dir.absolute()}")


def make_png(out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    colors = ['#1f77b4', '#2ca02c', '#ff7f0e', '#d62728']  
    # Dual-Slope Vdiff
    ax = axes[0]
    for vin, col in zip([0.1, 0.25, 0.4, 0.5], colors):
        res = run_dual_slope(-vin)
        ax.plot(res['t']*1e6, res['Vdiff'], col, lw=1.2, 
                label=f'$V_{{in}}$={vin}V, Z={res["Z"]}')
    
    ax.axhline(0, color='k', lw=0.5, ls=':')
    ax.axvline(T_ACC*1e6, color='gray', lw=1, ls='--')
    ax.set_xlabel('t [us]')
    ax.set_ylabel('$V_{out,diff}$ [V]')
    ax.set_title('Dual-Slope Conversion')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.text(T_ACC*1e6/2, 0.42, 'Integration', ha='center', fontsize=9, color='gray')
    ax.text(T_ACC*1e6*1.3, 0.25, 'Deintegration', ha='center', fontsize=9, color='gray')
    # Single-ended
    ax = axes[1]
    for Vin, col in zip([0.1, 0.25, 0.4, 0.5], colors):
        res = run_dual_slope(-Vin)
        ax.plot(res['t']*1e6, res['Vout_p'], col, lw=1.2, label=f'$V_{{in}}$={Vin}V')
        ax.plot(res['t']*1e6, res['Vout_n'], col, lw=1.2, ls='--', alpha=0.7)
    ax.axhline(V_CM, color='gray', lw=1, ls='--')
    ax.axhline(V_DD, color='k', lw=0.8, ls=':')
    ax.axhline(0, color='k', lw=0.8, ls=':')
    ax.axvline(T_ACC*1e6, color='gray', lw=1, ls='--')
    ax.set_xlabel('t [us]')
    ax.set_ylabel('$V_{out}$ [V]')
    ax.set_title('Single-Ended Outputs (solid: $V_{out,p}$, dashed: $V_{out,n}$)')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_ylim(-0.05, 1.25)
    ax.text(0.02, V_CM+0.03, '$V_{CM}$', fontsize=8, color='gray', transform=ax.get_yaxis_transform())
    ax.text(0.02, V_DD+0.03, '$V_{DD}$', fontsize=8, color='k', transform=ax.get_yaxis_transform())
    plt.tight_layout()
    plt.savefig(out_dir / "integrator_plots.png", dpi=150)
    plt.close()
    print("[PNG] integrator_plots.png erstellt")


if __name__ == "__main__":
    generate_plots("./plot_data")
