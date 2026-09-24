"""Validação pontual do artigo usando Backlash-K0."""

from pathlib import Path

import numpy as np

from bifurcation_validation import build_multirotor, run_at_speed


BASE_DIR = Path(__file__).resolve().parent
CSV_DIR = Path(r"C:\Users\M\Desktop\doutorado\teste_backlash_ross\mestrado\cvs_paper")
OUTPUT_DIR = BASE_DIR / "validation_results_K0"


def run_validation(speeds=(1000, 3000, 4500, 6000), sim_cycles=30):
    OUTPUT_DIR.mkdir(exist_ok=True)
    results = {}
    for speed in speeds:
        model = build_multirotor()
        backlash, idx_x1 = run_at_speed(model, speed, n_cicles=sim_cycles, cut_cicles=0)
        data = {
            "time": backlash.time,
            "x1": backlash.time_response.yout[:, idx_x1],
            "delta": backlash.backlash_results["delta"],
            "contact_force": backlash.backlash_results["Fm"],
        }
        np.savez(OUTPUT_DIR / f"validation_{speed}rpm_K0.npz", **data)
        results[speed] = data
    return results


if __name__ == "__main__":
    run_validation()
