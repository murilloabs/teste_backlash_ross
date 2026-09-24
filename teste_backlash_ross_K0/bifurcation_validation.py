"""Varredura de bifurcação usando a variante Backlash-K0."""

from copy import deepcopy
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import ross as rs
from tqdm import tqdm
import plotly.graph_objects as go

from backlash_ross import Backlash


BASE_DIR = Path(__file__).resolve().parent


def build_multirotor():
    z1 = z2 = 20
    module = 0.01
    pressure_angle = np.radians(20.0)
    width = 0.030
    mass = 6.57
    inertia = 0.0365
    steel = rs.Material(name="Steel", rho=7850, E=2e11, Poisson=0.3)
    stiff_steel = rs.Material(name="Steel_Stiff", rho=0.01, E=1e15, Poisson=0.3)
    shaft = [rs.ShaftElement(L=0.0001, idl=0.0, odl=0.0001, material=stiff_steel, n=0)]
    bearing = rs.BearingElement(n=0, kxx=1e8, kyy=1e8, cxx=512.64, cyy=512.64)
    pitch_diameter = module * z1
    bore = np.sqrt(pitch_diameter**2 - 4 * mass / (np.pi * width * steel.rho))
    gear = rs.GearElementTVMS(
        n=0, material=steel, width=width, bore_diameter=bore,
        module=module, n_teeth=z1, pr_angle=pressure_angle, helix_angle=0,
        addendum_coeff=1, tip_clearance_coeff=0.25,
    )
    gear.m, gear.Ip, gear.Id = mass, inertia, 0.0001 * inertia / 2
    rotor = rs.Rotor(shaft_elements=shaft, disk_elements=[gear], bearing_elements=[bearing])
    return rs.MultiRotor(
        driving_rotor=rotor,
        driven_rotor=deepcopy(rotor),
        coupled_nodes=(0, 0),
        update_mesh_stiffness=True,
        square_varying_stiffness={"enable": True, "amplitude_ratio": 0.275},
        orientation_angle=0.0,
        position="above",
    )


def run_at_speed(multirotor, speed_rpm, n_cicles=20, cut_cicles=10):
    speed = speed_rpm * np.pi / 30.0
    backlash = Backlash(
        multirotor, speed, b0=50e-6, error_amp=20e-6,
        gear_mesh_stiffness=None, num_points_cicle=1500,
        n_cicles=n_cicles, cut_cicles=cut_cicles,
        use_multirotor_coupling_stiffness=False,
        compute_contact_ratio=True, mesh_damping_ratio=0.07,
    )
    backlash._get_or_create_stiffness_table(
        square_varying_stiffness=True, kd=6.5072e8, ks=3.6228e8, n_poits=1000
    )
    gears = [int(e.n) for e in multirotor.disk_elements if isinstance(e, rs.GearElement)]
    force = np.zeros((len(backlash.time), multirotor.ndof))
    force[:, gears[0] * multirotor.number_dof + 5] = 300 + 100 * np.sin(speed * backlash.time)
    driven_speed = multirotor.mesh.gear_ratio * speed
    force[:, gears[1] * multirotor.number_dof + 5] = 300 + 100 * np.sin(driven_speed * backlash.time)
    backlash.run_dynamic_backlash(
        gears, [0.0, 0.0], [0.0, 0.0], integration_method="internal_newmark",
        gamma=0.5, beta=0.25, tol=1e-6, sigma=1e5,
        smooth_operator=False, add_force=force,
    )
    idx_x1 = gears[0] * multirotor.number_dof
    return backlash, idx_x1


def varredura_bifurcacao(rpm_min=1000, rpm_max=8000, num_steps=300):
    model = build_multirotor()
    speeds, displacements = [], []
    for rpm in tqdm(np.linspace(rpm_min, rpm_max, num_steps), desc="Bifurcação K0"):
        backlash, idx_x1 = run_at_speed(model, rpm)
        mesh_period = 2 * np.pi / (rpm * np.pi / 30 * 20)
        times = backlash.time
        samples = np.interp(
            np.arange(times[0], times[-1], mesh_period),
            times, backlash.time_response.yout[:, idx_x1],
        )
        speeds.extend([rpm / 1000] * len(samples))
        displacements.extend(samples * 1e6)
    return np.asarray(speeds), np.asarray(displacements)


# def plotar_diagrama(speed, displacement, output="diagrama_bifurcacao_K0"):
#     output = BASE_DIR / output
#     plt.figure(figsize=(10, 6), dpi=200)
#     plt.scatter(speed, displacement, s=0.5, c="magenta", alpha=0.6)
#     plt.xlabel("Rotational speed n1 / (kr/min)")
#     plt.ylabel("Displacement x1 / micrometers")
#     plt.tight_layout()
#     plt.savefig(output.with_suffix(".pdf"), bbox_inches="tight")
#     plt.close()

def plotar_diagrama(x_data, y_data, filename_base="diagrama_bifurcacao"):
    """
    Gera o gráfico estilo artigo em PDF e uma versão interativa em HTML.
    """
    print(f"\nSalvando gráficos... Aguarde.")

    # --- INÍCIO DA ADIÇÃO: Captura o diretório onde o arquivo atual está ---
    try:
        diretorio_base = os.path.dirname(os.path.abspath(__file__))
    except NameError:
        diretorio_base = os.getcwd()
    # --- FIM DA ADIÇÃO ---
    
    # =========================================================================
    # 1. VERSÃO PDF (MATPLOTLIB) - Foco em Qualidade para Publicação
    # =========================================================================
    plt.rcParams['font.family'] = 'serif'
    plt.rcParams['font.serif'] = ['Times New Roman']
    plt.rcParams['mathtext.fontset'] = 'stix'
    
    fig, ax = plt.subplots(figsize=(10, 6), dpi=300)
    ax.scatter(x_data, y_data, s=0.5, c='magenta', alpha=0.6, edgecolors='none')
    
    ax.set_xlim(1, 8)
    
    ymin, ymax = np.min(y_data), np.max(y_data)
    margem = (ymax - ymin) * 0.1
    ax.set_ylim(ymin - margem, ymax + margem)
    
    ax.set_xlabel(r'$\mathbf{Rotational\ speed\ \mathit{n}_1 / (kr/min)}$', fontsize=14, fontweight='bold')
    ax.set_ylabel(r'$\mathbf{Displacement\ \mathit{x}_1 / \mu m}$', fontsize=14, fontweight='bold')
    
    for spine in ax.spines.values():
        spine.set_linewidth(1.5)
    ax.tick_params(direction='in', width=1.5, length=6, labelsize=12, top=True, right=True)

    plt.tight_layout()
    
    # Salva em PDF (Vetorizado, sem perda de qualidade)
    # --- ALTERAÇÃO AQUI: Salva no diretorio_base ---
    pdf_path = os.path.join(diretorio_base, f"{filename_base}.pdf")
    plt.savefig(pdf_path, dpi=300, format='pdf', bbox_inches='tight')
    print(f" -> PDF salvo com sucesso: '{pdf_path}'")
    
    # Opcional: Salvar em PNG também para visualização rápida
    # plt.savefig(f"{filename_base}.png", dpi=300, bbox_inches='tight')
    
    plt.close() # Fecha a figura para liberar memória

    # =========================================================================
    # 2. VERSÃO HTML (PLOTLY) - Foco em Análise Interativa (Zoom, Hover)
    # =========================================================================
    # Usamos Scattergl (WebGL) porque diagramas de bifurcação têm MUITOS pontos 
    # e o Scatter normal faria o seu navegador travar.
    fig_html = go.Figure()
    
    fig_html.add_trace(go.Scattergl(
        x=x_data, 
        y=y_data, 
        mode='markers',
        marker=dict(size=3, color='magenta', opacity=0.5),
        name='Poincaré Points'
    ))
    
    fig_html.update_layout(
        title="Diagrama de Bifurcação - Análise Interativa",
        xaxis_title="Rotational speed n1 / (kr/min)",
        yaxis_title="Displacement x1 / μm",
        template="plotly_white",
        width=1200,
        height=700,
        hovermode="closest"
    )
    
    # Linha pontilhada só pra marcar o início do caos (ajuste o x=5.1 para o seu caso real)
    fig_html.add_vline(x=5.1, line_dash="dash", line_color="gray", annotation_text="Início do Caos")
    
    # --- ALTERAÇÃO AQUI: Salva no diretorio_base ---
    html_path = os.path.join(diretorio_base, f"{filename_base}_interativo.html")
    fig_html.write_html(html_path, include_plotlyjs="cdn") # cdn deixa o arquivo menor
    print(f" -> HTML interativo salvo com sucesso: '{html_path}'\n")


if __name__ == "__main__":
    x, y = varredura_bifurcacao(rpm_min=1000, rpm_max=8000, num_steps=300)
    np.savez(BASE_DIR / "bifurcation_K0.npz", speed=x, displacement=y)
    plotar_diagrama(x, y)
