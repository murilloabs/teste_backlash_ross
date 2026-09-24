import sys


import shutil  # <-- BIBLIOTECA PARA LER O TAMANHO DO TERMINAL
import os
import csv
import datetime
import numpy as np
from copy import deepcopy as copy
import time
import ross as rs
from ross.results import TimeResponseResults
from numba import njit, objmode
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import pandas as pd
import pickle
from scipy.interpolate import CubicSpline
from scipy.integrate import cumulative_trapezoid

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Polygon
from scipy.signal import detrend  # <-- A MÁGICA QUE SALVA A CÂMERA
from scipy.interpolate import CubicSpline

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Polygon
from matplotlib.ticker import MaxNLocator, FormatStrFormatter


try:
    from tqdm import tqdm
except ImportError:
    tqdm = lambda x, **kwargs: x

__all__ = ["Backlash", "compute_dfft"]


def compute_dfft(x, t, freq_unit="Hz", window="hann"):
    """
    Calcula a FFT nativamente com NumPy (já é ultra-otimizado em C nativo do NumPy).
    Não usamos Numba aqui pois Numba não suporta np.fft e essa função roda 
    apenas no final para plotagem, não afetando o tempo da simulação.
    """
    x = np.asarray(x, dtype=np.float64)
    t = np.asarray(t, dtype=np.float64)
    dt = t[1] - t[0]
    N = len(x)
    
    x_centered = x - np.mean(x)
        
    correction = 1.0
    if window == "hann":
        w = np.hanning(N)
        x_centered = x_centered * w
        correction = 1.0 / np.mean(w) # Compensa a perda de energia da janela

    X_full = np.fft.rfft(x_centered)
    
    freq = np.fft.rfftfreq(N, d=dt)
    amplitude = (2.0 / N) * np.abs(X_full) * correction
    
    if freq_unit == "rad/s": 
        freq = 2.0 * np.pi * freq
    elif freq_unit == "rpm": 
        freq = freq * 60.0

    return freq, amplitude


import sys
import time
import shutil

@njit
def print_integration_progress(step, total_steps, start_time, print_interval, method_name, dt):
    """Exibe o progresso atualizando a mesma linha no terminal com relógio formatado e dt."""
    
    if (step > 0 and step % print_interval == 0):
        with objmode():
            # O bloco objmode não pode importar o módulo time durante a
            # compilação Numba. O cronômetro principal continua sendo medido
            # fora deste trecho; aqui usamos um valor mínimo para o progresso.
            elapsed = 1e-6
            percent = (step / (total_steps - 1)) * 100.0

            if elapsed <= 0.0:
                elapsed = 1e-6  
            
            if elapsed > 0.0 and step > 0:
                speed_it = step / elapsed
                sec_per_it = elapsed / step
                eta = ((total_steps - 1) - step) * sec_per_it
            else:
                speed_it = 0.0
                sec_per_it = 0.0
                eta = 0.0
                
            bar_length = 20 
            filled_len = int(bar_length * percent / 100.0)
            bar = '#' * filled_len + '-' * (bar_length - filled_len)
            
            e_h = int(elapsed // 3600)
            e_m = int((elapsed % 3600) // 60)
            e_s = int(elapsed % 60)
            if e_h > 0:
                str_elapsed = "%dh%02dm%02ds" % (e_h, e_m, e_s)
            else:
                str_elapsed = "%02dm%02ds" % (e_m, e_s)
                
            eta_h = int(eta // 3600)
            eta_m = int((eta % 3600) // 60)
            eta_s = int(eta % 60)
            if eta_h > 0:
                str_eta = "%dh%02dm%02ds" % (eta_h, eta_m, eta_s)
            else:
                str_eta = "%02dm%02ds" % (eta_m, eta_s)

            str_dt = "%.2e" % dt
            str_percent = "%.1f" % percent
            str_speed = "%.1f" % speed_it
            str_sec_per_it = "%.5f" % sec_per_it
            
            raw_msg = str(method_name) + " |" + bar + "| " + str_percent + "% | Passos: " + str(step) + "/" + str(total_steps-1) + " | dt: " + str_dt + " | Tempo: " + str_elapsed + " | ETA: " + str_eta + " | Vel: " + str_speed + " it/s (" + str_sec_per_it + " s/it)"
            
            # Evita acessar `shutil` e `sys` dentro do bloco objmode, pois
            # esses módulos não são resolvidos pelo compilador Numba.
            print(raw_msg)


@njit
def inv(angle):
    """Função involuta do círculo base: inv(a) = tan(a) - a"""
    return np.tan(angle) - angle

@njit
def bilinear_interp(theta, cr, theta_arr, cr_arr, K_table):
    """Interpolação bilinear ultrarrápida 2D para encontrar a Rigidez Instantânea (k_m)."""
    
    theta = theta % theta_arr[-1] 
    
    i = np.searchsorted(theta_arr, theta) - 1
    j = np.searchsorted(cr_arr, cr) - 1
    
    if i < 0: i = 0
    if i >= len(theta_arr) - 1: i = len(theta_arr) - 2
    if j < 0: j = 0
    if j >= len(cr_arr) - 1: j = len(cr_arr) - 2
    
    t1, t2 = theta_arr[i], theta_arr[i+1]
    c1, c2 = cr_arr[j], cr_arr[j+1]
    
    wt = (theta - t1) / (t2 - t1) if t2 != t1 else 0.0
    wc = (cr - c1) / (c2 - c1) if c2 != c1 else 0.0
    
    k00, k10 = K_table[i, j], K_table[i+1, j]
    k01, k11 = K_table[i, j+1], K_table[i+1, j+1]
    
    k0 = k00 * (1 - wt) + k10 * wt
    k1 = k01 * (1 - wt) + k11 * wt
    
    return k0 * (1 - wc) + k1 * wc



    
    

    

    
    
    







    
    









    




@njit
def calculate_dynamic_backlash_force(
    disp_resp, velc_resp, gear_nodes, number_of_dof, ndof_total,
    d0, orientation_angle, R1, R2, alfa0, helix_angle, b0, 
    error_step, angular_pos, compute_cr_flag, nominal_cr,
    Ra1, Ra2, module, sigma, smooth_operator, # <-- Parâmetro Booleano Adicionado
    theta_arr, cr_arr, K_table,
    M_eq, damping_ratio, error_dot_step
):
    idx1, idx2 = number_of_dof * gear_nodes[0], number_of_dof * gear_nodes[1]
    
    x1, y1, z1 = disp_resp[idx1], disp_resp[idx1+1], disp_resp[idx1+2]
    rx1, ry1, t1 = disp_resp[idx1+3], disp_resp[idx1+4], disp_resp[idx1+5]
    
    x2, y2, z2 = disp_resp[idx2], disp_resp[idx2+1], disp_resp[idx2+2]
    rx2, ry2, t2 = disp_resp[idx2+3], disp_resp[idx2+4], disp_resp[idx2+5]

    vx1, vy1, vz1 = velc_resp[idx1], velc_resp[idx1+1], velc_resp[idx1+2]
    vrx1, vry1, vt1 = velc_resp[idx1+3], velc_resp[idx1+4], velc_resp[idx1+5]
    vx2, vy2, vz2 = velc_resp[idx2], velc_resp[idx2+1], velc_resp[idx2+2]
    vrx2, vry2, vt2 = velc_resp[idx2+3], velc_resp[idx2+4], velc_resp[idx2+5]

    cos_ori, sin_ori = np.cos(orientation_angle), np.sin(orientation_angle)
    x2_abs, y2_abs = x2 + d0 * cos_ori, y2 + d0 * sin_ori
    dx, dy = x2_abs - x1, y2_abs - y1
    d_inst = np.sqrt(dx**2 + dy**2)
    if d_inst < 1e-12: d_inst = 1e-12 
    beta = np.arctan2(dy, dx)
    
    cos_alfa_val = (R1 + R2) / d_inst
    if cos_alfa_val > 1.0: cos_alfa_val = 1.0
    elif cos_alfa_val < -1.0: cos_alfa_val = -1.0
    alfa = np.arccos(cos_alfa_val)

    psi = alfa - beta
    sin_psi, cos_psi = np.sin(psi), np.cos(psi)
    sin_beta_h, cos_beta_h = np.sin(helix_angle), np.cos(helix_angle)

    d_inst2 = d_inst**2
    term_in_sqrt = d_inst2 - (R1 + R2)**2
    if term_in_sqrt < 1e-12: term_in_sqrt = 1e-12
    term_sqrt = np.sqrt(term_in_sqrt)

    alfa_x1 = -((R1 + R2) * dx) / (d_inst2 * term_sqrt)
    alfa_y1 = -((R1 + R2) * dy) / (d_inst2 * term_sqrt)
    beta_x1 = dy / d_inst2
    beta_y1 = -dx / d_inst2

    alfa_x2, alfa_y2 = -alfa_x1, -alfa_y1
    beta_x2, beta_y2 = -beta_x1, -beta_y1

    delta = (
        ((x1 - x2) * sin_psi + (y1 - y2) * cos_psi + R1 * t1 + R2 * t2) * cos_beta_h +
        ((-z1 + z2) + (R1 * rx1 + R2 * rx2) * sin_psi + (R1 * ry1 + R2 * ry2) * cos_psi) * sin_beta_h
        - error_step
    )


    

    inv_alfa = np.tan(alfa) - alfa
    inv_alfa0 = np.tan(alfa0) - alfa0
    delta_b = (R1 + R2) * (inv_alfa - inv_alfa0)
    bt = b0 + delta_b * cos_beta_h

    geom_trans = (x1 - x2) * cos_psi - (y1 - y2) * sin_psi
    geom_rot   = (R1 * rx1 + R2 * rx2) * cos_psi - (R1 * ry1 + R2 * ry2) * sin_psi
    
    psi_x1 = alfa_x1 - beta_x1
    psi_y1 = alfa_y1 - beta_y1
    psi_x2 = alfa_x2 - beta_x2
    psi_y2 = alfa_y2 - beta_y2

    d_delta_dx1 = (sin_psi + geom_trans * psi_x1) * cos_beta_h + (geom_rot * psi_x1) * sin_beta_h
    d_delta_dy1 = (cos_psi + geom_trans * psi_y1) * cos_beta_h + (geom_rot * psi_y1) * sin_beta_h
    d_delta_dx2 = (-sin_psi + geom_trans * psi_x2) * cos_beta_h + (geom_rot * psi_x2) * sin_beta_h
    d_delta_dy2 = (-cos_psi + geom_trans * psi_y2) * cos_beta_h + (geom_rot * psi_y2) * sin_beta_h

    d_delta_dz1, d_delta_dz2 = -sin_beta_h, sin_beta_h

    d_delta_drx1 = R1 * sin_psi * sin_beta_h
    d_delta_dry1 = R1 * cos_psi * sin_beta_h
    d_delta_drx2 = R2 * sin_psi * sin_beta_h
    d_delta_dry2 = R2 * cos_psi * sin_beta_h

    d_delta_dt1, d_delta_dt2 = R1 * cos_beta_h, R2 * cos_beta_h

    tan2_alfa = np.tan(alfa)**2
    bt_x1 = (R1 + R2) * tan2_alfa * alfa_x1 * cos_beta_h
    bt_y1 = (R1 + R2) * tan2_alfa * alfa_y1 * cos_beta_h
    bt_x2 = (R1 + R2) * tan2_alfa * alfa_x2 * cos_beta_h
    bt_y2 = (R1 + R2) * tan2_alfa * alfa_y2 * cos_beta_h


    delta_dot = (
        d_delta_dx1 * vx1 + d_delta_dy1 * vy1 + d_delta_dz1 * vz1 +
        d_delta_drx1 * vrx1 + d_delta_dry1 * vry1 + d_delta_dt1 * vt1 +
        d_delta_dx2 * vx2 + d_delta_dy2 * vy2 + d_delta_dz2 * vz2 +
        d_delta_drx2 * vrx2 + d_delta_dry2 * vry2 + d_delta_dt2 * vt2
        - error_dot_step
    )

    alfa_dot = alfa_x1 * vx1 + alfa_y1 * vy1 + alfa_x2 * vx2 + alfa_y2 * vy2
    bt_dot = (R1 + R2) * tan2_alfa * alfa_dot * cos_beta_h

    
    if smooth_operator:
        x1_val = delta - bt
        x2_val = delta + bt
        
        tanh_x1 = np.tanh(sigma * x1_val)
        tanh_x2 = np.tanh(sigma * x2_val)
        
        g1 = x1_val * tanh_x1
        g2 = x2_val * tanh_x2
        
        f_val = delta + 0.5 * (g1 - g2)
        
        gp1 = tanh_x1 + sigma * x1_val * (1.0 - tanh_x1**2)
        gp2 = tanh_x2 + sigma * x2_val * (1.0 - tanh_x2**2)
        
        df_ddelta = 1.0 + 0.5 * (gp1 - gp2)
        df_dbt    = 0.5 * (-gp1 - gp2)
        
        f1_val = df_ddelta * delta_dot + df_dbt * bt_dot
        
        f_x1 = d_delta_dx1 * df_ddelta + bt_x1 * df_dbt
        f_y1 = d_delta_dy1 * df_ddelta + bt_y1 * df_dbt
        f_z1 = d_delta_dz1 * df_ddelta
        f_rx1 = d_delta_drx1 * df_ddelta
        f_ry1 = d_delta_dry1 * df_ddelta
        f_t1  = d_delta_dt1 * df_ddelta
        
        f_x2 = d_delta_dx2 * df_ddelta + bt_x2 * df_dbt
        f_y2 = d_delta_dy2 * df_ddelta + bt_y2 * df_dbt
        f_z2 = d_delta_dz2 * df_ddelta
        f_rx2 = d_delta_drx2 * df_ddelta
        f_ry2 = d_delta_dry2 * df_ddelta
        f_t2  = d_delta_dt2 * df_ddelta

    else:
        if delta > bt: 
            f_val = delta - bt
            f1_val = delta_dot - bt_dot  # <-- Adicionado -bt_dot (Eq. 10)
            sgn = 1.0
        elif delta < -bt: 
            f_val = delta + bt
            f1_val = delta_dot + bt_dot  # <-- Adicionado +bt_dot (Eq. 10)
            sgn = -1.0
        else:
            f_val = 0.0
            f1_val = 0.0
            sgn = 0.0
            
        if sgn != 0.0:
            f_x1 = d_delta_dx1 - sgn * bt_x1
            f_y1 = d_delta_dy1 - sgn * bt_y1
            f_z1 = d_delta_dz1
            f_rx1 = d_delta_drx1
            f_ry1 = d_delta_dry1
            f_t1  = d_delta_dt1
            
            f_x2 = d_delta_dx2 - sgn * bt_x2
            f_y2 = d_delta_dy2 - sgn * bt_y2
            f_z2 = d_delta_dz2
            f_rx2 = d_delta_drx2
            f_ry2 = d_delta_dry2
            f_t2  = d_delta_dt2
        else:
            f_x1 = f_y1 = f_z1 = f_rx1 = f_ry1 = f_t1 = 0.0
            f_x2 = f_y2 = f_z2 = f_rx2 = f_ry2 = f_t2 = 0.0


    contact_ratio = nominal_cr
    if compute_cr_flag:
        pb = np.pi * module * np.cos(alfa0) 
        contact_ratio = (np.sqrt(Ra1**2 - R1**2) + np.sqrt(Ra2**2 - R2**2) - d_inst * np.sin(alfa)) / pb

    K_time_step = bilinear_interp(angular_pos, contact_ratio, theta_arr, cr_arr, K_table)
    c_m = 2.0 * damping_ratio * np.sqrt(K_time_step * M_eq)

    Fm = K_time_step * f_val + c_m * f1_val

    
    backlash_force = np.zeros(ndof_total)

    backlash_force[idx1]   = -Fm * f_x1 
    backlash_force[idx1+1] = -Fm * f_y1 
    backlash_force[idx1+2] = -Fm * f_z1 
    backlash_force[idx1+3] = -Fm * f_rx1 
    backlash_force[idx1+4] = -Fm * f_ry1 
    backlash_force[idx1+5] = -Fm * f_t1 
    
    backlash_force[idx2]   = -Fm * f_x2 
    backlash_force[idx2+1] = -Fm * f_y2 
    backlash_force[idx2+2] = -Fm * f_z2 
    backlash_force[idx2+3] = -Fm * f_rx2 
    backlash_force[idx2+4] = -Fm * f_ry2 
    backlash_force[idx2+5] = -Fm * f_t2 

    logs = np.array([
        x1, y1, x2_abs, y2_abs, t1, t2, 
        d_inst, beta, alfa, contact_ratio, delta, bt, f_val, K_time_step, Fm
    ], dtype=np.float64)

    return backlash_force, logs



@njit
def newmark_predict(ny, y0, ydot0, y2dot0, dt, gamma, beta):
    y2dot = np.zeros(ny)
    ydot = ydot0 + y2dot0 * (1.0 - gamma) * dt
    y = y0 + ydot0 * dt + y2dot0 * (0.5 - beta) * (dt**2)
    return y, ydot, y2dot

@njit
def newmark_calc_rotor_res(y, ydot, y2dot, t_eval, F_unb_eval, M, C, K, 
                           gear_nodes, number_of_dof, ndof_total, d0, orientation_angle, 
                           R1, R2, alfa0, helix_angle, b0, compute_cr_flag, nominal_cr,
                           Ra1, Ra2, module, sigma, smooth_operator, theta_arr, cr_arr, K_table,
                           M_eq, damping_ratio, 
                           theta_eval, error_eval, error_dot_eval):


    F_backlash, logs = calculate_dynamic_backlash_force(
        y, ydot, gear_nodes, number_of_dof, ndof_total,
        d0, orientation_angle, R1, R2, alfa0, helix_angle, b0,
        error_eval, theta_eval, compute_cr_flag, nominal_cr, # <-- PASSANDO OS AVALIADOS
        Ra1, Ra2, module, sigma, smooth_operator, theta_arr, cr_arr, K_table,
        M_eq, damping_ratio, error_dot_eval                  # <-- PASSANDO O AVALIADO
    )
    F_total = F_unb_eval + F_backlash
    res = F_total - (M @ y2dot + C @ ydot + K @ y)
    return res, F_backlash, logs

@njit
def newmark_build_jacobian(y, ydot, y2dot, dt, gamma, beta, t_eval, F_unb_eval, res_base, 
                           M, C, K, active_dofs, epsilon, gear_nodes, number_of_dof, ndof_total, 
                           d0, orientation_angle, R1, R2, alfa0, helix_angle, b0, 
                           compute_cr_flag, nominal_cr, Ra1, Ra2, module, sigma, smooth_operator, theta_arr, cr_arr, K_table,
                           M_eq, damping_ratio, theta_eval, error_eval, error_dot_eval): # <-- LIMPO
    
    J = M + C * (gamma * dt) + K * (beta * (dt**2))
    
    F_nl_base, _ = calculate_dynamic_backlash_force(
        y, ydot, gear_nodes, number_of_dof, ndof_total,
        d0, orientation_angle, R1, R2, alfa0, helix_angle, b0,
        error_eval, theta_eval, compute_cr_flag, nominal_cr, 
        Ra1, Ra2, module, sigma, smooth_operator, theta_arr, cr_arr, K_table,
        M_eq, damping_ratio, error_dot_eval                  
    )
    
    for i in active_dofs:
        y_orig, ydot_orig = y[i], ydot[i]

        y[i] = y_orig + epsilon * beta * (dt**2)
        ydot[i] = ydot_orig + epsilon * gamma * dt

        F_nl_pert, _ = calculate_dynamic_backlash_force(
            y, ydot, gear_nodes, number_of_dof, ndof_total,
            d0, orientation_angle, R1, R2, alfa0, helix_angle, b0,
            error_eval, theta_eval, compute_cr_flag, nominal_cr, 
            Ra1, Ra2, module, sigma, smooth_operator, theta_arr, cr_arr, K_table,
            M_eq, damping_ratio, error_dot_eval                  
        )

        dF_nl_daccel = (F_nl_pert - F_nl_base) / epsilon

        for j in active_dofs:
            J[j, i] -= dF_nl_daccel[j]

        y[i], ydot[i] = y_orig, ydot_orig

    return J

    





@njit
def newmark_converge_nr(y0, ydot0, y2dot0, dt_sub, gamma, beta, tol, epsilon, t_eval, F_unb_eval, 
                        M, C, K, active_dofs, gear_nodes, number_of_dof, ndof_total, d0, 
                        orientation_angle, R1, R2, alfa0, helix_angle, b0, 
                        compute_cr_flag, nominal_cr, Ra1, Ra2, module, sigma, smooth_operator, theta_arr, cr_arr, K_table,
                        M_eq, damping_ratio, theta_eval, error_eval, error_dot_eval): # <-- LIMPO
    ny = len(y0)
    y, ydot, y2dot = newmark_predict(ny, y0, ydot0, y2dot0, dt_sub, gamma, beta)
    
    convergiu, need_rebuild = False, True
    norm_res = 0.0
    J = np.zeros((ny, ny))
    F_b_out = np.zeros(ny)
    logs_out = np.zeros(15)
    
    for nr_iter in range(1, 16):
        res_base, F_b, logs = newmark_calc_rotor_res(
            y, ydot, y2dot, t_eval, F_unb_eval, M, C, K, gear_nodes, number_of_dof, 
            ndof_total, d0, orientation_angle, R1, R2, alfa0, helix_angle, b0, 
            compute_cr_flag, nominal_cr, Ra1, Ra2, module, sigma, smooth_operator, theta_arr, cr_arr, K_table, 
            M_eq, damping_ratio, theta_eval, error_eval, error_dot_eval # <-- CHAMADA LIMPA
        )
        norm_res = np.linalg.norm(res_base)

        if norm_res < tol:
            convergiu = True
            F_b_out, logs_out = F_b, logs
            break

        need_rebuild = True

        if need_rebuild or nr_iter % 5 == 0:
            J = newmark_build_jacobian(
                y, ydot, y2dot, dt_sub, gamma, beta, t_eval, F_unb_eval, res_base, 
                M, C, K, active_dofs, epsilon, gear_nodes, number_of_dof, ndof_total, d0, 
                orientation_angle, R1, R2, alfa0, helix_angle, b0, 
                compute_cr_flag, nominal_cr, Ra1, Ra2, module, sigma, smooth_operator, theta_arr, cr_arr, K_table, 
                M_eq, damping_ratio, theta_eval, error_eval, error_dot_eval # <-- CHAMADA LIMPA
            )
            need_rebuild = False

        dy2dot = np.linalg.solve(J, res_base)
        y2dot += dy2dot
        ydot += dy2dot * gamma * dt_sub
        y += dy2dot * beta * (dt_sub**2)

    return y, ydot, y2dot, nr_iter, convergiu, norm_res, F_b_out, logs_out

@njit
def newmark_solver_full(t_array, yout, logs_matrix, force_matrix, F_unb,
                        M, C, K, active_dofs, gamma, beta, tol, epsilon,
                        gear_nodes, number_of_dof, ndof_total, d0, 
                        orientation_angle, R1, R2, alfa0, helix_angle, b0, 
                        compute_cr_flag, nominal_cr, Ra1, Ra2, 
                        module, sigma, smooth_operator, theta_arr, cr_arr, K_table,
                        M_eq, damping_ratio, start_time, y_init, ydot_init, y2dot_init,
                        theta_array, error_array, error_dot_array):  # <-- LIMPO
    
    n_steps = len(t_array)
    ny = ndof_total
    dt_macro = t_array[1] - t_array[0]
    dt_min = dt_macro * 1e-5 
    time_tol = dt_macro * 1e-6 

    y0 = np.copy(y_init)
    ydot0 = np.copy(ydot_init)
    y2dot0 = np.copy(y2dot_init)
    
    _, fb0, logs_0 = newmark_calc_rotor_res(
            y0, ydot0, y2dot0, t_array[0], F_unb[0], M, C, K, gear_nodes, number_of_dof, 
            ndof_total, d0, orientation_angle, R1, R2, alfa0, helix_angle, b0, 
            compute_cr_flag, nominal_cr, Ra1, Ra2, module, sigma, smooth_operator, theta_arr, cr_arr, K_table, 
            M_eq, damping_ratio, theta_array[0], error_array[0], error_dot_array[0] 
    )
    yout[0, :] = y0
    for col in range(15): logs_matrix[0, col] = logs_0[col]
    for dof in range(ndof_total): force_matrix[0, dof] = fb0[dof]

    print_interval = max(1, n_steps // 20)
    dt_sub = dt_macro 

    for step in range(1, n_steps):
        t_target, t_current = t_array[step], t_array[step - 1]
        F_unb_prev, F_unb_next = F_unb[step - 1], F_unb[step]

        while (t_target - t_current) > time_tol:
            if (t_current + dt_sub) > (t_target - time_tol):
                dt_current_step = t_target - t_current
                is_last_substep = True
            else:
                dt_current_step = dt_sub
                is_last_substep = False

            t_eval = t_current + dt_current_step
            ratio = (t_eval - t_array[step - 1]) / dt_macro
            F_unb_eval = F_unb_prev + ratio * (F_unb_next - F_unb_prev)

            theta_eval = theta_array[step - 1] + ratio * (theta_array[step] - theta_array[step - 1])
            error_eval = error_array[step - 1] + ratio * (error_array[step] - error_array[step - 1])
            error_dot_eval = error_dot_array[step - 1] + ratio * (error_dot_array[step] - error_dot_array[step - 1])

            y_new, ydot_new, y2dot_new, nr_iter, convergiu, norm_res, F_b_out, logs_out = newmark_converge_nr(
                y0, ydot0, y2dot0, dt_current_step, gamma, beta, tol, epsilon, t_eval, F_unb_eval, 
                M, C, K, active_dofs, gear_nodes, number_of_dof, ndof_total, d0, orientation_angle, 
                R1, R2, alfa0, helix_angle, b0, compute_cr_flag, nominal_cr, 
                Ra1, Ra2, module, sigma, smooth_operator, theta_arr, cr_arr, K_table, 
                M_eq, damping_ratio, theta_eval=theta_eval, error_eval=error_eval, error_dot_eval=error_dot_eval 
            )

            if convergiu:
                t_current += dt_current_step
                y0, ydot0, y2dot0 = y_new, ydot_new, y2dot_new
                
                if nr_iter <= 5: 
                    dt_sub = min(dt_sub * 2.0, dt_macro)
                elif nr_iter >= 10: 
                    dt_sub = max(dt_sub * 0.5, dt_min)
                
                if is_last_substep:
                    yout[step, :] = y0
                    for col in range(15): logs_matrix[step, col] = logs_out[col]
                    for dof in range(ndof_total): force_matrix[step, dof] = F_b_out[dof]
            else:
                dt_sub *= 0.25 
                if dt_sub < dt_min:
                    raise RuntimeError("Newmark divergiu. Limite dt_min atingido.")

            print_integration_progress(step, n_steps, start_time, print_interval, "Newmark Adapt.", dt_sub)

    return ydot0, y2dot0

class Backlash:
    def __init__(self, 
                 multirotor,
                 speed_driving_gear,
                 b0=0.0,
                 error_amp=0.0,
                 gear_mesh_stiffness=None,
                 num_points_cicle=1000, 
                 n_cicles=2, 
                 cut_cicles=1,
                 use_multirotor_coupling_stiffness=False,
                 compute_contact_ratio=True,
                 mesh_damping_ratio=0.07,  # csi (\xi) - Padrão 0.07 segundo Yi et al. (2019)

                 ):            
        
        self.multirotor = copy(multirotor)
        self.speed_driving_gear = speed_driving_gear
        self.b0 = b0
        self.error_amp = error_amp
        self.gear_mesh_stiffness = gear_mesh_stiffness
        self.n_cicles = n_cicles
        self.cut_cicles = cut_cicles
        self.compute_contact_ratio = compute_contact_ratio
        
        self.mesh_damping_ratio = mesh_damping_ratio 

        self.gears = np.array([e for e in self.multirotor.disk_elements if isinstance(e, rs.GearElement)])
        if len(self.gears) != 2:
            raise ValueError("O multirotor deve conter exatamente duas engrenagens acopladas.")

        J1, J2 = self.gears[0].Ip, self.gears[1].Ip
        R1, R2 = self.gears[0].base_radius, self.gears[1].base_radius
        self.M_eq = (J1 * J2) / (J2 * (R1**2) + J1 * (R2**2))

        n_cicles_sim = self.n_cicles + self.cut_cicles
        max_time = n_cicles_sim * (2 * np.pi * self.gears[0].n_teeth) / (self.speed_driving_gear * self.gears[0].n_teeth)
        
        self.num_points_total = num_points_cicle * n_cicles_sim
        self.time = np.linspace(0, max_time, self.num_points_total)
        self.n_cut = num_points_cicle * self.cut_cicles

        wm = self.speed_driving_gear * self.gears[0].n_teeth
        self.error = self.error_amp * np.sin(wm * self.time)

        self.init_backlash_results()

        if not use_multirotor_coupling_stiffness:
            self.multirotor.gear_mesh_stiffness = 0
            self.multirotor.update_mesh_stiffness = False

    def animate_gears(self, scale=150, frames=400, interval=30, revolutions=1, start_time=None, save_path=None):
        """
        Animação de Excelência (Artigo/Dissertação):
        - Eixos formatados com 2 casas decimais (FormatStrFormatter).
        - Grid principal altamente discretizado.
        - Pinhão (Verde Escuro) e Coroa (Amarelo).
        - Dentes NUNCA se atravessam (Trava Cinemática Visual no Raio de Base).
        - Loop Infinito Perfeito (endpoint=False).
        """
        g1, g2 = self.gears[0], self.gears[1]
        z1, z2 = g1.n_teeth, g2.n_teeth
        mod = g1.module
        alpha = g1.pr_angle 
        b0 = getattr(self, 'b0', 0.0) 
        
        Rp1, Rp2 = g1.pitch_diameter / 2, g2.pitch_diameter / 2
        Rb1 = Rp1 * np.cos(alpha) 
        Rb2 = Rp2 * np.cos(alpha) 
        d0 = Rp1 + Rp2  
        ori = getattr(self.multirotor, 'orientation_angle', 0.0)
        
        T_rev = (2 * np.pi) / self.speed_driving_gear
        
        if start_time is None:
            t_start = max(0, self.time[-1] - (revolutions * T_rev))
        else:
            t_start = start_time
            
        t_end = t_start + (revolutions * T_rev)
        t_start = np.clip(t_start, self.time[0], self.time[-1])
        t_end = np.clip(t_end, self.time[0], self.time[-1])
        
        t_frames = np.linspace(t_start, t_end, frames, endpoint=False)

        x1_raw = np.interp(t_frames, self.time, self.backlash_results['x1'])
        y1_raw = np.interp(t_frames, self.time, self.backlash_results['y1'])
        x2_raw = np.interp(t_frames, self.time, self.backlash_results['x2'])
        y2_raw = np.interp(t_frames, self.time, self.backlash_results['y2'])
        t1_raw = np.interp(t_frames, self.time, self.backlash_results['t1'])
        t2_raw = np.interp(t_frames, self.time, self.backlash_results['t2'])
        
        fm_raw = np.interp(t_frames, self.time, self.backlash_results['Fm'])
        delta_raw = np.interp(t_frames, self.time, self.backlash_results['delta'])
        bt_raw = np.interp(t_frames, self.time, self.backlash_results['bt'])

        x1_mean, y1_mean = np.mean(x1_raw), np.mean(y1_raw)
        x2_mean, y2_mean = np.mean(x2_raw), np.mean(y2_raw)
        t1_mean, t2_mean = np.mean(t1_raw), np.mean(t2_raw)

        x1_dyn, y1_dyn = (x1_raw - x1_mean), (y1_raw - y1_mean)
        x2_dyn, y2_dyn = (x2_raw - x2_mean), (y2_raw - y2_mean)
        
        cx1 = x1_dyn * scale
        cy1 = y1_dyn * scale
        cx2_nom, cy2_nom = d0 * np.cos(ori), d0 * np.sin(ori)
        cx2 = cx2_nom + (x2_dyn * scale)
        cy2 = cy2_nom + (y2_dyn * scale)

        dx_vis = (x1_dyn - x2_dyn) * scale
        dy_vis = (y1_dyn - y2_dyn) * scale
        phi1_dyn = (t1_raw - t1_mean) * scale
        
        delta_mean = np.mean(delta_raw)
        delta_dyn = delta_raw - delta_mean
        delta_vis = delta_mean + (delta_dyn * scale)
        
        delta_vis_capped = np.clip(delta_vis, -bt_raw, bt_raw)
        
        phi2_dyn = (delta_vis_capped - dx_vis * np.sin(alpha) - dy_vis * np.cos(alpha) - Rb1 * phi1_dyn) / Rb2
        
        phi1 = (self.speed_driving_gear * t_frames) + t1_mean + phi1_dyn
        phi2 = -(self.speed_driving_gear * (z1 / z2) * t_frames) + t2_mean + phi2_dyn

        def gerar_perfil_involuto(Rp, z, modulo, ang_pressao, b0_linear, offset_fase=0):
            Rb = Rp * np.cos(ang_pressao)
            Ra = Rp + modulo
            Rf = Rp - 1.25 * modulo
            inv = lambda a: np.tan(a) - a
            pitch_angle = 2 * np.pi / z
            reducao_angular_b0 = b0_linear / (2 * Rp * np.cos(ang_pressao)) if b0_linear else 0.0
            half_base = (np.pi / (2 * z)) + inv(ang_pressao) - reducao_angular_b0
            max_param = np.arccos(Rb / Ra)
            t_vals = np.linspace(0, max_param, 15)
            r_inv = Rb / np.cos(t_vals)
            th_inv = inv(t_vals)
            r_poly, th_poly = [], []
            for i in range(z):
                offset = i * pitch_angle + offset_fase
                th_root_start = offset - pitch_angle + half_base
                th_root_end = offset - half_base
                r_poly.extend([Rf, Rf])
                th_poly.extend([th_root_start, th_root_end])
                if Rf < Rb:
                    r_poly.append(Rb)
                    th_poly.append(th_root_end)
                r_poly.extend(r_inv)
                th_poly.extend(offset - half_base + th_inv)
                r_poly.extend(r_inv[::-1])
                th_poly.extend(offset + half_base - th_inv[::-1])
                if Rf < Rb:
                    r_poly.append(Rb)
                    th_poly.append(offset + half_base)
            r_poly.append(r_poly[0])
            th_poly.append(th_poly[0])
            return np.array(r_poly) * np.cos(th_poly), np.array(r_poly) * np.sin(th_poly)

        bt_mean = np.mean(bt_raw)
        base_x1, base_y1 = gerar_perfil_involuto(Rp1, z1, mod, alpha, bt_mean, offset_fase=ori)
        base_x2, base_y2 = gerar_perfil_involuto(Rp2, z2, mod, alpha, bt_mean, offset_fase=ori + np.pi - (np.pi/z2))

        fig = plt.figure(figsize=(15, 7.5))
        gs = fig.add_gridspec(2, 3) 
        
        ax_gears = fig.add_subplot(gs[:, 0:2])
        ax_gears.set_aspect('equal')
        Ra1, Ra2 = Rp1 + 1.25*mod, Rp2 + 1.25*mod
        margem = max(Ra1, Ra2) * 0.15
        ax_gears.set_xlim(-Ra1 - margem, d0 + Ra2 + margem)
        ax_gears.set_ylim(-max(Ra1, Ra2) - margem, max(Ra1, Ra2) + margem)
        
        ax_gears.xaxis.set_major_locator(MaxNLocator(14))
        ax_gears.yaxis.set_major_locator(MaxNLocator(14))
        ax_gears.xaxis.set_major_formatter(FormatStrFormatter('%.2f'))
        ax_gears.yaxis.set_major_formatter(FormatStrFormatter('%.2f'))
        ax_gears.grid(True, linestyle=':', alpha=0.5)
        
        ax_gears.set_title("Dinâmica de Engrenamento e Vibração", fontsize=13, fontweight='bold')
        ax_gears.set_xlabel("Posição Horizontal (m)", fontsize=10)
        ax_gears.set_ylabel("Posição Vertical (m)", fontsize=10)
        
        poly1 = Polygon(np.column_stack((base_x1, base_y1)), closed=True, fill=True, color='#228B22', alpha=0.9, ec='black', lw=0.8)
        poly2 = Polygon(np.column_stack((base_x2, base_y2)), closed=True, fill=True, color='#FFD700', alpha=0.9, ec='black', lw=0.8)
        ax_gears.add_patch(poly1)
        ax_gears.add_patch(poly2)

        orbit1_main, = ax_gears.plot([], [], color='darkgreen', lw=2.5, alpha=0.8)
        orbit2_main, = ax_gears.plot([], [], color='darkgoldenrod', lw=2.5, alpha=0.8)
        mesh_line, = ax_gears.plot([], [], 'k-', lw=1.5, zorder=5)
        center_pts, = ax_gears.plot([], [], 'ko', markersize=7, zorder=6) 
        
        texto_info = ax_gears.text(0.02, 0.97, '', transform=ax_gears.transAxes, fontsize=11, family='monospace',
                                verticalalignment='top', bbox=dict(facecolor='white', alpha=0.95, edgecolor='black'))

        time_ticks = np.linspace(t_frames[0], t_frames[-1], 6)

        ax_dte = fig.add_subplot(gs[0, 2])
        delta_um = delta_raw * 1e6
        bt_um = bt_raw * 1e6
        min_dte_y = min(np.min(delta_um), np.min(bt_um) * 0.8) 
        max_dte_y = max(np.max(delta_um), np.max(bt_um))
        margin_dte = (max_dte_y - min_dte_y) * 0.15 if max_dte_y > min_dte_y else 1.0
        
        ax_dte.set_xlim(t_frames[0], t_frames[-1])
        ax_dte.set_ylim(min_dte_y - margin_dte, max_dte_y + margin_dte)
        
        ax_dte.set_xticks(time_ticks)
        ax_dte.xaxis.set_major_formatter(FormatStrFormatter('%.2f'))
        ax_dte.yaxis.set_major_locator(MaxNLocator(8))
        
        ax_dte.set_title("Erro de Transmissão (δ) vs Folga", fontsize=11, fontweight='bold')
        ax_dte.set_ylabel("Deslocamento (μm)", fontsize=9)
        ax_dte.grid(True, linestyle=':', alpha=0.6)
        
        line_bt, = ax_dte.plot([], [], 'r--', lw=1.5, alpha=0.7, label='+bt (Folga)')
        line_dte, = ax_dte.plot([], [], 'b-', lw=2.0, label='δ (DTE)')
        pt_dte,   = ax_dte.plot([], [], 'bo', markersize=6)
        ax_dte.legend(loc="upper right", fontsize=8)

        ax_fm = fig.add_subplot(gs[1, 2])
        min_fm = np.min(fm_raw)
        max_fm = np.max(fm_raw)
        margin_fm = (max_fm - min_fm) * 0.15 if max_fm > min_fm else 100.0
        
        ax_fm.set_xlim(t_frames[0], t_frames[-1])
        ax_fm.set_ylim(max(0, min_fm - margin_fm), max_fm + margin_fm)
        
        ax_fm.set_xticks(time_ticks)
        ax_fm.xaxis.set_major_formatter(FormatStrFormatter('%.2f'))
        ax_fm.yaxis.set_major_locator(MaxNLocator(8))
        
        ax_fm.set_title("Força de Engrenamento Fm", fontsize=11, fontweight='bold')
        ax_fm.set_ylabel("Força (N)", fontsize=9)
        ax_fm.set_xlabel("Tempo (s)", fontsize=9)
        ax_fm.grid(True, linestyle=':', alpha=0.6)
        
        line_fm, = ax_fm.plot([], [], color='purple', lw=2.0)
        pt_fm,   = ax_fm.plot([], [], 'mo', markersize=6) 
        
        fig.tight_layout()

        def update(i):
            c1, s1 = np.cos(phi1[i]), np.sin(phi1[i])
            poly1.set_xy(np.column_stack((base_x1*c1 - base_y1*s1 + cx1[i], base_x1*s1 + base_y1*c1 + cy1[i])))
            
            c2, s2 = np.cos(phi2[i]), np.sin(phi2[i])
            poly2.set_xy(np.column_stack((base_x2*c2 - base_y2*s2 + cx2[i], base_x2*s2 + base_y2*c2 + cy2[i])))
            
            orbit1_main.set_data(cx1[:i+1], cy1[:i+1])
            orbit2_main.set_data(cx2[:i+1], cy2[:i+1])
            
            fm_atual = fm_raw[i]
            
            if fm_atual > 0.1:
                mesh_line.set_color('#333333') 
                mesh_line.set_linestyle('-')
                mesh_line.set_linewidth(2.5)
                estado = "CONTATO"
            else:
                mesh_line.set_color('#B0B0B0') 
                mesh_line.set_linestyle('--')
                mesh_line.set_linewidth(1.5)
                estado = "FOLGA (BKL)"

            mesh_line.set_data([cx1[i], cx2[i]], [cy1[i], cy2[i]])
            center_pts.set_data([cx1[i], cx2[i]], [cy1[i], cy2[i]])
            
            texto = (f"Tempo   : {t_frames[i]:.4f} s\n"
                    f"Escala  : {scale}x\n"
                    f"Estado  : {estado}\n"
                    f"Fm      : {fm_atual:.1f} N\n"
                    f"DTE (δ) : {delta_raw[i] * 1e6:.2f} μm")
            texto_info.set_text(texto)

            line_bt.set_data(t_frames[:i+1], bt_um[:i+1])
            line_dte.set_data(t_frames[:i+1], delta_um[:i+1])
            pt_dte.set_data([t_frames[i]], [delta_um[i]])
            
            line_fm.set_data(t_frames[:i+1], fm_raw[:i+1])
            pt_fm.set_data([t_frames[i]], [fm_raw[i]])
            
            return poly1, poly2, orbit1_main, orbit2_main, mesh_line, center_pts, texto_info, line_bt, line_dte, pt_dte, line_fm, pt_fm

        ani = FuncAnimation(fig, update, frames=frames, interval=interval, blit=True)

        if save_path:
            print(f"Renderizando Dashboard Final com Escalas Refinadas... Aguarde.")
            ani.save(save_path, writer='pillow')
            print(f"--> Animação salva em: {save_path}")
        else:
            plt.show()

    def init_backlash_results(self):
        """Inicializa o dicionário global que armazena os dados instantâneos."""
        def zeros_arr(): return np.zeros(self.num_points_total)
        self.backlash_total_force = np.zeros((self.num_points_total, self.multirotor.ndof))

        self.backlash_results = {
            "x1": zeros_arr(), "y1": zeros_arr(), "x2": zeros_arr(), "y2": zeros_arr(),
            "t1": zeros_arr(), "t2": zeros_arr(), "d": zeros_arr(), "beta": zeros_arr(),
            "alfa": zeros_arr(), "contact_ratio": zeros_arr(), "delta": zeros_arr(),
            "bt": zeros_arr(), "f": zeros_arr(), "K_time": zeros_arr(), "Fm": zeros_arr()
        }

    def cut_backlash_results(self):
        """Remove o período transiente inicial da simulação."""
        for key in self.backlash_results:
            self.backlash_results[key] = self.backlash_results[key][self.n_cut:]
        if hasattr(self, 'backlash_total_force'):
            self.backlash_total_force = self.backlash_total_force[self.n_cut:, :]
        if hasattr(self, 'unb_force'):
            self.unbalance_force = self.unb_force.T[self.n_cut:, :]
    


    def generate_speed_ramp(self, ramp_fraction=0.0):
        """Gera uma rampa linear de velocidade angular."""
        t = np.asarray(self.time)
        omega_max = self.speed_driving_gear
        if ramp_fraction <= 0.0: return np.full_like(t, omega_max)  

        T_ramp = ramp_fraction * t[-1]
        speed_ramp = np.zeros_like(t)
        ramp_mask = t <= T_ramp
        speed_ramp[ramp_mask] = omega_max * (t[ramp_mask] / T_ramp)
        speed_ramp[~ramp_mask] = omega_max
        return speed_ramp


    def _get_or_create_stiffness_table(self, force_recalculate=False, square_varying_stiffness= False, kd=0, ks=0, n_poits = 200):
        """Carrega a tabela de rigidez 2D da memória ou gera uma nova usando o ROSS."""
        
        try:
            main_file = sys.modules['__main__'].__file__
            
            diretorio_execucao = os.path.dirname(os.path.abspath(main_file))
            
            nome_arquivo = os.path.splitext(os.path.basename(main_file))[0]
            
        except AttributeError:
            diretorio_execucao = os.getcwd() 
            nome_arquivo = "notebook_simulacao" 
            
        filename = f"k_table_{nome_arquivo}.npz"
        caminho_completo = os.path.join(diretorio_execucao, filename)

        if os.path.exists(caminho_completo) and not force_recalculate:
            print(f"Carregando Lookup Table de rigidez: '{caminho_completo}'...")
            with np.load(caminho_completo) as data:
                return data['theta_arr'], data['cr_arr'], data['K_table']
            
        print(f"Gerando nova grade de rigidez 2D em: {caminho_completo}...")
        
        pitch_angle = 2 * np.pi / self.gears[0].n_teeth
        
        theta_arr = np.linspace(0.0, pitch_angle, n_poits) 
        cr_arr = np.linspace(0.8, 2.5, n_poits) 
        K_table = np.zeros((len(theta_arr), len(cr_arr)))

        cr_original = self.multirotor.mesh.contact_ratio


        if square_varying_stiffness:
            Tm = pitch_angle
            
            limites = (cr_arr - 1) * Tm
            
            for i, th in enumerate(tqdm(theta_arr, desc="Gerando K_table (Square Mode)")):
                fase = th % Tm
                
                K_table[i, :] = np.where(
                    fase < limites, 
                    kd, 
                    np.where(np.isclose(fase, limites), (kd + ks) / 2, ks)
                )

        else:
            for i, th in enumerate(tqdm(theta_arr, desc="Gerando K_table")):
                for j, cr_val in enumerate(cr_arr):
                    self.multirotor.mesh.contact_ratio = cr_val
                    K_table[i, j] = self.multirotor.mesh.get_variable_stiffness(angular_position=th)


        self.multirotor.mesh.contact_ratio = cr_original

        np.savez(caminho_completo, theta_arr=theta_arr, cr_arr=cr_arr, K_table=K_table)
        return theta_arr, cr_arr, K_table
    

    def compute_backlash_force(self, step, time_step, disp_resp, velc_resp, accl_resp, **kwargs):
        """Callback compatível com a API nativa do método de integração do ROSS."""
        
        if not hasattr(self, 'theta_arr'):
            self.theta_arr, self.cr_arr, self.K_table = self._get_or_create_stiffness_table()

        gear_nodes = np.array([e.n for e in self.gears], dtype=np.int64)
        number_of_dof = self.multirotor.number_dof
        ndof_total = self.multirotor.ndof
        
        R1, R2 = self.gears[0].base_radius, self.gears[1].base_radius
        Ra1, Ra2 = self.gears[0].radii_dict["addendum"], self.gears[1].radii_dict["addendum"]
        module, alfa0 = self.gears[0].module, self.gears[0].pr_angle
        d0 = (self.gears[0].pitch_diameter + self.gears[1].pitch_diameter) / 2
        orientation_angle = self.multirotor.orientation_angle
        nominal_cr = self.multirotor.mesh.contact_ratio
        
        helix_angle = self.multirotor.mesh.helix_angle
        
        wm = self.speed_driving_gear * self.gears[0].n_teeth
        error_step = self.error[step]
        error_dot_step = self.error_amp * wm * np.cos(wm * self.time[step]) 
        angular_pos = self.speed_driving_gear * self.time[step]

        backlash_force, logs = calculate_dynamic_backlash_force(
            disp_resp, velc_resp, gear_nodes, number_of_dof, ndof_total,
            d0, orientation_angle, R1, R2, alfa0, helix_angle, self.b0,
            error_step, angular_pos, self.compute_contact_ratio, nominal_cr,
            Ra1, Ra2, module, self.sigma, self.smooth_operator, # <-- ADICIONADO AQUI
            self.theta_arr, self.cr_arr, self.K_table,
            self.M_eq, self.mesh_damping_ratio, error_dot_step 
        )

        keys = ["x1", "y1", "x2", "y2", "t1", "t2", "d", "beta", "alfa", 
                "contact_ratio", "delta", "bt", "f", "K_time", "Fm"]
        for i, key in enumerate(keys):
            self.backlash_results[key][step] = logs[i]

        self.backlash_total_force[step, :] = backlash_force
        self.multirotor.contact_ratio = logs[9] 
        return backlash_force

    def internal_newmark(self, F_unb, gamma=0.5, beta=0.25, tol=1e-6, epsilon=1e-8,
                         y_init=None, ydot_init=None, y2dot_init=None, # <-- SWEEP
                         theta_array=None, error_array=None, error_dot_array=None): # <-- CINEMÁTICA
        """Integra o sistema global usando Newmark Adaptativo C/C++ customizado."""
        M = np.ascontiguousarray(self.multirotor.M())
        K_sys = np.ascontiguousarray(self.multirotor.K(self.speed_driving_gear))
        F_unb = np.ascontiguousarray(F_unb)
        
        C_base = np.asarray(self.multirotor.C(self.speed_driving_gear))
        G_mat = np.asarray(self.multirotor.G())
        C_sys = np.ascontiguousarray(C_base + G_mat * self.speed_driving_gear)

        

        n_steps = len(self.time)
        wm = self.speed_driving_gear * self.gears[0].n_teeth

        gear_nodes = np.array([e.n for e in self.gears], dtype=np.int64)
        number_of_dof, ndof_total = self.multirotor.number_dof, self.multirotor.ndof

        gear_nodes = np.array([e.n for e in self.gears], dtype=np.int64)
        number_of_dof, ndof_total = self.multirotor.number_dof, self.multirotor.ndof

        

        if y_init is None: y_init = np.zeros(ndof_total)
        if ydot_init is None: ydot_init = np.zeros(ndof_total)
        if y2dot_init is None: y2dot_init = np.zeros(ndof_total)

        active_dofs = np.concatenate([
            np.arange(node * self.multirotor.number_dof, (node + 1) * self.multirotor.number_dof)
            for node in gear_nodes
        ]).astype(np.int64)
        

        d0 = (self.gears[0].pitch_diameter + self.gears[1].pitch_diameter) / 2
        R1, R2 = self.gears[0].base_radius, self.gears[1].base_radius
        alfa0 = self.gears[0].pr_angle
        orientation_angle = self.multirotor.orientation_angle
        nominal_cr = self.multirotor.mesh.contact_ratio
        Ra1, Ra2 = self.gears[0].radii_dict["addendum"], self.gears[1].radii_dict["addendum"]
        module = self.gears[0].module
        
        helix_angle = self.multirotor.mesh.helix_angle

        if not hasattr(self, 'theta_arr'):
            self.theta_arr, self.cr_arr, self.K_table = self._get_or_create_stiffness_table()

        yout = np.zeros((n_steps, ndof_total))
        logs_matrix = np.zeros((n_steps, 15))
        force_matrix = np.zeros((n_steps, ndof_total))

        print(f"\nIniciando Newmark Adaptativo Interno para {n_steps} passos...")

        start_time_newmark = time.time()

        final_ydot, final_y2dot = newmark_solver_full(
            self.time, yout, logs_matrix, force_matrix, F_unb,
            M, C_sys, K_sys, active_dofs, gamma, beta, tol, epsilon,
            gear_nodes, number_of_dof, ndof_total, d0, 
            orientation_angle, R1, R2, alfa0, helix_angle, self.b0, 
            self.compute_contact_ratio, nominal_cr, Ra1, Ra2, 
            module, self.sigma, self.smooth_operator, self.theta_arr, self.cr_arr, self.K_table,
            self.M_eq, self.mesh_damping_ratio, start_time_newmark,
            y_init, ydot_init, y2dot_init,
            theta_array, error_array, error_dot_array
        )

        self.final_states = (yout[-1, :], final_ydot, final_y2dot)

        keys = ["x1", "y1", "x2", "y2", "t1", "t2", "d", "beta", "alfa",
                "contact_ratio", "delta", "bt", "f", "K_time", "Fm"]
        for idx, key in enumerate(keys): self.backlash_results[key] = logs_matrix[:, idx]
        self.backlash_total_force = force_matrix

        return yout
    
    def run_dynamic_backlash(self, 
                             unb_node,
                             unb_magnitude,
                             unb_phase,
                             integration_method="internal_newmark",
                             add_force=None,
                             sigma=1e4,             
                             smooth_operator=True,    
                             y_init=None, ydot_init=None, y2dot_init=None, # <-- SWEEP
                             **kwargs):
        """Função Principal Roteadora - Prepara Forças e Chama o Integrador Selecionado."""

        start_time_total = time.perf_counter() # <-- INICIA O CRONÔMETRO AQUI

        self.sigma = sigma
        self.smooth_operator = smooth_operator
        
        ramp_fraction = kwargs.get('ramp_fraction', 0.0)
        speed_array = self.generate_speed_ramp(ramp_fraction=ramp_fraction)

        theta_array = cumulative_trapezoid(speed_array, self.time, initial=0.0)
        
        z1 = self.gears[0].n_teeth
        error_array = self.error_amp * np.sin(z1 * theta_array)
        error_dot_array = self.error_amp * (z1 * speed_array) * np.cos(z1 * theta_array)

        self.unb_force, _, _, _ = self.multirotor.unbalance_force_over_time(
            unb_node, unb_magnitude, unb_phase, speed_array, self.time, return_all=True)
        
        F = self.unb_force.T
        if add_force is not None: F += add_force

        print(f"==================================================")
        print(f"Iniciando simulação com o integrador: '{integration_method.upper()}'")
        print(f"==================================================")

        if integration_method.lower() == "internal_newmark":
            gamma = kwargs.get('gamma', 0.5)
            beta = kwargs.get('beta', 0.25)
            tol = kwargs.get('tol', 1e-6)
            yout_disp = self.internal_newmark(
                F_unb=F, gamma=gamma, beta=beta, tol=tol,
                y_init=y_init, ydot_init=ydot_init, y2dot_init=y2dot_init, # <-- SWEEP
                theta_array=theta_array, error_array=error_array, error_dot_array=error_dot_array # <-- CINEMÁTICA
            )
            results = TimeResponseResults(rotor=self.multirotor, t=self.time, yout=yout_disp, xout=[])

        else:
            raise ValueError("O único método de integração disponível é internal_newmark.")
        self.cut_backlash_results()
        
        results.yout = results.yout[self.n_cut:, :]
        results.t = results.t[self.n_cut:]
        self.time = self.time[self.n_cut:]
        self.time_response = results

        self.exec_time_dynamic = time.perf_counter() - start_time_total
                                                       
        return results

    def run_linear_baseline(self, unb_node, unb_magnitude, unb_phase, add_force=None, 
                            sigma=1e4, smooth_operator=True, **kwargs): 
        
        """
        Executa a simulação linear em DOIS PASSOS (Pseudo-Acoplamento Iterativo):
        1. Roda com desbalanceamento puro para encontrar a órbita linear.
        2. Extrai a força exata do engrenamento para essa órbita via Numba.
        3. Roda novamente o ROSS aplicando o desbalanceamento + força de engrenamento.
        """
        import time
        import numpy as np
        from scipy.integrate import cumulative_trapezoid # <-- IMPORTAÇÃO NECESSÁRIA

        start_time_total = time.perf_counter() # <-- INICIA O CRONÔMETRO AQUI

        self.sigma = sigma
        self.smooth_operator = smooth_operator

        ramp_fraction = kwargs.get('ramp_fraction', 0.0)
        speed_array = self.generate_speed_ramp(ramp_fraction=ramp_fraction)

        theta_array = cumulative_trapezoid(speed_array, self.time, initial=0.0)
        
        z1 = self.gears[0].n_teeth
        error_array = self.error_amp * np.sin(z1 * theta_array)
        error_dot_array = self.error_amp * (z1 * speed_array) * np.cos(z1 * theta_array)

        unb_force, _, _, _ = self.multirotor.unbalance_force_over_time(
            unb_node, unb_magnitude, unb_phase, speed_array, self.time, return_all=True)
        
        F_unb = unb_force.T
        if add_force is not None: 
            F_unb += add_force

        print(f"==================================================")
        print(f"Iniciando simulação BASELINE LINEAR (2 Passos Iterativos)")
        print(f"==================================================")

        print("-> Passo 1/2: Simulação ROSS (Apenas Desbalanceamento)...")
        t1 = time.time()
        results_step1 = self.multirotor.run_time_response(
            speed=speed_array, F=F_unb, t=self.time, method="default", **kwargs
        )
        print(f"   Tempo decorrido Passo 1: {time.time()-t1:.2e} s")

        ndof_total = self.multirotor.ndof
        yout_raw_1 = np.ascontiguousarray(results_step1.yout)
        dt = self.time[1] - self.time[0]

        if yout_raw_1.shape[1] == 2 * ndof_total:
            yout_1 = np.ascontiguousarray(yout_raw_1[:, :ndof_total])
            ydot_1 = np.ascontiguousarray(yout_raw_1[:, ndof_total:])
        else:
            yout_1 = yout_raw_1
            ydot_1 = np.ascontiguousarray(np.gradient(yout_raw_1, dt, axis=0))

        gear_nodes = np.array([e.n for e in self.gears], dtype=np.int64)
        number_of_dof = self.multirotor.number_dof
        d0 = (self.gears[0].pitch_diameter + self.gears[1].pitch_diameter) / 2
        R1, R2 = self.gears[0].base_radius, self.gears[1].base_radius
        alfa0 = self.gears[0].pr_angle
        orientation_angle = self.multirotor.orientation_angle
        nominal_cr = self.multirotor.mesh.contact_ratio
        Ra1, Ra2 = self.gears[0].radii_dict["addendum"], self.gears[1].radii_dict["addendum"]
        module = self.gears[0].module
        helix_angle = self.multirotor.mesh.helix_angle

        if not hasattr(self, 'theta_arr'):
            self.theta_arr, self.cr_arr, self.K_table = self._get_or_create_stiffness_table()

        print("-> Calculando Força de Engrenamento Teórica via Numba...")
        start_time_extract = time.time() 

        logs_matrix_1, F_mesh_global = extract_backlash_logs_from_trajectory(
            yout_1, ydot_1, self.time, gear_nodes, number_of_dof, ndof_total,
            d0, orientation_angle, R1, R2, alfa0, helix_angle, self.b0, 
            self.compute_contact_ratio, nominal_cr, 
            Ra1, Ra2, module, self.sigma, self.smooth_operator, self.theta_arr, self.cr_arr, self.K_table,
            self.M_eq, self.mesh_damping_ratio, start_time_extract,
            theta_array, error_array, error_dot_array # <-- OS VETORES AGORA EXISTEM E SÃO PASSADOS
        )

        F_total = F_unb + F_mesh_global

        print("-> Passo 2/2: Simulação ROSS (Desbalanceamento + Malha)...")
        t2 = time.time()
        results_final = self.multirotor.run_time_response(
            speed=speed_array, F=F_total, t=self.time, method="default", **kwargs
        )
        print(f"   Tempo decorrido Passo 2: {time.time()-t2:.2e} s")

        self.linear_backlash_results = {}
        keys = ["x1", "y1", "x2", "y2", "t1", "t2", "d", "beta", "alfa",
                "contact_ratio", "delta", "bt", "f", "K_time", "Fm"]
        
        for idx, key in enumerate(keys):
            self.linear_backlash_results[key] = logs_matrix_1[self.n_cut:, idx]

        results_final.yout = results_final.yout[self.n_cut:, :ndof_total] 
        results_final.t = results_final.t[self.n_cut:]
        
        self.linear_time_response = results_final
        self.exec_time_linear = time.perf_counter() - start_time_total
        
        print("Baseline linear recalculado e acoplado com sucesso!")
        return results_final


    def save_results(self, unb_node, unb_magnitude, unb_phase, integration_method, output_dir="resultados_backlash", compress_csv=False, add_force=None):
        """Salva metadados detalhados, séries temporais, matriz de forças externas e o objeto binário no diretório de execução."""

        try:
            main_file = sys.modules['__main__'].__file__
            diretorio_execucao = os.path.dirname(os.path.abspath(main_file))
            nome_arquivo_teste = os.path.splitext(os.path.basename(main_file))[0]
        except AttributeError:
            diretorio_execucao = os.getcwd()
            nome_arquivo_teste = "jupyter_notebook"

        if os.path.isabs(output_dir):
            base_dir = output_dir
        else:
            base_dir = os.path.join(diretorio_execucao, output_dir)
            
        if not os.path.exists(base_dir): 
            os.makedirs(base_dir)

        timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%Hh-%Mm-%Ss")
        run_folder = os.path.join(base_dir, f"simulacao_{timestamp}")
        os.makedirs(run_folder)

        def formatar_tempo(segundos_totais):
            horas, resto = divmod(segundos_totais, 3600)
            minutos, segundos = divmod(resto, 60)
            return f"{int(horas):02d}:{int(minutos):02d}:{segundos:06.3f}"

        report_path = os.path.join(run_folder, "relatorio_simulacao.txt")
        dt = self.time[1] - self.time[0]
        
        with open(report_path, "w", encoding="utf-8") as f:
            f.write("====================================================\n")
            f.write("       RELATÓRIO DE SIMULAÇÃO - ROTODINÂMICA        \n")
            f.write("====================================================\n")
            f.write(f"Data e Hora           : {datetime.datetime.now().strftime('%d/%m/%Y %H:%M:%S')}\n")
            f.write(f"Diretório de Origem   : {base_dir}\n") 
            f.write(f"Script de Execução    : {nome_arquivo_teste}.py\n")
            f.write("----------------------------------------------------\n")
            
            f.write("TEMPOS DE PROCESSAMENTO (HH:MM:SS)\n")
            if hasattr(self, 'exec_time_linear'):
                f.write(f"Baseline Linear       : {formatar_tempo(self.exec_time_linear)}\n")
            
            if hasattr(self, 'exec_time_dynamic'):
                f.write(f"Dinâmica Não-Linear   : {formatar_tempo(self.exec_time_dynamic)}\n")
            f.write("----------------------------------------------------\n")

            f.write("PARÂMETROS DE INTEGRAÇÃO E SOLVER\n")
            f.write(f"Método de Integração  : {integration_method.upper()}\n")
            f.write(f"Passo de Tempo (dt)   : {dt:.2e} s\n")
            f.write(f"Frequência de Amostr. : {1/dt:.2f} Hz\n")
            f.write(f"Ciclos Simulados      : {getattr(self, 'n_cicles', 'N/A')} (Totais) | {getattr(self, 'cut_cicles', 'N/A')} (Descartados)\n")
            f.write("----------------------------------------------------\n")
            
            f.write("CONFIGURAÇÕES DE NÃO-LINEARIDADE (CONTATO E FOLGA)\n")
            f.write(f"Smooth Operator       : {getattr(self, 'smooth_operator', 'Não Definido')}\n")
            f.write(f"Sigma (Suavização)    : {getattr(self, 'sigma', 'Não Definido')}\n")
            f.write(f"Cálculo de CR Dinâmico: {getattr(self, 'compute_contact_ratio', 'Não Definido')}\n")
            f.write(f"Usa Rigidez do ROSS   : {'Sim' if self.multirotor.update_mesh_stiffness else 'Não (Desativada)'}\n")
            f.write("----------------------------------------------------\n")
            
            f.write("GEOMETRIA DO ENGRENAMENTO E FÍSICA\n")
            f.write(f"Velocidade (Pinhão)   : {self.speed_driving_gear:.2f} rad/s\n")
            f.write(f"Frequência de Malha   : {(self.speed_driving_gear * self.gears[0].n_teeth)/(2*np.pi):.2f} Hz\n")
            f.write(f"Dentes (Z1 / Z2)      : {self.gears[0].n_teeth} / {self.gears[1].n_teeth}\n")
            f.write(f"Módulo                : {self.gears[0].module} m\n")
            f.write(f"Ângulo de Pressão (a0): {np.degrees(self.gears[0].pr_angle):.2f} °\n")
            f.write(f"Ângulo de Hélice      : {np.degrees(self.multirotor.mesh.helix_angle):.2f} °\n")
            f.write(f"Backlash Inicial (b0) : {self.b0:.4e} m\n")
            f.write(f"Erro Estático (Amp)   : {self.error_amp:.4e} m\n")
            f.write(f"Razão de Amortecimento: {self.mesh_damping_ratio}\n")
            f.write(f"Massa Equivalente (M) : {self.M_eq:.4f} kg\n")
            f.write("----------------------------------------------------\n")
            
            f.write("PARÂMETROS DE DESBALANCEAMENTO\n")
            f.write(f"Nós Excitados         : {unb_node}\n")
            f.write(f"Magnitude (kg.m)      : {unb_magnitude}\n")
            f.write(f"Fase (rad)            : {unb_phase}\n")
            f.write("----------------------------------------------------\n")
            
            f.write(f"Possui Baseline Linear? : {'Sim' if hasattr(self, 'linear_backlash_results') else 'Não'}\n")
            f.write(f"Vetor Add Force Extra?  : {'Sim' if add_force is not None else 'Não'}\n")
            f.write("====================================================\n")

        if hasattr(self, 'backlash_results') and len(self.backlash_results["x1"]) > 0:
            csv_nl_path = os.path.join(run_folder, "historico_temporal_nao_linear.csv")
            if compress_csv: csv_nl_path += ".gz"
            
            df_nl = pd.DataFrame({"Tempo_s": self.time})
            for key, array_data in self.backlash_results.items():
                df_nl[key] = array_data
                
            df_nl.to_csv(csv_nl_path, index=False, compression='gzip' if compress_csv else None)

        if hasattr(self, 'linear_backlash_results'):
            csv_lin_path = os.path.join(run_folder, "historico_temporal_linear.csv")
            if compress_csv: csv_lin_path += ".gz"
            
            df_lin = pd.DataFrame({"Tempo_s": self.linear_time_response.t})
            for key, array_data in self.linear_backlash_results.items():
                df_lin[key] = array_data
                
            df_lin.to_csv(csv_lin_path, index=False, compression='gzip' if compress_csv else None)

        if add_force is not None:
            csv_force_path = os.path.join(run_folder, "forcas_externas_add_force.csv")
            if compress_csv: csv_force_path += ".gz"
            
            add_force_arr = np.asarray(add_force)
            
            if add_force_arr.ndim == 1:
                col_names = [f"DOF_{i}" for i in range(add_force_arr.shape[0])]
                df_forcas = pd.DataFrame([add_force_arr], columns=col_names)
            
            else:
                col_names = [f"DOF_{i}" for i in range(add_force_arr.shape[1])]
                df_forcas = pd.DataFrame(add_force_arr, columns=col_names)
                
                if add_force_arr.shape[0] == len(self.time):
                    df_forcas.insert(0, "Tempo_s", self.time)
                    
            df_forcas.to_csv(csv_force_path, index=False, compression='gzip' if compress_csv else None)

        nome_arquivo_pickle = f"modelo_completo_{nome_arquivo_teste}.pkl"
        pickle_path = os.path.join(run_folder, nome_arquivo_pickle)
        try:
            with open(pickle_path, 'wb') as f:
                pickle.dump(self, f, protocol=pickle.HIGHEST_PROTOCOL)
            print(f"Objeto binário '{nome_arquivo_pickle}' salvo com sucesso!")
        except Exception as e:
            print(f"AVISO: Ocorreu um erro ao salvar o objeto binário: {e}")

        print(f"\nResultados salvos com sucesso na pasta:\n'{run_folder}'")
        return run_folder
    
    @staticmethod
    def load_model(filepath):
        """
        Carrega um modelo de simulação salvo anteriormente a partir de um arquivo .pkl.

        from seu_arquivo_onde_esta_a_classe import Backlash

        modelo_recuperado = Backlash.load_model("resultados_backlash/simulacao_2026-03-17_15h-30m-45s/modelo_completo.pkl")

        import plotly.express as px
        fig = px.line(x=modelo_recuperado.time, y=modelo_recuperado.backlash_results["Fm"])
        fig.show()

        modelo_recuperado.multirotor.plot_rotor()
        
        Exemplo de uso:
        >>> from seu_script import Backlash
        >>> meu_modelo = Backlash.load_model("caminho/para/modelo_completo.pkl")
        """        
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"Arquivo não encontrado: {filepath}")
            
        print(f"Carregando modelo do arquivo '{filepath}'...")
        with open(filepath, 'rb') as f:
            loaded_model = pickle.load(f)
            
        print("Modelo carregado com sucesso!")
        return loaded_model

    def plot_dashboard(self, freq_unit="Hz", decimation=1, save_path=None, is_linear=False, dft_y_scale="log", time_range=None, freq_range=None):
        """
        Gera um painel interativo consolidado com disposição tempo/frequência para as variáveis.
        O HTML será salvo na mesma pasta do script se um caminho absoluto não for fornecido.
        """
        if save_path:
            try:
                main_file = sys.modules['__main__'].__file__
                diretorio_execucao = os.path.dirname(os.path.abspath(main_file))
            except AttributeError:
                diretorio_execucao = os.getcwd()
                
            if not os.path.isabs(save_path):
                save_path = os.path.join(diretorio_execucao, save_path)

        if is_linear:
            if not hasattr(self, 'linear_backlash_results'):
                print("ERRO: Resultados lineares não encontrados. Rode 'run_linear_baseline()' primeiro.")
                return
            dados = self.linear_backlash_results
            tempo = self.linear_time_response.t
            titulo_base = "Baseline Linear"
        else:
            dados = self.backlash_results
            tempo = self.time
            titulo_base = "Resultados"

        rpm_vel = self.speed_driving_gear * (30 / np.pi)
        titulo = f"Dashboard de {titulo_base} - Dinâmica de Engrenamento ({rpm_vel:.0f} RPM)"

        freq_delta, amp_delta = compute_dfft(dados["delta"], tempo, freq_unit=freq_unit)
        freq_bt, amp_bt       = compute_dfft(dados["bt"], tempo, freq_unit=freq_unit)
        freq_fm, amp_fm       = compute_dfft(dados["Fm"], tempo, freq_unit=freq_unit)
        freq_km, amp_km       = compute_dfft(dados["K_time"], tempo, freq_unit=freq_unit)
        freq_d, amp_d         = compute_dfft(dados["d"], tempo, freq_unit=freq_unit)

        if time_range is not None:
            t_min, t_max = time_range
            mask_t = (tempo >= t_min) & (tempo <= t_max)
            
            tempo_cortado = tempo[mask_t]
            dados_cortados = {k: np.array(v)[mask_t] for k, v in dados.items()}
            
            if len(tempo_cortado) == 0:
                print(f"ERRO: O range de tempo {time_range} não contém dados. Verifique os limites.")
                return
        else:
            tempo_cortado = tempo
            dados_cortados = dados

        d0 = (self.gears[0].pitch_diameter + self.gears[1].pitch_diameter) / 2
        alfa0_deg = np.degrees(self.gears[0].pr_angle)
        nominal_cr = self.multirotor.mesh.contact_ratio

        t_plot = tempo_cortado[::decimation]
        res = {k: v[::decimation] for k, v in dados_cortados.items()}
        
        d0_vec = np.full_like(t_plot, d0)
        alfa0_vec = np.full_like(t_plot, alfa0_deg)
        cr0_vec = np.full_like(t_plot, nominal_cr)

        fig = make_subplots(
            rows=5, cols=2,
            subplot_titles=(
                "Erro de Transmissão (δ) e Backlash (+bt)", "Espectro DFT (δ e +bt)",
                "Força Dinâmica (Fm)", "Espectro DFT (Fm)",
                "Rigidez de Engrenamento (K_m)", "Espectro DFT (K_m)",
                "Distância entre Centros (d)", "Espectro DFT (d)",
                "Ângulo de Pressão Dinâmico (α)", "Razão de Contato (CR)" # Ambos no domínio do tempo agora
            ),
            vertical_spacing=0.06, horizontal_spacing=0.08
        )

        fig.add_trace(go.Scattergl(x=t_plot, y=res["delta"], name="δ(t)", line=dict(color='blue')), row=1, col=1)
        fig.add_trace(go.Scattergl(x=t_plot, y=res["bt"], name="+bt", line=dict(color='red', dash='dash')), row=1, col=1)
        
        fig.add_trace(go.Scattergl(x=t_plot, y=res["Fm"], name="Fm(t)", line=dict(color='purple')), row=2, col=1)
        fig.add_trace(go.Scattergl(x=t_plot, y=res["K_time"], name="K_m(t)", line=dict(color='teal')), row=3, col=1)
        
        fig.add_trace(go.Scattergl(x=t_plot, y=res["d"], name="d(t)", line=dict(color='green')), row=4, col=1)
        fig.add_trace(go.Scattergl(x=t_plot, y=d0_vec, name="d0 (Nominal)", line=dict(color='black', dash='dot')), row=4, col=1)
        
        fig.add_trace(go.Scattergl(x=t_plot, y=np.degrees(res["alfa"]), name="α(t)", line=dict(color='darkorange')), row=5, col=1)
        fig.add_trace(go.Scattergl(x=t_plot, y=alfa0_vec, name="α0 (Nominal)", line=dict(color='black', dash='dot')), row=5, col=1)

        fig.add_trace(go.Scattergl(x=t_plot, y=res["contact_ratio"], name="CR(t)", line=dict(color='olive')), row=5, col=2)
        fig.add_trace(go.Scattergl(x=t_plot, y=cr0_vec, name="CR0 (Nominal)", line=dict(color='black', dash='dot')), row=5, col=2)

        if freq_range is not None:
            f_min, f_max = freq_range
            m_delta = (freq_delta >= f_min) & (freq_delta <= f_max); freq_delta, amp_delta = freq_delta[m_delta], amp_delta[m_delta]
            m_bt = (freq_bt >= f_min) & (freq_bt <= f_max); freq_bt, amp_bt = freq_bt[m_bt], amp_bt[m_bt]
            m_fm = (freq_fm >= f_min) & (freq_fm <= f_max); freq_fm, amp_fm = freq_fm[m_fm], amp_fm[m_fm]
            m_km = (freq_km >= f_min) & (freq_km <= f_max); freq_km, amp_km = freq_km[m_km], amp_km[m_km]
            m_d = (freq_d >= f_min) & (freq_d <= f_max); freq_d, amp_d = freq_d[m_d], amp_d[m_d]

        fig.add_trace(go.Scattergl(x=freq_delta, y=amp_delta, name="DFT(δ)", line=dict(color='blue')), row=1, col=2)
        fig.add_trace(go.Scattergl(x=freq_bt, y=amp_bt, name="DFT(+bt)", line=dict(color='red', dash='dash')), row=1, col=2)
        fig.add_trace(go.Scattergl(x=freq_fm, y=amp_fm, name="DFT(Fm)", line=dict(color='purple')), row=2, col=2)
        fig.add_trace(go.Scattergl(x=freq_km, y=amp_km, name="DFT(K_m)", line=dict(color='teal')), row=3, col=2)
        fig.add_trace(go.Scattergl(x=freq_d, y=amp_d, name="DFT(d)", line=dict(color='green')), row=4, col=2)

        fig.update_layout(title_text=titulo, height=1500, width=1400, template="plotly_white", hovermode="x unified", showlegend=False)
        
        for r in range(1, 6):
            fig.update_xaxes(title_text="Tempo (s)", row=r, col=1)
        
        for r in range(1, 5):
            fig.update_xaxes(title_text=f"Frequência ({freq_unit})", row=r, col=2)
        fig.update_xaxes(title_text="Tempo (s)", row=5, col=2)
        
        fig.update_yaxes(title_text="Desloc. (m)", row=1, col=1);   fig.update_yaxes(title_text="Amp. (m)", row=1, col=2)
        fig.update_yaxes(title_text="Força (N)", row=2, col=1);     fig.update_yaxes(title_text="Amp. (N)", row=2, col=2)
        fig.update_yaxes(title_text="Rigidez (N/m)", row=3, col=1); fig.update_yaxes(title_text="Amp. (N/m)", row=3, col=2)
        fig.update_yaxes(title_text="Distância (m)", row=4, col=1); fig.update_yaxes(title_text="Amp. (m)", row=4, col=2)
        fig.update_yaxes(title_text="Ângulo (deg)", row=5, col=1);  fig.update_yaxes(title_text="Razão", row=5, col=2)

        if dft_y_scale.lower() == "log":
            for r in range(1, 5):
                fig.update_yaxes(type="log", row=r, col=2)
        elif dft_y_scale.lower() == "linear":
            for r in range(1, 5):
                fig.update_yaxes(type="linear", row=r, col=2)

        leg_st = dict(xref="x domain", yref="y domain", x=0.98, y=0.98, xanchor="right", yanchor="top", 
                      showarrow=False, align="left", bgcolor="rgba(255,255,255,0.8)", bordercolor="lightgray", borderwidth=1)
        
        fig.add_annotation(text="<span style='color:blue'>— δ(t)</span><br><span style='color:red'>-- +bt</span>", row=1, col=1, **leg_st)
        fig.add_annotation(text="<span style='color:purple'>— Fm(t)</span>", row=2, col=1, **leg_st)
        fig.add_annotation(text="<span style='color:teal'>— K_m(t)</span>", row=3, col=1, **leg_st)
        fig.add_annotation(text="<span style='color:green'>— d(t)</span><br><span style='color:black'>-- d0 (Nominal)</span>", row=4, col=1, **leg_st)
        fig.add_annotation(text="<span style='color:darkorange'>— α(t)</span><br><span style='color:black'>-- α0 (Nominal)</span>", row=5, col=1, **leg_st)
        
        fig.add_annotation(text="<span style='color:blue'>— DFT(δ)</span><br><span style='color:red'>-- DFT(+bt)</span>", row=1, col=2, **leg_st)
        fig.add_annotation(text="<span style='color:purple'>— DFT(Fm)</span>", row=2, col=2, **leg_st)
        fig.add_annotation(text="<span style='color:teal'>— DFT(K_m)</span>", row=3, col=2, **leg_st)
        fig.add_annotation(text="<span style='color:green'>— DFT(d)</span>", row=4, col=2, **leg_st)
        fig.add_annotation(text="<span style='color:olive'>— CR(t)</span><br><span style='color:black'>-- CR0 (Nominal)</span>", row=5, col=2, **leg_st)

        if save_path:
            fig.write_html(save_path, include_plotlyjs="cdn")
            print(f"Dashboard leve HTML salvo em: {save_path}")
        else:
            fig.show()

    def save_and_plot_linear_baseline(self, csv_filename="baseline_linear_dados.csv", plot_filename="baseline_linear_plot.html"):
        """
        Guarda os dados do baseline linear num ficheiro CSV e gera um painel 
        interativo (Plotly HTML) com os gráficos das principais variáveis.
        """

        if not hasattr(self, 'linear_backlash_results') or not self.linear_backlash_results:
            raise ValueError("Nenhum resultado linear encontrado. Execute run_linear_baseline() primeiro.")

        print(f"\nA guardar os dados em '{csv_filename}'...")

        time_arr = self.linear_time_response.t
        data = {"Time": time_arr}
        for key, value_array in self.linear_backlash_results.items():
            data[key] = value_array

        df = pd.DataFrame(data)
        df.to_csv(csv_filename, index=False)
        print("Dados CSV guardados com sucesso!")

        print(f"A gerar os gráficos em '{plot_filename}'...")
        
        fig = make_subplots(
            rows=3, cols=1, 
            shared_xaxes=True,
            vertical_spacing=0.08,
            subplot_titles=(
                "Força Dinâmica de Engrenamento (Fm)", 
                "Erro de Transmissão vs. Folga (Delta e limites bt)", 
                "Rigidez Variável no Tempo (K_time)"
            )
        )

        res = self.linear_backlash_results

        fig.add_trace(go.Scatter(x=time_arr, y=res["Fm"], name="Fm", line=dict(color='blue')), row=1, col=1)

        fig.add_trace(go.Scatter(x=time_arr, y=res["delta"], name="Delta", line=dict(color='purple')), row=2, col=1)
        fig.add_trace(go.Scatter(x=time_arr, y=res["bt"], name="+bt (Limite Folga)", line=dict(color='red', dash='dash')), row=2, col=1)
        fig.add_trace(go.Scatter(x=time_arr, y=-res["bt"], name="-bt (Limite Folga)", line=dict(color='red', dash='dash')), row=2, col=1)

        fig.add_trace(go.Scatter(x=time_arr, y=res["K_time"], name="Rigidez (K)", line=dict(color='green')), row=3, col=1)

        fig.update_layout(
            title_text="Dashboard - Resultados do Baseline Linear",
            height=900,
            showlegend=True,
            template="plotly_white"
        )
        
        fig.update_xaxes(title_text="Tempo (s)", row=3, col=1)
        fig.update_yaxes(title_text="Força (N)", row=1, col=1)
        fig.update_yaxes(title_text="Deslocamento (m)", row=2, col=1)
        fig.update_yaxes(title_text="Rigidez (N/m)", row=3, col=1)

        fig.write_html(plot_filename)
        print("Gráficos guardados com sucesso!")
        

    def plot_poincare_map(self, is_linear=False, save_dir=None, plot_filename="mapa_poincare.html", csv_filename="dados_poincare.csv", discard_periods=None, phase_offset_ratio=0.25, use_spline=True):
        """
        Gera o Mapa de Poincaré sobreposto ao Espaço de Fase Contínuo,
        dividindo em dois arquivos HTML (Pinhão+DTE e Coroa), e exportando para CSV. 
        Cores otimizadas para daltonismo (Paleta Okabe-Ito).
        
        Novo: Inclui phase_offset_ratio para sincronizar a fase com papers de referência
        e cálculo de derivadas analíticas precisas via CubicSpline.
        """

        if is_linear:
            if not hasattr(self, 'linear_backlash_results'):
                print("ERRO: Resultados lineares não encontrados. Rode 'run_linear_baseline()' primeiro.")
                return
            dados = self.linear_backlash_results
            yout_disp = self.linear_time_response.yout
            tempo = self.linear_time_response.t
            prefixo = "linear_"
        else:
            if not hasattr(self, 'backlash_results'):
                print("ERRO: Resultados não-lineares não encontrados. Rode a simulação primeiro.")
                return
            dados = self.backlash_results
            yout_disp = self.time_response.yout
            tempo = self.time
            prefixo = "nao_linear_"

        try:
            main_file = sys.modules['__main__'].__file__
            diretorio_execucao = os.path.dirname(os.path.abspath(main_file))
        except AttributeError:
            diretorio_execucao = os.getcwd()

        if save_dir is None:
            save_dir = diretorio_execucao
        elif not os.path.isabs(save_dir):
            save_dir = os.path.join(diretorio_execucao, save_dir)
            
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)

        dt = tempo[1] - tempo[0]
        delta = dados["delta"]
        delta_dot = np.gradient(delta, dt)
        yout_vel = np.gradient(yout_disp, dt, axis=0)

        wm = self.speed_driving_gear * self.gears[0].n_teeth
        Tm = (2.0 * np.pi) / wm  
        
        t_max = tempo[-1]
        n_periods = int(t_max / Tm)
        
        if discard_periods is None:
            discard_periods = int(n_periods * 0.8) 
            print(f"Descarte automático: ignorando os primeiros {discard_periods} de {n_periods} ciclos.")
            
        if discard_periods >= n_periods:
            print(f"AVISO: 'discard_periods' ({discard_periods}) é maior que o total de ciclos ({n_periods}).")
            discard_periods = int(n_periods * 0.5)
            print(f"Ajustando descarte para {discard_periods} ciclos.")
            
        t_discard_start = discard_periods * Tm
        idx_steady = np.searchsorted(tempo, t_discard_start)
        
        t_offset = phase_offset_ratio * Tm
        t_poincare = np.arange(discard_periods, n_periods) * Tm + t_offset
        t_poincare = t_poincare[t_poincare < tempo[-1]]

        num_dof = self.multirotor.number_dof
        idx1, idx2 = num_dof * self.gears[0].n, num_dof * self.gears[1].n

        if use_spline:
            cs_delta = CubicSpline(tempo, delta)
            p_delta = cs_delta(t_poincare)
            p_delta_dot = cs_delta(t_poincare, nu=1) # nu=1 tira a derivada analítica do spline
            
            p_disp_g1, p_vel_g1 = [], []
            p_disp_g2, p_vel_g2 = [], []
            
            for i in range(6):
                cs_g1 = CubicSpline(tempo, yout_disp[:, idx1 + i])
                p_disp_g1.append(cs_g1(t_poincare))
                p_vel_g1.append(cs_g1(t_poincare, nu=1))
                
                cs_g2 = CubicSpline(tempo, yout_disp[:, idx2 + i])
                p_disp_g2.append(cs_g2(t_poincare))
                p_vel_g2.append(cs_g2(t_poincare, nu=1))
        else:
            p_delta = np.interp(t_poincare, tempo, delta)
            p_delta_dot = np.interp(t_poincare, tempo, delta_dot)
            
            p_disp_g1 = [np.interp(t_poincare, tempo, yout_disp[:, idx1 + i]) for i in range(6)]
            p_vel_g1  = [np.interp(t_poincare, tempo, yout_vel[:, idx1 + i]) for i in range(6)]
            p_disp_g2 = [np.interp(t_poincare, tempo, yout_disp[:, idx2 + i]) for i in range(6)]
            p_vel_g2  = [np.interp(t_poincare, tempo, yout_vel[:, idx2 + i]) for i in range(6)]

        c_delta = delta[idx_steady:]
        c_delta_dot = delta_dot[idx_steady:]
        c_disp_g1 = [yout_disp[idx_steady:, idx1 + i] for i in range(6)]
        c_vel_g1  = [yout_vel[idx_steady:, idx1 + i] for i in range(6)]
        c_disp_g2 = [yout_disp[idx_steady:, idx2 + i] for i in range(6)]
        c_vel_g2  = [yout_vel[idx_steady:, idx2 + i] for i in range(6)]

        caminho_csv = os.path.join(save_dir, prefixo + csv_filename)
        print(f"Salvando CSV do Mapa de Poincaré em '{caminho_csv}'...")
        dados_exportacao = {
            "Tempo_s": t_poincare,
            "Delta_um": p_delta * 1e6, "dDelta_dt_mm_s": p_delta_dot * 1000,
            "Pinao_x_um": p_disp_g1[0] * 1e6, "Pinao_dx_dt_mm_s": p_vel_g1[0] * 1000,
            "Pinao_y_um": p_disp_g1[1] * 1e6, "Pinao_dy_dt_mm_s": p_vel_g1[1] * 1000,
            "Pinao_z_um": p_disp_g1[2] * 1e6, "Pinao_dz_dt_mm_s": p_vel_g1[2] * 1000,
            "Pinao_rx_mrad": p_disp_g1[3] * 1e3, "Pinao_drx_dt_mrad_s": p_vel_g1[3] * 1000,
            "Pinao_ry_mrad": p_disp_g1[4] * 1e3, "Pinao_dry_dt_mrad_s": p_vel_g1[4] * 1000,
            "Pinao_tz_mrad": p_disp_g1[5] * 1e3, "Pinao_dtz_dt_mrad_s": p_vel_g1[5] * 1000,
            "Coroa_x_um": p_disp_g2[0] * 1e6, "Coroa_dx_dt_mm_s": p_vel_g2[0] * 1000,
            "Coroa_y_um": p_disp_g2[1] * 1e6, "Coroa_dy_dt_mm_s": p_vel_g2[1] * 1000,
            "Coroa_z_um": p_disp_g2[2] * 1e6, "Coroa_dz_dt_mm_s": p_vel_g2[2] * 1000,
            "Coroa_rx_mrad": p_disp_g2[3] * 1e3, "Coroa_drx_dt_mrad_s": p_vel_g2[3] * 1000,
            "Coroa_ry_mrad": p_disp_g2[4] * 1e3, "Coroa_dry_dt_mrad_s": p_vel_g2[4] * 1000,
            "Coroa_tz_mrad": p_disp_g2[5] * 1e3, "Coroa_dtz_dt_mrad_s": p_vel_g2[5] * 1000,
        }
        pd.DataFrame(dados_exportacao).to_csv(caminho_csv, index=False, encoding='utf-8')

        cor_espaco_fase = '#56B4E9' # Azul Claro Celeste
        cor_poincare = '#D55E00'    # Vermelhão/Laranja
        
        line_style = dict(color=cor_espaco_fase, width=1.5)
        marker_style = dict(size=6, color=cor_poincare, symbol='circle', line=dict(color='white', width=0.5), opacity=0.9)

        caminho_plot_pinhao = os.path.join(save_dir, prefixo + "pinhao_" + plot_filename)
        fig1 = make_subplots(
            rows=3, cols=3,
            specs=[[None, {"type": "scatter"}, None], [{"type": "scatter"}]*3, [{"type": "scatter"}]*3],
            subplot_titles=["DTE: δ vs dδ/dt", "Pinhão: x", "Pinhão: y", "Pinhão: z", "Pinhão: rx", "Pinhão: ry", "Pinhão: θz"],
            vertical_spacing=0.12
        )
        
        fig1.add_trace(go.Scatter(x=c_delta * 1e6, y=c_delta_dot * 1000, mode='lines', line=line_style, opacity=0.5, name="Espaço Fase"), row=1, col=2)
        fig1.add_trace(go.Scatter(x=dados_exportacao["Delta_um"], y=dados_exportacao["dDelta_dt_mm_s"], mode='markers', marker=marker_style, name="Poincaré"), row=1, col=2)
        
        ch_p_d = ["Pinao_x_um", "Pinao_y_um", "Pinao_z_um", "Pinao_rx_mrad", "Pinao_ry_mrad", "Pinao_tz_mrad"]
        ch_p_v = ["Pinao_dx_dt_mm_s", "Pinao_dy_dt_mm_s", "Pinao_dz_dt_mm_s", "Pinao_drx_dt_mrad_s", "Pinao_dry_dt_mrad_s", "Pinao_dtz_dt_mrad_s"]
        
        for i in range(3): 
            fig1.add_trace(go.Scatter(x=c_disp_g1[i] * 1e6, y=c_vel_g1[i] * 1000, mode='lines', line=line_style, opacity=0.5, showlegend=False), row=2, col=i+1)
            fig1.add_trace(go.Scatter(x=dados_exportacao[ch_p_d[i]], y=dados_exportacao[ch_p_v[i]], mode='markers', marker=marker_style, showlegend=False), row=2, col=i+1)
            
        for i in range(3, 6): 
            fig1.add_trace(go.Scatter(x=c_disp_g1[i] * 1e3, y=c_vel_g1[i] * 1000, mode='lines', line=line_style, opacity=0.5, showlegend=False), row=3, col=i-2)
            fig1.add_trace(go.Scatter(x=dados_exportacao[ch_p_d[i]], y=dados_exportacao[ch_p_v[i]], mode='markers', marker=marker_style, showlegend=False), row=3, col=i-2)

        fig1.update_layout(title_text=f"Poincaré + Espaço de Fase - Pinhão e DTE ({prefixo[:-1].upper()})", height=900, width=1200, template="plotly_white")
        
        fig1.update_xaxes(title_text="Deslocamento (μm)", row=1, col=2); fig1.update_yaxes(title_text="Velocidade (mm/s)", row=1, col=2)
        for c in range(1, 4):
            fig1.update_xaxes(title_text="Deslocamento (μm)", row=2, col=c); fig1.update_yaxes(title_text="Velocidade (mm/s)", row=2, col=c)
            fig1.update_xaxes(title_text="Ângulo (mrad)", row=3, col=c); fig1.update_yaxes(title_text="Vel. Angular (mrad/s)", row=3, col=c)

        fig1.write_html(caminho_plot_pinhao, include_plotlyjs="cdn")

        caminho_plot_coroa = os.path.join(save_dir, prefixo + "coroa_" + plot_filename)
        fig2 = make_subplots(
            rows=2, cols=3,
            specs=[[{"type": "scatter"}]*3, [{"type": "scatter"}]*3],
            subplot_titles=["Coroa: x", "Coroa: y", "Coroa: z", "Coroa: rx", "Coroa: ry", "Coroa: θz"],
            vertical_spacing=0.15
        )
        
        ch_c_d = ["Coroa_x_um", "Coroa_y_um", "Coroa_z_um", "Coroa_rx_mrad", "Coroa_ry_mrad", "Coroa_tz_mrad"]
        ch_c_v = ["Coroa_dx_dt_mm_s", "Coroa_dy_dt_mm_s", "Coroa_dz_dt_mm_s", "Coroa_drx_dt_mrad_s", "Coroa_dry_dt_mrad_s", "Coroa_dtz_dt_mrad_s"]
        
        for i in range(3): 
            fig2.add_trace(go.Scatter(x=c_disp_g2[i] * 1e6, y=c_vel_g2[i] * 1000, mode='lines', line=line_style, opacity=0.5, showlegend=False), row=1, col=i+1)
            fig2.add_trace(go.Scatter(x=dados_exportacao[ch_c_d[i]], y=dados_exportacao[ch_c_v[i]], mode='markers', marker=marker_style, showlegend=False), row=1, col=i+1)
            
        for i in range(3, 6): 
            fig2.add_trace(go.Scatter(x=c_disp_g2[i] * 1e3, y=c_vel_g2[i] * 1000, mode='lines', line=line_style, opacity=0.5, showlegend=False), row=2, col=i-2)
            fig2.add_trace(go.Scatter(x=dados_exportacao[ch_c_d[i]], y=dados_exportacao[ch_c_v[i]], mode='markers', marker=marker_style, showlegend=False), row=2, col=i-2)

        fig2.update_layout(title_text=f"Poincaré + Espaço de Fase - Coroa ({prefixo[:-1].upper()})", height=700, width=1200, template="plotly_white")
        
        for c in range(1, 4):
            fig2.update_xaxes(title_text="Deslocamento (μm)", row=1, col=c); fig2.update_yaxes(title_text="Velocidade (mm/s)", row=1, col=c)
            fig2.update_xaxes(title_text="Ângulo (mrad)", row=2, col=c); fig2.update_yaxes(title_text="Vel. Angular (mrad/s)", row=2, col=c)

        fig2.write_html(caminho_plot_coroa, include_plotlyjs="cdn")

        print("Mapa de Poincaré e Espaço de Fase gerados com sucesso!")




            


        
        
            
            
        







        

        
        
        
            

        


        
        
            

        
