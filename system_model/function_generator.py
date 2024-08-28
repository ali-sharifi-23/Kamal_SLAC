import numpy as np
import symforce
import sympy as sp
import os 

symforce.set_symbolic_api("symengine")
symforce.set_log_level("warning")

symforce.set_epsilon_to_symbol()

from symforce import codegen
from symforce.codegen import codegen_util
from symforce import ops
import symforce.symbolic as sf
from symforce.values import Values
from symforce.notebook_util import display, display_code, display_code_file
from itertools import permutations

out_put_save_directory = os.getcwd()

# initial pose
t_i = sf.V3.symbolic("t_i")
Rot_i = sf.Rot3.symbolic("Rot_i").to_rotation_matrix()
X_i = sf.Matrix44.eye()
X_i[:3, :3] = Rot_i
X_i[:3, 3] = t_i
# final pose
t_f = sf.V3.symbolic("t_f")
Rot_f = sf.Rot3.symbolic("Rot_f").to_rotation_matrix()
X_f = sf.Matrix44.eye()
X_f[:3, :3] = Rot_f
X_f[:3, 3] = t_f
# odometry measurment
Rot_odo = sf.Rot3.symbolic("Rot_odo").to_rotation_matrix()
t_odo = sf.V3.symbolic("t_odo")
s = sf.Symbol('s')
# anchor points
a_1 = sf.V2.symbolic("a_1")
a_2 = sf.V2.symbolic("a_2")
a_3 = sf.V2.symbolic("a_3")
a_4 = sf.V2.symbolic("a_4")
# 
t_0 = sf.V3.symbolic("t_0")
Rot_0 = sf.Rot3.symbolic("Rot_0").to_rotation_matrix()
X_0 = sf.Matrix44.eye()
X_0[:3, :3] = Rot_0
X_0[:3, 3] = t_0
# forward kinematic
r_delta = 0.025
r_TSTA = 0.035
r_Break = 0.035
theta_delta, theta_TSTA, theta_Break = sf.symbols("theta_delta theta_TSTA theta_Break")
m_0 = sf.sqrt((t_0[0] - a_1[0])**2 + (t_0[1] - a_1[1])**2)
n_0 = sf.sqrt((t_0[0] - a_2[0])**2 + (t_0[1] - a_2[1])**2)
k_0 = sf.sqrt((t_0[0] - a_3[0])**2 + (t_0[1] - a_3[1])**2)
h_0 = sf.sqrt((t_0[0] - a_4[0])**2 + (t_0[1] - a_4[1])**2)
l_0 = sf.V3.symbolic("")
l_0[0] = m_0 + n_0
l_0[1] = m_0 + 2*k_0
l_0[2] = n_0 + 2*h_0
l = sf.V3.symbolic("")
l[0] = r_delta*theta_delta + l_0[0]
l[1] = r_TSTA*theta_TSTA + l_0[1]
l[2] = r_Break*theta_Break + l_0[2]
m = (2*l[0] + 0.5*(l[1] + l[2] - l[0]))/(l[0]**2 - 0.25*(l[2]**2 + l[0]**2 - 2*l[0]*l[2] - l[1]**2))
n = l[0] - m
k = 0.5*(l[1] - m)
h = 0.5*(l[2] - n)
X = sf.V2.symbolic("")
X[0] = (m**2 - n**2 + a_2[0]**2)/(2*a_2[0])
X[1] = a_2[1] - sf.sqrt(m**2 - X[0]**2)


def odo_error_func(X_i: sf.Matrix44, X_f: sf.Matrix44,
                  Rot_odo: sf.Rot3.symbolic("Rot_odo").to_rotation_matrix(), t_odo: sf.V3.symbolic("t_odo"),
                   s: sf.Symbol('s')) ->sf.V6:
    T_odo = sf.Matrix44.eye()
    T_odo[:3, :3] = Rot_odo
    T_odo[:3, 3] = t_odo
    T_odo[3, 3] = 1/s
    error = X_f*X_i.inv() - T_odo
    error_in_tangent_list = error.to_tangent()
    error_in_tangent = sf.V6.symbolic("")
    error_in_tangent[0] = error_in_tangent_list[0]
    error_in_tangent[1] = error_in_tangent_list[1]
    error_in_tangent[2] = error_in_tangent_list[2]
    error_in_tangent[3] = error_in_tangent_list[3]
    error_in_tangent[4] = error_in_tangent_list[4]
    error_in_tangent[5] = error_in_tangent_list[5]
    return error_in_tangent
residual_func_codegen = codegen.Codegen.function(func=odo_error_func, config=codegen.CppConfig(),)
residual_func_codegen_data = residual_func_codegen.generate_function(output_dir=out_put_save_directory)

def odo_error_func_wrt_s(X_i: sf.Matrix44, X_f: sf.Matrix44,
                        Rot_odo: sf.Rot3.symbolic("Rot_odo").to_rotation_matrix(), t_odo: sf.V3.symbolic("t_odo"),
                   s: sf.Symbol('s')) ->sf.Matrix:
    error = odo_error_func(X_i, X_f, Rot_odo, t_odo, s)
    error_wrt_s = error.diff(s)
    return error_wrt_s
residual_func_codegen = codegen.Codegen.function(func=odo_error_func_wrt_s, config=codegen.CppConfig(),)
residual_func_codegen_data = residual_func_codegen.generate_function(output_dir=out_put_save_directory)

def odo_error_func_wrt_pos_i(X_i: sf.Matrix44, X_f: sf.Matrix44,
                        Rot_odo: sf.Rot3.symbolic("Rot_odo").to_rotation_matrix(), t_odo: sf.V3.symbolic("t_odo"),
                   s: sf.Symbol('s')) ->sf.Matrix:
    error = odo_error_func(X_i, X_f, Rot_odo, t_odo, s)
    error_wrt_pos_i = error.jacobian(X_i)
    return error_wrt_pos_i
residual_func_codegen = codegen.Codegen.function(func=odo_error_func_wrt_pos_i, config=codegen.CppConfig(),)
residual_func_codegen_data = residual_func_codegen.generate_function(output_dir=out_put_save_directory)

def odo_error_func_wrt_pos_f(X_i: sf.Matrix44, X_f: sf.Matrix44,
                        Rot_odo: sf.Rot3.symbolic("Rot_odo").to_rotation_matrix(), t_odo: sf.V3.symbolic("t_odo"),
                   s: sf.Symbol('s')) ->sf.Matrix:
    error = odo_error_func(X_i, X_f, Rot_odo, t_odo, s)
    error_wrt_pos_f = error.jacobian(X_f)
    return error_wrt_pos_f
residual_func_codegen = codegen.Codegen.function(func=odo_error_func_wrt_pos_f, config=codegen.CppConfig(),)
residual_func_codegen_data = residual_func_codegen.generate_function(output_dir=out_put_save_directory)
