import numpy as np
import symforce
import os

# Symforce setup
symforce.set_symbolic_api("symengine")
symforce.set_log_level("warning")
symforce.set_epsilon_to_symbol()

from symforce import codegen
from symforce.codegen import codegen_util
from symforce import ops
import symforce.symbolic as sf
from symforce.values import Values
from symforce.notebook_util import display, display_code, display_code_file

out_put_save_directory = os.getcwd()

# Define symbols and initial setup
t_i = sf.V3.symbolic("t_i")
Rot_i = sf.Rot3.symbolic("Rot_i")
X_i = sf.Pose3(R=Rot_i, t=t_i)

t_f = sf.V3.symbolic("t_f")
Rot_f = sf.Rot3.symbolic("Rot_f")
X_f = sf.Pose3(R=Rot_f, t=t_f)

Rot_odo = sf.Rot3.symbolic("Rot_odo")
t_odo = sf.V3.symbolic("t_odo")
s = sf.Symbol('s')
T_odo = sf.Pose3(R=Rot_odo, t=s*t_odo)
display(T_odo)

a_1 = sf.V2.symbolic("a_1")
a_2 = sf.V2.symbolic("a_2")
a_3 = sf.V2.symbolic("a_3")
a_4 = sf.V2.symbolic("a_4")

r_delta, r_TSTA, r_Break = sf.symbols("r_delta r_TSTA r_Break")
theta_delta, theta_TSTA, theta_Break = sf.symbols("theta_delta theta_TSTA theta_Break")
m_0, n_0, k_0, h_0 = sf.symbols("m_0 n_0 k_0 h_0")

# Odometry error model
odo_error = X_f * X_i.inverse() * T_odo.inverse()
odo_error_in_tangent_list = odo_error.to_tangent()

print(len(odo_error_in_tangent_list))
odo_error_in_tangent = sf.V6()
odo_error_in_tangent[0] = odo_error_in_tangent_list[0]
odo_error_in_tangent[1] = odo_error_in_tangent_list[1]
odo_error_in_tangent[2] = odo_error_in_tangent_list[2]
odo_error_in_tangent[3] = odo_error_in_tangent_list[3]
odo_error_in_tangent[4] = odo_error_in_tangent_list[4]
odo_error_in_tangent[5] = odo_error_in_tangent_list[5]

# Functions
def odo_error_func(Rot_i: sf.Rot3, t_i: sf.V3,
                   Rot_f: sf.Rot3, t_f: sf.V3,
                   Rot_odo: sf.Rot3, t_odo: sf.V3,
                   s: sf.Symbol,
                   epsilon: sf.Scalar):
    return odo_error_in_tangent

residual_func_codegen = codegen.Codegen.function(func=odo_error_func, config=codegen.CppConfig(),)
residual_func_codegen_data = residual_func_codegen.generate_function(output_dir=out_put_save_directory)

def odo_error_func_wrt_s(Rot_i: sf.Rot3, t_i: sf.V3,
                         Rot_f: sf.Rot3, t_f: sf.V3,
                         Rot_odo: sf.Rot3, t_odo: sf.V3,
                         s: sf.Symbol,
                         epsilon: sf.Scalar):
                         
    return odo_error_in_tangent.diff(s)

residual_func_codegen = codegen.Codegen.function(func=odo_error_func_wrt_s, config=codegen.CppConfig(),)
residual_func_codegen_data = residual_func_codegen.generate_function(output_dir=out_put_save_directory)

def odo_error_func_wrt_pos_i(Rot_i: sf.Rot3, t_i: sf.V3,
                         Rot_f: sf.Rot3, t_f: sf.V3,
                         Rot_odo: sf.Rot3, t_odo: sf.V3,
                         s: sf.Symbol,
                         epsilon: sf.Scalar):
    return odo_error_in_tangent.jacobian(X_i)

residual_func_codegen = codegen.Codegen.function(func=odo_error_func_wrt_pos_i, config=codegen.CppConfig(),)
residual_func_codegen_data = residual_func_codegen.generate_function(output_dir=out_put_save_directory)

def odo_error_func_wrt_pos_f(Rot_i: sf.Rot3, t_i: sf.V3,
                         Rot_f: sf.Rot3, t_f: sf.V3,
                         Rot_odo: sf.Rot3, t_odo: sf.V3,
                         s: sf.Symbol,
                         epsilon: sf.Scalar):
    return odo_error_in_tangent.jacobian(X_f)

residual_func_codegen = codegen.Codegen.function(func=odo_error_func_wrt_pos_f, config=codegen.CppConfig(),)
residual_func_codegen_data = residual_func_codegen.generate_function(output_dir=out_put_save_directory)