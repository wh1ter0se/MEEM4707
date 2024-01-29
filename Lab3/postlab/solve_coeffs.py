import math
from sympy import symbols, Eq, solve

# Define the symbols
a1, a2, a3, a4 = symbols('a1 a2 a3 a4')
b = 0.287

# Straight line values
V_cmd1 = 0.05
V_act1 = 0.0505
omega_cmd1 = 0
omega_act1 = -0.0134

# Turning values
V_cmd2 = 0
V_act2 = 0.0028
omega_cmd2 = math.radians(-90/10) # -0.313
omega_act2 = -0.1573

# Given equations
eq1 = Eq(V_act1, a1 * V_cmd1 + 0.25 * a2 * b * omega_cmd1 + a3)
eq2 = Eq(V_act2, a1 * V_cmd2 + 0.25 * a2 * b * omega_cmd2 + a3)
eq3 = Eq(omega_act1, (a2 / b) * V_cmd1 + a1 * omega_cmd1 + a4)
eq4 = Eq(omega_act2, (a2 / b) * V_cmd2 + a1 * omega_cmd2 + a4)

# Solve the system for V_command and omega_command
solutions = solve((eq1, eq2, eq3, eq4), (a1, a2, a3, a4))

print(solutions)
