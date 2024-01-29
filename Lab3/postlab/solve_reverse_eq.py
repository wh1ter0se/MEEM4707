from sympy import symbols, Eq, solve

# Define the symbols
V_command, omega_command, V_actual, omega_actual, a1, a2, a3, a4, b = symbols('V_command omega_command V_actual omega_actual a1 a2 a3 a4 b')

# Given equations
eq1 = Eq(V_actual, a1 * V_command + 0.25 * a2 * b * omega_command + a3)
eq2 = Eq(omega_actual, (a2 / b) * V_command + a1 * omega_command + a4)

# Solve the system for V_command and omega_command
solutions = solve((eq1, eq2), (V_command, omega_command))

print(solutions)
