import math

vals = [2.3, 1.4, 0.5, 7.7, 1.7, 0.4, math.inf, 1.2, 4, 1.8 ]

min_val = min(vals)
min_indx = vals.index(min_val)
print(f"Minimum value is {min_val} at index {min_indx}")

print(f"Fourth element: {vals[3]}") 