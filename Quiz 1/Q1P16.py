import numpy as np

mat = np.zeros((5,5))
for i in range(5):
    for j in range(5):
        val = i + j + 2
        mat[i, j] = val + 2 if val % 2 == 0 else val

colsums = [sum(mat[col_indx]) for col_indx in range(5)]
print(colsums)