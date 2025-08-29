import numpy as np

def multiply_matrix(A,B):
    new_matrix = np.array([[sum(a*b for a,b in zip(A_row,B_col)) for B_col in zip(*B)] for A_row in A])
    return new_matrix

def skew_symmetric_matrix_from_vector(v):
    if np.linalg.norm(v) != 0:
        v.shape = (3, 1)
        skew_matrix = np.array([
            [0., -v[2,0], v[1,0]],
            [v[2,0], 0., -v[0,0]],
            [-v[1,0], v[0,0], 0.]])
    else:
        skew_matrix = np.zeros((3, 3))

    return skew_matrix

def cross_prod(a, b):
    result = np.array([a[1]*b[2] - a[2]*b[1],
            a[2]*b[0] - a[0]*b[2],
            a[0]*b[1] - a[1]*b[0]])

    return result