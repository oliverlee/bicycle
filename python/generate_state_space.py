#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import numpy as np
import scipy.signal as sig
from dtk.bicycle import *


def print_matrix(A):
    print(',\n'.join(', '.join('{:0.16e}'.format(cell) for cell in row)
          for row in A) + ';\n')


def generate_state_space(v, dt):
    A1, B1 = benchmark_state_space(*benchmark_matrices(), v=v, g=9.80665)
    A = np.zeros((5, 5))
    B = np.zeros((5, 2))
    A[1:, 1:] = A1
    B[1:, :] = B1
    p = benchmark_parameters()
    A[0, 2] = v*np.cos(p['lambda'])/p['w']
    A[0, 4] = p['c']*np.cos(p['lambda'])/p['w']

    C = np.eye(5)
    D = np.zeros((5, 2))
    Ad, Bd, _, _, _ = sig.cont2discrete((A, B, C, D), dt, 'bilinear')

    print('A <<')
    print_matrix(A)

    print('B <<')
    print_matrix(B)

    print('Ad <<')
    print_matrix(Ad)

    print('Bd <<')
    print_matrix(Bd)


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print(("Usage: {} <v> <dt>\n\nGenerate state spaces matrices for " +
               "benchmark bicycle.").format(__file__))
        sys.exit(1)

    np.set_printoptions(precision=16)
    generate_state_space(float(sys.argv[1]), float(sys.argv[2]))
    sys.exit(0)
