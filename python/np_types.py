#!/usr/in/env python
# -*- coding: utf-8 -*-
import types
import numpy as np


# TODO: generate following sizes with templates
state_size = 4
input_size = 2
output_size = 2
second_order_size = 2

natural_t = np.uint32
real_t = np.float64

state_t = (real_t, (state_size, 1))
input_t = (real_t, (input_size, 1))
output_t = (real_t, (output_size, 1))

state_matrix_t = (real_t, (state_size, state_size))
second_order_matrix_t = (real_t, (second_order_size, second_order_size))
lqr_gain_matrix_t = (real_t, (input_size, state_size))
kalman_gain_matrix_t = (real_t, (state_size, output_size))

symmetric_state_matrix_t = state_matrix_t
symmetric_output_matrix_t = (real_t, (output_size, output_size))
symmetric_input_matrix_t = (real_t, (input_size, input_size))

bicycle_t = [
        ('v', real_t),                      # bicycle forward speed
        ('dt', real_t),                     # discrete time system sample time
        ('M',) + second_order_matrix_t,     # mass matrix
        ('C1',) + second_order_matrix_t,    # v proportional damping matrix
        ('K0',) + second_order_matrix_t,    # v independent stiffness matrix
        ('K2',) + second_order_matrix_t]    # v**2 prop. stiffness matrix

kalman_t = [
        ('x',) + state_t,                   # state estimate
        ('P',) + symmetric_state_matrix_t,  # error covariance
        ('Q',) + symmetric_state_matrix_t,  # process noise covariance
        ('R',) + symmetric_input_matrix_t,  # measurement noise covariance
        ('K',) + kalman_gain_matrix_t]      # Kalman gain

lqr_t = [
        ('n', natural_t),                   # horizon length (in samples)
        ('r',) + state_t,                   # state reference
        ('Q',) + symmetric_state_matrix_t,  # state cost
        ('R',) + symmetric_input_matrix_t,  # input cost
        ('P',) + symmetric_state_matrix_t,  # horizon cost
        ('K',) + lqr_gain_matrix_t]         # LQR gain

sample_t = [
        ('ts', natural_t),                  # timestamp
        ('bicycle', bicycle_t),
        ('kalman', kalman_t),
        ('lqr', lqr_t),
        ('x',) + state_t,                   # system state
        ('u',) + input_t,                   # system input
        ('y',) + output_t,                  # system output
        ('z',) + output_t]                  # system output with noise


class Record(types.SimpleNamespace):
    def __init__(self, dtype, length, mask=False):
        self.dtype = np.dtype(dtype)
        self.size = length
        for name, subtype in self.dtype.fields.items():
            subtype = subtype[0]
            if subtype.fields:
                r = Record(subtype, length)
            else:
                r = np.ma.zeros(length, dtype=subtype)
                if mask:
                    r.mask = True
            setattr(self, name, r)
        self._fill_value = tuple(getattr(self, f).fill_value
                                 for f in self.dtype.names)

    def __getitem__(self, indx):
        record = Record(self.dtype, 0)
        for name in self.dtype.names:
            setattr(record, name, getattr(self, name)[indx])
            if not record.size:
                try:
                    record.size = len(getattr(record, name))
                except TypeError:
                    record.size = 1
        return record

    def __setitem__(self, indx, value):
        for name in self.dtype.names:
            old = getattr(self, name)[indx]
            new = getattr(value, name)
            if len(new) == 1:
                new = new[0]
            old = new
        return None

    def __str__(self):
        if self.size > 1:
            mstr = ["(%s)" % ",".join([str(i) for i in s])
                    for s in zip(*[getattr(self, f) for f in self.dtype.names])]
            return "[%s]" % ", ".join(mstr)
        else:
            mstr = ["%s" % ",".join([str(i) for i in s])
                    for s in zip([getattr(self, f) for f in self.dtype.names])]
            return "(%s)" % ", ".join(mstr)

    def __repr__(self):
        _names = self.dtype.names
        fmt = "%%%is : %%s" % (max([len(n) for n in _names]) + 4,)
        reprstr = [fmt % (f, getattr(self, f)) for f in self.dtype.names]
        reprstr.insert(0, 'Record(')
        reprstr.extend([fmt % ('    fill_value', self.fill_value),
                         '              )'])
        return str("\n".join(reprstr))

    def __len__(self):
        return self.size

    @property
    def fill_value(self):
        return self._fill_value
