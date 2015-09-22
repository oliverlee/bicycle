#!/usr/in/env python
# -*- coding: utf-8 -*-
from collections import namedtuple
import sys
import numpy as np
import flatbuffers
import flatbuffers.number_types as fbsnt
import fbs.SampleLog
import fbs.Sample


def load_sample_log(filename):
    with open(filename, 'rb') as f:
        buf = f.read()
    log = fbs.SampleLog.SampleLog.GetRootAsSampleLog(bytearray(buf), 0)
    samples = []
    for i in range(log.SamplesLength()):
        sb = log.Samples(i)
        o = flatbuffers.number_types.UOffsetTFlags.py_type(sb._tab.Offset(4))
        a = sb._tab.Vector(o)
        s = fbs.Sample.Sample.GetRootAsSample(
                sb._tab.Bytes[a : a + sb.DataLength()], 0)
        samples.append(s)
    return samples


Bicycle = namedtuple('Bicycle',
        ['v',   # forward speed
         'dt',  # sample time
         'M',   # mass matrix
         'C1',  # v proportional damping matrix
         'K0',  # speed independent stiffness matrix
         'K2']) # v**2 proportional stiffness matrix

Kalman = namedtuple('Kalman',
        ['x',   # state estimate
         'P',   # error covariance
         'Q',   # process noise covariance
         'R',   # measurement noise covariance
         'K'])  # Kalman gain

Lqr = namedtuple('Lqr',
        ['n',   # horizon length
         'r',   # state reference
         'Q',   # state cost
         'R',   # input cost
         'P',   # horizon cost
         'K'])  # LQR gain

Sample = namedtuple('Sample',
        ['t',       # timestamp
         'bicycle', # bicycle namedtuple
         'kalman',  # kalman namedtuple
         'lqr',     # lqr namedtuple
         'x',       # state vector
         'u',       # input vector
         'y',       # output vector
         'z'])      # measurement vector


class Convert(object):
    # TODO: generate following sizes with templates
    _n = 4 # state size
    _m = 2 # input size
    _l = 2 # output size
    _o = 2 # second order size

    @staticmethod
    def __ma(x, mask=False):
        """Convenience function to create numpy masked array."""
        if mask:
            return np.ma.array([x], mask=True)
        return np.ma.array([x])

    @classmethod
    def __get_struct_value(cls, accessor, converter, default):
        value = accessor()
        if value is not None:
            return cls.__ma(cls.converter(value))
        return default

    @classmethod
    def __get_scalar_value(cls, accessor, default_scalar_value, default):
        value = accessor()
        if value != default_scalar_value:
            return cls.__ma(value)
        return default

    @classmethod
    def convert_state(cls, fbs_state):
        t = fbs_state._tab
        return np.ma.frombuffer(t.Bytes[t.Pos : t.Pos +
            cls._n*fbsnt.Float64Flags.bytewidth], dtype=np.float64)

    @classmethod
    def convert_input(cls, fbs_input):
        t = fbs_input._tab
        return np.ma.frombuffer(t.Bytes[t.Pos : t.Pos +
            cls._m*fbsnt.Float64Flags.bytewidth], dtype=np.float64)

    @classmethod
    def convert_output(cls, fbs_output):
        t = fbs_output._tab
        return np.ma.frombuffer(t.Bytes[t.Pos : t.Pos +
            cls._l*fbsnt.Float64Flags.bytewidth], dtype=np.float64)

    @classmethod
    def convert_measurement(cls, fbs_measurement):
        t = fbs_measurement._tab
        return np.ma.frombuffer(t.Bytes[t.Pos : t.Pos +
            cls._l*fbsnt.Float64Flags.bytewidth], dtype=np.float64)

    @classmethod
    def convert_second_order_matrix(cls, fbs_second_order_matrix):
        t = fbs_second_order_matrix._tab
        return np.ma.frombuffer(t.Bytes[t.Pos : t.Pos +
            cls._o*cls._o*fbsnt.Float64Flags.bytewidth],
            dtype=np.float64).reshape((cls._o, cls._o))

    @classmethod
    def convert_symmetric_state_matrix(cls, fbs_symmetric_state_matrix):
        t = fbs_symmetric_state_matrix._tab
        a = np.ma.frombuffer(t.Bytes[t.Pos : t.Pos +
            (cls._n*(cls._n + 1)/2)*fbsnt.Float64Flags.bytewidth],
            dtype=np.float64)
        b = np.zeros((cls._n, cls._n))
        b[np.triu_indices(cls._n)] = a
        return b + np.triu(b, 1).T

    @classmethod
    def convert_symmetric_input_matrix(cls, fbs_symmetric_input_matrix):
        t = fbs_symmetric_input_matrix._tab
        a = np.ma.frombuffer(t.Bytes[t.Pos : t.Pos +
            (cls._m*(cls._m + 1)/2)*fbsnt.Float64Flags.bytewidth],
            dtype=np.float64)
        b = np.zeros((cls._m, cls._m))
        b[np.triu_indices(cls._m)] = a
        return b + np.triu(b, 1).T

    @classmethod
    def convert_symmetric_output_matrix(cls, fbs_symmetric_output_matrix):
        t = fbs_symmetric_output_matrix._tab
        a = np.ma.frombuffer(t.Bytes[t.Pos : t.Pos +
            (cls._l*(cls._l + 1)/2)*fbsnt.Float64Flags.bytewidth],
            dtype=np.float64)
        b = np.zeros((cls._l, cls._l))
        b[np.triu_indices(cls._l)] = a
        return b + np.triu(b, 1).T

    @classmethod
    def convert_kalman_gain_matrix(cls, fbs_kalman_gain_matrix):
        t = fbs_kalman_gain_matrix._tab
        return np.ma.frombuffer(t.Bytes[t.Pos : t.Pos +
            cls._n*cls._l*fbsnt.Float64Flags.bytewidth],
            dtype=np.float64).reshape((cls._n, cls._l))

    @classmethod
    def convert_lqr_gain_matrix(cls, fbs_lqr_gain_matrix):
        t = fbs_lqr_gain_matrix._tab
        return np.ma.frombuffer(t.Bytes[t.Pos : t.Pos +
            cls._m*cls._n*fbsnt.Float64Flags.bytewidth],
            dtype=np.float64).reshape((cls._m, cls._n))

    @classmethod
    def convert_bicycle(cls, fbs_bicycle):
        v = cls.__ma(0, True)
        dt = cls.__ma(0, True)
        M = cls.__ma(np.zeros((cls._o, cls._o)), True)
        C1 = cls.__ma(np.zeros((cls._o, cls._o)), True)
        K0 = cls.__ma(np.zeros((cls._o, cls._o)), True)
        K2 = cls.__ma(np.zeros((cls._o, cls._o)), True)
        if fbs_bicycle is None:
            return Bicycle(v, dt, M, C1, K0, K2)

        # forward velocity must be nonzero
        # sample period must be greater than zero
        v = cls.__get_scalar_value(fbs_bicycle.V, 0, v)
        dt = cls.__get_scalar_value(fbs_bicycle.Dt, 0, dt)
        M = cls.__get_struct_value(fbs_bicycle.M,
                                   convert_second_order_matrix, M)
        C1 = cls.__get_struct_value(fbs_bicycle.C1,
                                    convert_second_order_matrix, C1)
        K0 = cls.__get_struct_value(fbs_bicycle.K0,
                                    convert_second_order_matrix, K0)
        K2 = cls.__get_struct_value(fbs_bicycle.K2,
                                    convert_second_order_matrix, K2)
        return Bicycle(v, dt, M, C1, K0, K2)

    @classmethod
    def convert_kalman(cls, fbs_kalman):
        x = cls.__ma(np.zeros((cls._n, 1)), True)
        P = cls.__ma(np.zeros((cls._n, cls._n)), True)
        Q = cls.__ma(np.zeros((cls._n, cls._n)), True)
        R = cls.__ma(np.zeros((cls._l, cls._l)), True)
        K = cls.__ma(np.zeros((cls._n, cls._l)), True)
        if fbs_kalman is None:
            return Kalman(x, P, Q, R, K)

        x = cls.__get_struct_value(fbs_kalman.StateEstimate, convert_state, x)
        R = cls.__get_struct_value(fbs_kalman.ErrorCovariance,
                                   convert_symmetric_state_matrix, R)
        Q = cls.__get_struct_value(fbs_kalman.ProcessNoiseCovariance,
                                   convert_symmetric_state_matrix, Q)
        R = cls.__get_struct_value(fbs_kalman.MeasurementNoiseCovariance,
                                   convert_symmetric_output_matrix, R)
        K = cls.__get_struct_value(fbs_kalman.KalmanGain,
                                   convert_kalman_gain_matrix, K)
        return Kalman(x, P, Q, R, K)

    @classmethod
    def convert_lqr(cls, fbs_lqr):
        n = cls.__ma(0, True)
        r = cls.__ma(np.zeros((cls._n, 1)), True)
        Q = cls.__ma(np.zeros((cls._n, cls._n)), True)
        R = cls.__ma(np.zeros((cls._m, cls._m)), True)
        P = cls.__ma(np.zeros((cls._n, cls._n)), True)
        K = cls.__ma(np.zeros((cls._m, cls._n)), True)
        if fbs_lqr is None:
            return Lqr(n, r, Q, R, P, K)

        # horizon length must be greater than zero
        n = cls.__get_scalar_value(fbs_lqr.Horizon, 0, n)
        r = cls.__get_struct_value(fbs_lqr.StateTarget, cls.convert_state, r)
        Q = cls.__get_struct_value(fbs_lqr.StateCost,
                                   convert_symmetric_state_matrix, Q)
        R = cls.__get_struct_value(fbs_lqr.InputCost,
                                   convert_symmetric_input_matrix, R)
        P = cls.__get_struct_value(fbs_lqr.HorizonCost,
                                   convert_symmetric_state_matrix, P)
        K = cls.__get_struct_value(fbs_lqr.LqrGain, convert_lqr_gain_matrix, K)
        return Lqr(n, r, Q, R, P, K)

    @classmethod
    def convert_sample(cls, fbs_sample):
        t = cls.__ma(0, True)
        print('none bike')
        print(cls.convert_bicycle(None))
        bicycle = cls.__ma(cls.convert_bicycle(None), True)
        kalman = cls.__ma(cls.convert_kalman(None), True)
        lqr = cls.__ma(cls.convert_lqr(None), True)
        x = cls.__ma(np.zeros((cls._n, 1)), True)
        u = cls.__ma(np.zeros((cls._m, 1)), True)
        y = cls.__ma(np.zeros((cls._l, 1)), True)
        z = cls.__ma(np.zeros((cls._l, 1)), True)
        if fbs_sample is None:
            return Sample(t, bicycle, kalman, lqr, x, u, y, z)

        # assume timestamp is always serialized
        t = cls.__ma(fbs_sample.Timestamp())
        bicycle = cls.__get_struct_value(fbs_sample.Bicycle,
                                         cls.convert_bicycle, bicycle)
        kalman = cls.__get_struct_value(fbs_sample.Kalman,
                                        cls.convert_kalman, kalman)
        lqr = cls.__get_struct_value(fbs_sample.Lqr, cls.convert_lqr, lqr)
        x = cls.__get_struct_value(fbs_sample.State, cls.convert_state, x)
        u = cls.__get_struct_value(fbs_sample.Input, cls.convert_input, u)
        y = cls.__get_struct_value(fbs_sample.Output, cls.convert_output, y)
        z = cls.__get_struct_value(fbs_sample.Measurement,
                                   cls.convert_measurement, z)
        return Sample(t, bicycle, kalman, lqr, x, u, y, z)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(("Usage: {} <sample_log_file>\n\nPrint sample log " +
               "data.").format(__file__));
        sys.exit(1)

    samples = load_sample_log(sys.argv[1])
    for s in samples:
        sample = Convert.convert_sample(s)
        print(sample)
    sys.exit(0)
