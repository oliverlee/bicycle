#!/usr/bin/env python
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

    @classmethod
    def convert_state(cls, fbs_state):
        t = fbs_state._tab
        return np.frombuffer(t.Bytes[t.Pos : t.Pos +
            cls._n*fbsnt.Float64Flags.bytewidth], dtype=np.float64)

    @classmethod
    def convert_input(cls, fbs_input):
        t = fbs_input._tab
        return np.frombuffer(t.Bytes[t.Pos : t.Pos +
            cls._m*fbsnt.Float64Flags.bytewidth], dtype=np.float64)

    @classmethod
    def convert_output(cls, fbs_output):
        t = fbs_output._tab
        return np.frombuffer(t.Bytes[t.Pos : t.Pos +
            cls._l*fbsnt.Float64Flags.bytewidth], dtype=np.float64)

    @classmethod
    def convert_measurement(cls, fbs_measurement):
        t = fbs_measurement._tab
        return np.frombuffer(t.Bytes[t.Pos : t.Pos +
            cls._l*fbsnt.Float64Flags.bytewidth], dtype=np.float64)

    @classmethod
    def convert_second_order_matrix(cls, fbs_second_order_matrix):
        t = fbs_second_order_matrix._tab
        return np.frombuffer(t.Bytes[t.Pos : t.Pos +
            cls._o*cls._o*fbsnt.Float64Flags.bytewidth],
            dtype=np.float64).reshape((cls._o, cls._o))

    @classmethod
    def convert_symmetric_state_matrix(cls, fbs_symmetric_state_matrix):
        t = fbs_symmetric_state_matrix._tab
        a = np.frombuffer(t.Bytes[t.Pos : t.Pos +
            (cls._n*(cls._n + 1)/2)*fbsnt.Float64Flags.bytewidth],
            dtype=np.float64)
        b = np.zeros((cls._n, cls._n))
        b[np.triu_indices(cls._n)] = a
        return b + np.triu(b, 1).T

    @classmethod
    def convert_symmetric_input_matrix(cls, fbs_symmetric_input_matrix):
        t = fbs_symmetric_input_matrix._tab
        a = np.frombuffer(t.Bytes[t.Pos : t.Pos +
            (cls._m*(cls._m + 1)/2)*fbsnt.Float64Flags.bytewidth],
            dtype=np.float64)
        b = np.zeros((cls._m, cls._m))
        b[np.triu_indices(cls._m)] = a
        return b + np.triu(b, 1).T

    @classmethod
    def convert_symmetric_output_matrix(cls, fbs_symmetric_output_matrix):
        t = fbs_symmetric_output_matrix._tab
        a = np.frombuffer(t.Bytes[t.Pos : t.Pos +
            (cls._l*(cls._l + 1)/2)*fbsnt.Float64Flags.bytewidth],
            dtype=np.float64)
        b = np.zeros((cls._l, cls._l))
        b[np.triu_indices(cls._l)] = a
        return b + np.triu(b, 1).T

    @classmethod
    def convert_kalman_gain_matrix(cls, fbs_kalman_gain_matrix):
        t = fbs_kalman_gain_matrix._tab
        return np.frombuffer(t.Bytes[t.Pos : t.Pos +
            cls._n*cls._l*fbsnt.Float64Flags.bytewidth],
            dtype=np.float64).reshape((cls._n, cls._l))

    @classmethod
    def convert_lqr_gain_matrix(cls, fbs_lqr_gain_matrix):
        t = fbs_lqr_gain_matrix._tab
        return np.frombuffer(t.Bytes[t.Pos : t.Pos +
            cls._m*cls._n*fbsnt.Float64Flags.bytewidth],
            dtype=np.float64).reshape((cls._m, cls._n))

    @classmethod
    def convert_bicycle(cls, fbs_bicycle):
        v = 0
        dt = 0
        M = np.zeros((cls._o, cls._o))
        C1 = np.zeros((cls._o, cls._o))
        K0 = np.zeros((cls._o, cls._o))
        K2 = np.zeros((cls._o, cls._o))
        if fbs_bicycle is None:
            return Bicycle(v, dt, M, C1, K0, K2)

        value = fbs_bicycle.V()
        if value != 0:
            v = value
        value = fbs_bicycle.Dt()
        if value != 0:
            dt = value
        value = fbs_bicycle.M()
        if value is not None:
            M = cls.convert_second_order_matrix(value)
        value = fbs_bicycle.C1()
        if value is not None:
            C1 = cls.convert_second_order_matrix(value)
        value = fbs_bicycle.K0()
        if value is not None:
            K0 = cls.convert_second_order_matrix(value)
        value = fbs_bicycle.K2()
        if value is not None:
            K2 = cls.convert_second_order_matrix(value)
        return Bicycle(v, dt, M, C1, K0, K2)

    @classmethod
    def convert_kalman(cls, fbs_kalman):
        x = np.zeros((cls._n, 1))
        P = np.zeros((cls._n, cls._n))
        Q = np.zeros((cls._n, cls._n))
        R = np.zeros((cls._l, cls._l))
        K = np.zeros((cls._n, cls._l))
        if fbs_kalman is None:
            return Kalman(x, P, Q, R, K)

        value = fbs_kalman.StateEstimate()
        if value is not None:
            x = cls.convert_state(value)
        value = fbs_kalman.ErrorCovariance()
        if value is not None:
            P = cls.convert_symmetric_state_matrix(value)
        value = fbs_kalman.ProcessNoiseCovariance()
        if value is not None:
            Q = cls.convert_symmetric_state_matrix(value)
        value = fbs_kalman.MeasurementNoiseCovariance()
        if value is not None:
            R = cls.convert_symmetric_output_matrix(value)
        value = fbs_kalman.KalmanGain()
        if value is not None:
            K = cls.convert_kalman_gain_matrix(value)
        return Kalman(x, P, Q, R, K)

    @classmethod
    def convert_lqr(cls, fbs_lqr):
        n = 0
        r = np.zeros((cls._n, 1))
        Q = np.zeros((cls._n, cls._n))
        R = np.zeros((cls._m, cls._m))
        P = np.zeros((cls._n, cls._n))
        K = np.zeros((cls._m, cls._n))
        if fbs_lqr is None:
            return Lqr(n, r, Q, R, P, K)

        value = fbs_lqr.Horizon()
        if value != 0:
            n = value
        value = fbs_lqr.StateTarget()
        if value is not None:
            r = cls.convert_state(value)
        value = fbs_lqr.StateCost()
        if value is not None:
            Q = cls.convert_symmetric_state_matrix(value)
        value = fbs_lqr.InputCost()
        if value is not None:
            R = cls.convert_symmetric_input_matrix(value)
        value = fbs_lqr.HorizonCost()
        if value is not None:
            P = cls.convert_symmetric_state_matrix(value)
        value = fbs_lqr.LqrGain()
        if value is not None:
            K = cls.convert_lqr_gain_matrix(value)
        return Lqr(n, r, Q, R, P, K)

    @classmethod
    def convert_sample(cls, fbs_sample):
        t = 0
        bicycle = cls.convert_bicycle(None)
        kalman = cls.convert_kalman(None)
        lqr = cls.convert_lqr(None)
        x = np.zeros((cls._n, 1))
        u = np.zeros((cls._m, 1))
        y = np.zeros((cls._l, 1))
        z = np.zeros((cls._l, 1))
        if fbs_sample is None:
            return Sample(t, bicycle, kalman, lqr, x, u, y, z)

        value = fbs_sample.Timestamp()
        if value != 0:
            t = value
        value = fbs_sample.Bicycle()
        if value is not None:
            bicycle = cls.convert_bicycle(value)
        value = fbs_sample.Kalman()
        if value is not None:
            kalman = cls.convert_kalman(value)
        value = fbs_sample.Lqr()
        if value is not None:
            lqr = cls.convert_lqr(value)
        value = fbs_sample.State()
        if value is not None:
            x = cls.convert_state(value)
        value = fbs_sample.Input()
        if value is not None:
            u = cls.convert_input(value)
        value = fbs_sample.Output()
        if value is not None:
            y = cls.convert_output(value)
        value = fbs_sample.Measurement()
        if value is not None:
            z = cls.convert_measurement(value)
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
