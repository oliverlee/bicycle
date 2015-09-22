#!/usr/in/env python
# -*- coding: utf-8 -*-
import sys
import types
import numpy as np
import numpy.ma.mrecords as mr
import flatbuffers
from flatbuffers.number_types import Float64Flags as fbfloat64
import fbs.SampleLog
import fbs.Sample
import np_types as npt


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


def set_masked_record_field(record_field, accessor,
                            converter=None, masked_value=None):
    value = accessor()
    if converter is None:
        # field type is a scalar value
        if value == masked_value:
            record_field[0] = np.ma.masked
        else:
            record_field[0] = value
    else:
        # field type is flatbuffers struct or table
        if value is None:
            record_field[0] = np.ma.masked
        else:
            record_field[0] = converter(value)
    return None

# flatbuffers struct converters
def convert_state(fbs_state):
    t = fbs_state._tab
    return np.ma.frombuffer(t.Bytes[t.Pos : t.Pos +
        npt.state_size*fbfloat64.bytewidth], dtype=npt.state_t)

def convert_input(fbs_input):
    t = fbs_input._tab
    return np.ma.frombuffer(t.Bytes[t.Pos : t.Pos +
        npt.input_size*fbfloat64.bytewidth], dtype=npt.input_t)

def convert_output(fbs_output):
    t = fbs_output._tab
    return np.ma.frombuffer(t.Bytes[t.Pos : t.Pos +
        npt.output_size*fbfloat64.bytewidth], dtype=npt.output_t)

def convert_second_order_matrix(fbs_second_order_matrix):
    t = fbs_second_order_matrix._tab
    return np.ma.frombuffer(t.Bytes[t.Pos : t.Pos +
        (npt.second_order_size**2)*fbfloat64.bytewidth],
        dtype=npt.second_order_matrix_t)

def convert_symmetric_state_matrix(fbs_symmetric_state_matrix):
    t = fbs_symmetric_state_matrix._tab
    a = np.ma.frombuffer(t.Bytes[t.Pos : t.Pos +
        (npt.state_size*(npt.state_size + 1)/2)*fbfloat64.bytewidth],
        dtype=npt.real_t)
    b = np.ma.zeros([], dtype=npt.symmetric_state_matrix_t)
    b[np.triu_indices(npt.state_size)] = a
    return b + np.triu(b, 1).T

def convert_symmetric_input_matrix(fbs_symmetric_input_matrix):
    t = fbs_symmetric_input_matrix._tab
    a = np.ma.frombuffer(t.Bytes[t.Pos : t.Pos +
        (npt.input_size*(npt.input_size + 1)/2)*fbfloat64.bytewidth],
        dtype=npt.real_t)
    b = np.ma.zeros([], dtype=npt.symmetric_input_matrix_t)
    b[np.triu_indices(npt.input_size)] = a
    return b + np.triu(b, 1).T

def convert_symmetric_output_matrix(fbs_symmetric_output_matrix):
    t = fbs_symmetric_output_matrix._tab
    a = np.ma.frombuffer(t.Bytes[t.Pos : t.Pos +
        (npt.output_size*(npt.output_size + 1)/2)*fbfloat64.bytewidth],
        dtype=npt.real_t)
    b = np.ma.zeros([], dtype=npt.symmetric_output_matrix_t)
    b[np.triu_indices(npt.output_size)] = a
    return b + np.triu(b, 1).T

def convert_kalman_gain_matrix(fbs_kalman_gain_matrix):
    t = fbs_kalman_gain_matrix._tab
    return np.ma.frombuffer(t.Bytes[t.Pos : t.Pos +
        (npt.state_size * npt.output_size)*fbfloat64.bytewidth],
        dtype=npt.kalman_gain_matrix_t)

def convert_lqr_gain_matrix(fbs_lqr_gain_matrix):
    t = fbs_lqr_gain_matrix._tab
    return np.ma.frombuffer(t.Bytes[t.Pos : t.Pos +
        (npt.input_size * npt.state_size)*fbfloat64.bytewidth],
        dtype=npt.lqr_gain_matrix_t)

# flatbuffers table converters
def convert_bicycle(fbs_bicycle):
    if fbs_bicycle is None:
        return mr.mrecarray(1, dtype=npt.bicycle_t, mask=True)

    bicycle = mr.mrecarray(1, dtype=npt.bicycle_t)
    set_masked_record_field(bicycle.v, fbs_bicycle.V, masked_value=0)
    set_masked_record_field(bicycle.dt, fbs_bicycle.Dt, masked_value=0)
    set_masked_record_field(bicycle.M, fbs_bicycle.M, convert_second_order_matrix)
    set_masked_record_field(bicycle.C1, fbs_bicycle.C1, convert_second_order_matrix)
    set_masked_record_field(bicycle.K0, fbs_bicycle.K0, convert_second_order_matrix)
    set_masked_record_field(bicycle.K2, fbs_bicycle.K2, convert_second_order_matrix)
    return bicycle

def convert_kalman(fbs_kalman):
    if fbs_kalman is None:
        return mr.mrecarray(1, dtype=npt.kalman_t, mask=True)

    kalman = mr.mrecarray(1, dtype=npt.kalman_t)
    set_masked_record_field(kalman.x, fbs_kalman.StateEstimate, convert_state)
    set_masked_record_field(kalman.P, fbs_kalman.HorizonCost, convert_symmetric_state_matrix)
    set_masked_record_field(kalman.Q, fbs_kalman.StateCost, convert_symmetric_state_matrix)
    set_masked_record_field(kalman.R, fbs_kalman.InputCost, convert_symmetric_output_matrix)
    set_masked_record_field(kalman.K, fbs_kalman.KalmanGain, convert_kalman_gain_matrix)
    return kalman

def convert_lqr(fbs_lqr):
    if fbs_lqr is None:
        return mr.mrecarray(1, dtype=npt.lqr_t, mask=True)

    lqr = mr.mrecarray(1, dtype=npt.lqr_t)
    set_masked_record_field(lqr.n, fbs_lqr.Horizon, masked_value=0)
    set_masked_record_field(lqr.r, fbs_lqr.StateTarget, convert_state)
    set_masked_record_field(lqr.Q, fbs_lqr.StateCost, convert_symmetric_state_matrix)
    set_masked_record_field(lqr.R, fbs_lqr.InputCost, convert_symmetric_input_matrix)
    set_masked_record_field(lqr.P, fbs_lqr.HorizonCost, convert_symmetric_state_matrix)
    set_masked_record_field(lqr.K, fbs_lqr.LqrGain, convert_lqr_gain_matrix)
    return lqr

def convert_sample(fbs_sample):
    # numpy only supports simple (non-nested) masked records
    sample = types.SimpleNamespace()
    for t in npt.sample_t:
        name = t[0]
        datatype = t[1:]
        if type(datatype[0]) is list:
            setattr(sample, name, mr.mrecarray(1, dtype=datatype[0], mask=True))
        else:
            if len(datatype) == 1:
                datatype = datatype[0]
            setattr(sample, name, np.ma.zeros(1, dtype=datatype))
            getattr(sample, name).mask = True

    if fbs_sample is None:
        return sample

    sample.ts[0] = fbs_sample.Timestamp() # assume always serialized
    set_masked_record_field(sample.bicycle, fbs_sample.Bicycle, convert_bicycle)
    set_masked_record_field(sample.kalman, fbs_sample.Kalman, convert_kalman)
    set_masked_record_field(sample.lqr, fbs_sample.Lqr, convert_lqr)
    set_masked_record_field(sample.x, fbs_sample.State, convert_state)
    set_masked_record_field(sample.u, fbs_sample.Input, convert_input)
    set_masked_record_field(sample.y, fbs_sample.Output, convert_output)
    set_masked_record_field(sample.z, fbs_sample.Measurement, convert_output)
    return sample


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(("Usage: {} <sample_log_file>\n\nPrint sample log " +
               "data.").format(__file__));
        sys.exit(1)

    samples = load_sample_log(sys.argv[1])
    for s in samples:
        sample = convert_sample(s)
        print(sample)
    sys.exit(0)
