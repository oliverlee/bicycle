#!/usr/in/env python
# -*- coding: utf-8 -*-
import numpy as np
from flatbuffers.number_types import Float64Flags as fbfloat64
from flatbuffers.number_types import UOffsetTFlags as fbuoffset
import np_types as npt
from fbs.SampleLog import SampleLog as FbsSampleLog
from fbs.Sample import Sample as FbsSample


# flatbuffers struct converters
def convert_state(fbs_state, rec_field, index):
    t = fbs_state._tab
    ma = np.ma.frombuffer(t.Bytes[t.Pos : t.Pos +
        npt.state_size*fbfloat64.bytewidth], dtype=npt.state_t)
    rec_field[index] = ma
    return None

def convert_input(fbs_input, rec_field, index):
    t = fbs_input._tab
    ma = np.ma.frombuffer(t.Bytes[t.Pos : t.Pos +
        npt.input_size*fbfloat64.bytewidth], dtype=npt.input_t)
    rec_field[index] = ma
    return None

def convert_output(fbs_output, rec_field, index):
    t = fbs_output._tab
    ma = np.ma.frombuffer(t.Bytes[t.Pos : t.Pos +
        npt.output_size*fbfloat64.bytewidth], dtype=npt.output_t)
    rec_field[index] = ma
    return None

def convert_second_order_matrix(fbs_second_order_matrix, rec_field, index):
    t = fbs_second_order_matrix._tab
    ma = np.ma.frombuffer(t.Bytes[t.Pos : t.Pos +
        (npt.second_order_size**2)*fbfloat64.bytewidth],
        dtype=npt.second_order_matrix_t)
    rec_field[index] = ma
    return None

def convert_symmetric_state_matrix(fbs_symmetric_state_matrix, rec_field, index):
    t = fbs_symmetric_state_matrix._tab
    a = np.ma.frombuffer(t.Bytes[t.Pos : t.Pos +
        (npt.state_size*(npt.state_size + 1)/2)*fbfloat64.bytewidth],
        dtype=npt.real_t)
    b = np.ma.zeros([], dtype=npt.symmetric_state_matrix_t)
    b[np.triu_indices(npt.state_size)] = a
    ma = b + np.triu(b, 1).T
    rec_field[index] = ma
    return None

def convert_symmetric_input_matrix(fbs_symmetric_input_matrix, rec_field, index):
    t = fbs_symmetric_input_matrix._tab
    a = np.ma.frombuffer(t.Bytes[t.Pos : t.Pos +
        (npt.input_size*(npt.input_size + 1)/2)*fbfloat64.bytewidth],
        dtype=npt.real_t)
    b = np.ma.zeros([], dtype=npt.symmetric_input_matrix_t)
    b[np.triu_indices(npt.input_size)] = a
    ma = b + np.triu(b, 1).T
    rec_field[index] = ma
    return None

def convert_symmetric_output_matrix(fbs_symmetric_output_matrix, rec_field, index):
    t = fbs_symmetric_output_matrix._tab
    a = np.ma.frombuffer(t.Bytes[t.Pos : t.Pos +
        (npt.output_size*(npt.output_size + 1)/2)*fbfloat64.bytewidth],
        dtype=npt.real_t)
    b = np.ma.zeros([], dtype=npt.symmetric_output_matrix_t)
    b[np.triu_indices(npt.output_size)] = a
    ma = b + np.triu(b, 1).T
    rec_field[index] = ma
    return None

def convert_kalman_gain_matrix(fbs_kalman_gain_matrix, rec_field, index):
    t = fbs_kalman_gain_matrix._tab
    ma = np.ma.frombuffer(t.Bytes[t.Pos : t.Pos +
        (npt.state_size * npt.output_size)*fbfloat64.bytewidth],
        dtype=npt.kalman_gain_matrix_t)
    rec_field[index] = ma
    return None

def convert_lqr_gain_matrix(fbs_lqr_gain_matrix, rec_field, index):
    t = fbs_lqr_gain_matrix._tab
    ma = np.ma.frombuffer(t.Bytes[t.Pos : t.Pos +
        (npt.input_size * npt.state_size)*fbfloat64.bytewidth],
        dtype=npt.lqr_gain_matrix_t)
    rec_field[index] = ma
    return None

# flatbuffers table converters
def convert_bicycle(fbs_bicycle, rec_bicycle, index):
    if fbs_bicycle is None:
        rec_bicycle[index] = np.ma.masked
        return None
    set_record_field(rec_bicycle.v, index, fbs_bicycle.V)
    set_record_field(rec_bicycle.dt, index, fbs_bicycle.Dt)
    set_record_field(rec_bicycle.M, index, fbs_bicycle.M, convert_second_order_matrix)
    set_record_field(rec_bicycle.C1, index, fbs_bicycle.C1, convert_second_order_matrix)
    set_record_field(rec_bicycle.K0, index, fbs_bicycle.K0, convert_second_order_matrix)
    set_record_field(rec_bicycle.K2, index, fbs_bicycle.K2, convert_second_order_matrix)
    return None

def convert_kalman(fbs_kalman, rec_kalman, index):
    if fbs_kalman is None:
        rec_kalman[index] = np.ma.masked
        return None
    set_record_field(rec_kalman.x, index, fbs_kalman.StateEstimate, convert_state)
    set_record_field(rec_kalman.P, index, fbs_kalman.HorizonCost, convert_symmetric_state_matrix)
    set_record_field(rec_kalman.Q, index, fbs_kalman.StateCost, convert_symmetric_state_matrix)
    set_record_field(rec_kalman.R, index, fbs_kalman.InputCost, convert_symmetric_output_matrix)
    set_record_field(rec_kalman.K, index, fbs_kalman.KalmanGain, convert_kalman_gain_matrix)
    return None

def convert_lqr(fbs_lqr, rec_lqr, index):
    if fbs_lqr is None:
        rec_lqr[index] = np.ma.masked
        return None
    set_record_field(rec_lqr.n, index, fbs_lqr.Horizon)
    set_record_field(rec_lqr.r, index, fbs_lqr.StateTarget, convert_state)
    set_record_field(rec_lqr.Q, index, fbs_lqr.StateCost, convert_symmetric_state_matrix)
    set_record_field(rec_lqr.R, index, fbs_lqr.InputCost, convert_symmetric_input_matrix)
    set_record_field(rec_lqr.P, index, fbs_lqr.HorizonCost, convert_symmetric_state_matrix)
    set_record_field(rec_lqr.K, index, fbs_lqr.LqrGain, convert_lqr_gain_matrix)
    return None

def convert_sample(fbs_sample, rec_sample, index):
    if fbs_sample is None:
        rec_sample[index] = np.ma.masked
        return None
    rec_sample.ts[index] = fbs_sample.Timestamp() # assume always serialized
    set_record_field(rec_sample.bicycle, index, fbs_sample.Bicycle, convert_bicycle)
    set_record_field(rec_sample.kalman, index, fbs_sample.Kalman, convert_kalman)
    set_record_field(rec_sample.lqr, index, fbs_sample.Lqr, convert_lqr)
    set_record_field(rec_sample.x, index, fbs_sample.State, convert_state)
    set_record_field(rec_sample.u, index, fbs_sample.Input, convert_input)
    set_record_field(rec_sample.y, index, fbs_sample.Output, convert_output)
    set_record_field(rec_sample.z, index, fbs_sample.Measurement, convert_output)
    return None

def set_record_field(field, index, get_func, convert_func=None):
    value = get_func()
    if convert_func is None:
        # field type is a scalar value
        if value == field[index]:
            # field[index] is initialized to a zero scalar
            # for now, treat zero as the masked value
            # TODO: allow nonzero masked values
            field[index] = np.ma.masked
        else:
            field[index] = value
    else:
        # field type is flatbuffers struct of table
        if value is None:
            field[index] = np.ma.masked
        else:
            convert_func(value, field, index)
    return None

def load_sample_log(filename):
    with open(filename, 'rb') as f:
        buf = f.read()
    log = FbsSampleLog.GetRootAsSampleLog(bytearray(buf), 0)
    length = log.SamplesLength()

    rec_samples = npt.Record(npt.sample_t, length, mask=False)
    for i in range(length):
        sb = log.Samples(i)
        o = fbuoffset.py_type(sb._tab.Offset(4))
        a = sb._tab.Vector(o)
        fbs_sample = FbsSample.GetRootAsSample( sb._tab.Bytes[a : a + sb.DataLength()], 0)
        convert_sample(fbs_sample, rec_samples, i)
    return rec_samples
