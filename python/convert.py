#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
from flatbuffers.number_types import Float64Flags as fbfloat64
from flatbuffers.number_types import UOffsetTFlags as fbuoffset
from fbs.SampleLog import SampleLog as FbsSampleLog
from fbs.Sample import Sample as FbsSample
import nptypes as npt
from nmrecords import nmrecarray


def get_fbs_sample(fbs_sample_log, index):
    sb = fbs_sample_log.Samples(index)
    o = fbuoffset.py_type(sb._tab.Offset(4)) # only field in fbs_samplebuffer
    a = sb._tab.Vector(o)
    return FbsSample.GetRootAsSample(sb._tab.Bytes[a : a+sb.DataLength()], 0)


def load_sample_log(filename):
    with open(filename, 'rb') as f:
        buf = f.read()
    log = FbsSampleLog.GetRootAsSampleLog(bytearray(buf), 0)
    rec_samples = nmrecarray(log.SamplesLength(), dtype=npt.sample_t)
    for i in range(len(rec_samples)):
        convert_record_subfield(rec_samples[i : i+1], get_fbs_sample(log, i),
                                convert_sample)
    return rec_samples


def convert_record_subfield(subfield, flatbuf, convert_func=None, default=0):
    if convert_func is None:
        # flatbuf value is a scalar value
        if flatbuf == default:
            subfield[0] = np.ma.masked
        else:
            subfield[0] = flatbuf
    else:
        # flatbuf is a struct of table
        if flatbuf is None:
            subfield[0] = np.ma.masked
        else:
            convert_func(subfield, flatbuf)


## flatbuffers table converters
def convert_sample(rec_sample, fbs_sample):
    rec_sample.ts[0] = fbs_sample.Timestamp() # assume always serialized
    convert_record_subfield(rec_sample.bicycle, fbs_sample.Bicycle(),
                            convert_bicycle)
    convert_record_subfield(rec_sample.kalman, fbs_sample.Kalman(),
                            convert_kalman)
    convert_record_subfield(rec_sample.lqr, fbs_sample.Lqr(),
                            convert_lqr)
    convert_record_subfield(rec_sample.x, fbs_sample.State(),
                            convert_state)
    convert_record_subfield(rec_sample.u, fbs_sample.Input(),
                            convert_input)
    convert_record_subfield(rec_sample.y, fbs_sample.Output(),
                            convert_output)
    convert_record_subfield(rec_sample.z, fbs_sample.Measurement(),
                            convert_output)


def convert_bicycle(rec_bicycle, fbs_bicycle):
    # v must be nonzero than zero
    # dt must be greater than zero
    convert_record_subfield(rec_bicycle.v, fbs_bicycle.V())
    convert_record_subfield(rec_bicycle.dt, fbs_bicycle.Dt())
    convert_record_subfield(rec_bicycle.M, fbs_bicycle.M(),
                            convert_second_order_matrix)
    convert_record_subfield(rec_bicycle.C1, fbs_bicycle.C1(),
                            convert_second_order_matrix)
    convert_record_subfield(rec_bicycle.K0, fbs_bicycle.K0(),
                            convert_second_order_matrix)
    convert_record_subfield(rec_bicycle.K2, fbs_bicycle.K2(),
                            convert_second_order_matrix)


def convert_kalman(rec_kalman, fbs_kalman):
    convert_record_subfield(rec_kalman.x, fbs_kalman.StateEstimate(),
                            convert_state)
    convert_record_subfield(rec_kalman.P, fbs_kalman.ErrorCovariance(),
                            convert_symmetric_state_matrix)
    convert_record_subfield(rec_kalman.Q, fbs_kalman.ProcessNoiseCovariance(),
                            convert_symmetric_state_matrix)
    convert_record_subfield(rec_kalman.R,
                            fbs_kalman.MeasurementNoiseCovariance(),
                            convert_symmetric_output_matrix)
    convert_record_subfield(rec_kalman.K, fbs_kalman.KalmanGain(),
                            convert_kalman_gain_matrix)


def convert_lqr(rec_lqr, fbs_lqr):
    # n must be greater than zero
    convert_record_subfield(rec_lqr.n, fbs_lqr.Horizon())
    convert_record_subfield(rec_lqr.r, fbs_lqr.StateTarget(), convert_state)
    convert_record_subfield(rec_lqr.Q, fbs_lqr.StateCost(),
                            convert_symmetric_state_matrix)
    convert_record_subfield(rec_lqr.R, fbs_lqr.InputCost(),
                            convert_symmetric_input_matrix)
    convert_record_subfield(rec_lqr.P, fbs_lqr.HorizonCost(),
                            convert_symmetric_state_matrix)
    convert_record_subfield(rec_lqr.K, fbs_lqr.LqrGain(),
                            convert_lqr_gain_matrix)


## flatbuffers struct converters
def convert_state(rec_field, fbs_state):
    t = fbs_state._tab
    ma = np.ma.frombuffer(t.Bytes[t.Pos : t.Pos +
        npt.state_size*fbfloat64.bytewidth], dtype=npt.state_t)
    rec_field[0] = ma


def convert_input(rec_field, fbs_input):
    t = fbs_input._tab
    ma = np.ma.frombuffer(t.Bytes[t.Pos : t.Pos +
        npt.input_size*fbfloat64.bytewidth], dtype=npt.input_t)
    rec_field[0] = ma


def convert_output(rec_field, fbs_output):
    t = fbs_output._tab
    ma = np.ma.frombuffer(t.Bytes[t.Pos : t.Pos +
        npt.output_size*fbfloat64.bytewidth], dtype=npt.output_t)
    rec_field[0] = ma


def convert_second_order_matrix(rec_field, fbs_second_order_matrix):
    t = fbs_second_order_matrix._tab
    ma = np.ma.frombuffer(t.Bytes[t.Pos : t.Pos +
        (npt.second_order_size**2)*fbfloat64.bytewidth],
        dtype=npt.second_order_matrix_t)
    rec_field[0] = ma


def convert_symmetric_state_matrix(rec_field, fbs_symmetric_state_matrix):
    t = fbs_symmetric_state_matrix._tab
    a = np.ma.frombuffer(t.Bytes[t.Pos : t.Pos +
        (npt.state_size*(npt.state_size + 1)//2)*fbfloat64.bytewidth],
        dtype=npt.real_t)
    b = np.ma.zeros([], dtype=npt.symmetric_state_matrix_t)
    b[np.triu_indices(npt.state_size)] = a
    ma = b + np.triu(b, 1).T
    rec_field[0] = ma


def convert_symmetric_input_matrix(rec_field, fbs_symmetric_input_matrix):
    t = fbs_symmetric_input_matrix._tab
    a = np.ma.frombuffer(t.Bytes[t.Pos : t.Pos +
        (npt.input_size*(npt.input_size + 1)//2)*fbfloat64.bytewidth],
        dtype=npt.real_t)
    b = np.ma.zeros([], dtype=npt.symmetric_input_matrix_t)
    b[np.triu_indices(npt.input_size)] = a
    ma = b + np.triu(b, 1).T
    rec_field[0] = ma


def convert_symmetric_output_matrix(rec_field, fbs_symmetric_output_matrix):
    t = fbs_symmetric_output_matrix._tab
    a = np.ma.frombuffer(t.Bytes[t.Pos : t.Pos +
        (npt.output_size*(npt.output_size + 1)//2)*fbfloat64.bytewidth],
        dtype=npt.real_t)
    b = np.ma.zeros([], dtype=npt.symmetric_output_matrix_t)
    b[np.triu_indices(npt.output_size)] = a
    ma = b + np.triu(b, 1).T
    rec_field[0] = ma


def convert_kalman_gain_matrix(rec_field, fbs_kalman_gain_matrix):
    t = fbs_kalman_gain_matrix._tab
    ma = np.ma.frombuffer(t.Bytes[t.Pos : t.Pos +
        (npt.state_size * npt.output_size)*fbfloat64.bytewidth],
        dtype=npt.kalman_gain_matrix_t)
    rec_field[0] = ma


def convert_lqr_gain_matrix(rec_field, fbs_lqr_gain_matrix):
    t = fbs_lqr_gain_matrix._tab
    ma = np.ma.frombuffer(t.Bytes[t.Pos : t.Pos +
        (npt.input_size * npt.state_size)*fbfloat64.bytewidth],
        dtype=npt.lqr_gain_matrix_t)
    rec_field[0] = ma
