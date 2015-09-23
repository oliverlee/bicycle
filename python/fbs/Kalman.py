# automatically generated, do not modify

# namespace: fbs

import flatbuffers

class Kalman(object):
    __slots__ = ['_tab']

    # Kalman
    def Init(self, buf, pos):
        self._tab = flatbuffers.table.Table(buf, pos)

    # Kalman
    def StateEstimate(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(4))
        if o != 0:
            x = o + self._tab.Pos
            from .State import State
            obj = State()
            obj.Init(self._tab.Bytes, x)
            return obj
        return None

    # Kalman
    def ErrorCovariance(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(6))
        if o != 0:
            x = o + self._tab.Pos
            from .SymmetricStateMatrix import SymmetricStateMatrix
            obj = SymmetricStateMatrix()
            obj.Init(self._tab.Bytes, x)
            return obj
        return None

    # Kalman
    def ProcessNoiseCovariance(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(8))
        if o != 0:
            x = o + self._tab.Pos
            from .SymmetricStateMatrix import SymmetricStateMatrix
            obj = SymmetricStateMatrix()
            obj.Init(self._tab.Bytes, x)
            return obj
        return None

    # Kalman
    def MeasurementNoiseCovariance(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(10))
        if o != 0:
            x = o + self._tab.Pos
            from .SymmetricOutputMatrix import SymmetricOutputMatrix
            obj = SymmetricOutputMatrix()
            obj.Init(self._tab.Bytes, x)
            return obj
        return None

    # Kalman
    def KalmanGain(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(12))
        if o != 0:
            x = o + self._tab.Pos
            from .KalmanGainMatrix import KalmanGainMatrix
            obj = KalmanGainMatrix()
            obj.Init(self._tab.Bytes, x)
            return obj
        return None

def KalmanStart(builder): builder.StartObject(5)
def KalmanAddStateEstimate(builder, stateEstimate): builder.PrependStructSlot(0, flatbuffers.number_types.UOffsetTFlags.py_type(stateEstimate), 0)
def KalmanAddErrorCovariance(builder, errorCovariance): builder.PrependStructSlot(1, flatbuffers.number_types.UOffsetTFlags.py_type(errorCovariance), 0)
def KalmanAddProcessNoiseCovariance(builder, processNoiseCovariance): builder.PrependStructSlot(2, flatbuffers.number_types.UOffsetTFlags.py_type(processNoiseCovariance), 0)
def KalmanAddMeasurementNoiseCovariance(builder, measurementNoiseCovariance): builder.PrependStructSlot(3, flatbuffers.number_types.UOffsetTFlags.py_type(measurementNoiseCovariance), 0)
def KalmanAddKalmanGain(builder, kalmanGain): builder.PrependStructSlot(4, flatbuffers.number_types.UOffsetTFlags.py_type(kalmanGain), 0)
def KalmanEnd(builder): return builder.EndObject()
