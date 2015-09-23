# automatically generated, do not modify

# namespace: fbs

import flatbuffers

class KalmanGainMatrix(object):
    __slots__ = ['_tab']

    # KalmanGainMatrix
    def Init(self, buf, pos):
        self._tab = flatbuffers.table.Table(buf, pos)

    # KalmanGainMatrix
    def K00(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(0))
    # KalmanGainMatrix
    def K01(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(8))
    # KalmanGainMatrix
    def K10(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(16))
    # KalmanGainMatrix
    def K11(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(24))
    # KalmanGainMatrix
    def K20(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(32))
    # KalmanGainMatrix
    def K21(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(40))
    # KalmanGainMatrix
    def K30(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(48))
    # KalmanGainMatrix
    def K31(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(56))

def CreateKalmanGainMatrix(builder, k00, k01, k10, k11, k20, k21, k30, k31):
    builder.Prep(8, 64)
    builder.PrependFloat64(k31)
    builder.PrependFloat64(k30)
    builder.PrependFloat64(k21)
    builder.PrependFloat64(k20)
    builder.PrependFloat64(k11)
    builder.PrependFloat64(k10)
    builder.PrependFloat64(k01)
    builder.PrependFloat64(k00)
    return builder.Offset()
