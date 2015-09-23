# automatically generated, do not modify

# namespace: fbs

import flatbuffers

class LqrGainMatrix(object):
    __slots__ = ['_tab']

    # LqrGainMatrix
    def Init(self, buf, pos):
        self._tab = flatbuffers.table.Table(buf, pos)

    # LqrGainMatrix
    def K00(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(0))
    # LqrGainMatrix
    def K01(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(8))
    # LqrGainMatrix
    def K02(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(16))
    # LqrGainMatrix
    def K03(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(24))
    # LqrGainMatrix
    def K10(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(32))
    # LqrGainMatrix
    def K11(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(40))
    # LqrGainMatrix
    def K12(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(48))
    # LqrGainMatrix
    def K13(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(56))

def CreateLqrGainMatrix(builder, k00, k01, k02, k03, k10, k11, k12, k13):
    builder.Prep(8, 64)
    builder.PrependFloat64(k13)
    builder.PrependFloat64(k12)
    builder.PrependFloat64(k11)
    builder.PrependFloat64(k10)
    builder.PrependFloat64(k03)
    builder.PrependFloat64(k02)
    builder.PrependFloat64(k01)
    builder.PrependFloat64(k00)
    return builder.Offset()
