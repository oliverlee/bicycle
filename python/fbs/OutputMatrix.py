# automatically generated, do not modify

# namespace: fbs

import flatbuffers

class OutputMatrix(object):
    __slots__ = ['_tab']

    # OutputMatrix
    def Init(self, buf, pos):
        self._tab = flatbuffers.table.Table(buf, pos)

    # OutputMatrix
    def C00(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(0))
    # OutputMatrix
    def C01(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(8))
    # OutputMatrix
    def C02(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(16))
    # OutputMatrix
    def C03(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(24))
    # OutputMatrix
    def C10(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(32))
    # OutputMatrix
    def C11(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(40))
    # OutputMatrix
    def C12(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(48))
    # OutputMatrix
    def C13(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(56))

def CreateOutputMatrix(builder, c00, c01, c02, c03, c10, c11, c12, c13):
    builder.Prep(8, 64)
    builder.PrependFloat64(c13)
    builder.PrependFloat64(c12)
    builder.PrependFloat64(c11)
    builder.PrependFloat64(c10)
    builder.PrependFloat64(c03)
    builder.PrependFloat64(c02)
    builder.PrependFloat64(c01)
    builder.PrependFloat64(c00)
    return builder.Offset()
