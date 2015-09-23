# automatically generated, do not modify

# namespace: fbs

import flatbuffers

class SecondOrderMatrix(object):
    __slots__ = ['_tab']

    # SecondOrderMatrix
    def Init(self, buf, pos):
        self._tab = flatbuffers.table.Table(buf, pos)

    # SecondOrderMatrix
    def M00(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(0))
    # SecondOrderMatrix
    def M01(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(8))
    # SecondOrderMatrix
    def M10(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(16))
    # SecondOrderMatrix
    def M11(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(24))

def CreateSecondOrderMatrix(builder, m00, m01, m10, m11):
    builder.Prep(8, 32)
    builder.PrependFloat64(m11)
    builder.PrependFloat64(m10)
    builder.PrependFloat64(m01)
    builder.PrependFloat64(m00)
    return builder.Offset()
