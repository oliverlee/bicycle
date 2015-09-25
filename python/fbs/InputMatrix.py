# automatically generated, do not modify

# namespace: fbs

import flatbuffers

class InputMatrix(object):
    __slots__ = ['_tab']

    # InputMatrix
    def Init(self, buf, pos):
        self._tab = flatbuffers.table.Table(buf, pos)

    # InputMatrix
    def B00(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(0))
    # InputMatrix
    def B01(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(8))
    # InputMatrix
    def B10(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(16))
    # InputMatrix
    def B11(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(24))
    # InputMatrix
    def B20(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(32))
    # InputMatrix
    def B21(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(40))
    # InputMatrix
    def B30(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(48))
    # InputMatrix
    def B31(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(56))

def CreateInputMatrix(builder, b00, b01, b10, b11, b20, b21, b30, b31):
    builder.Prep(8, 64)
    builder.PrependFloat64(b31)
    builder.PrependFloat64(b30)
    builder.PrependFloat64(b21)
    builder.PrependFloat64(b20)
    builder.PrependFloat64(b11)
    builder.PrependFloat64(b10)
    builder.PrependFloat64(b01)
    builder.PrependFloat64(b00)
    return builder.Offset()
