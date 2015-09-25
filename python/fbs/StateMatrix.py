# automatically generated, do not modify

# namespace: fbs

import flatbuffers

class StateMatrix(object):
    __slots__ = ['_tab']

    # StateMatrix
    def Init(self, buf, pos):
        self._tab = flatbuffers.table.Table(buf, pos)

    # StateMatrix
    def A00(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(0))
    # StateMatrix
    def A01(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(8))
    # StateMatrix
    def A02(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(16))
    # StateMatrix
    def A03(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(24))
    # StateMatrix
    def A10(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(32))
    # StateMatrix
    def A11(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(40))
    # StateMatrix
    def A12(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(48))
    # StateMatrix
    def A13(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(56))
    # StateMatrix
    def A20(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(64))
    # StateMatrix
    def A21(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(72))
    # StateMatrix
    def A22(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(80))
    # StateMatrix
    def A23(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(88))
    # StateMatrix
    def A30(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(96))
    # StateMatrix
    def A31(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(104))
    # StateMatrix
    def A32(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(112))
    # StateMatrix
    def A33(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(120))

def CreateStateMatrix(builder, a00, a01, a02, a03, a10, a11, a12, a13, a20, a21, a22, a23, a30, a31, a32, a33):
    builder.Prep(8, 128)
    builder.PrependFloat64(a33)
    builder.PrependFloat64(a32)
    builder.PrependFloat64(a31)
    builder.PrependFloat64(a30)
    builder.PrependFloat64(a23)
    builder.PrependFloat64(a22)
    builder.PrependFloat64(a21)
    builder.PrependFloat64(a20)
    builder.PrependFloat64(a13)
    builder.PrependFloat64(a12)
    builder.PrependFloat64(a11)
    builder.PrependFloat64(a10)
    builder.PrependFloat64(a03)
    builder.PrependFloat64(a02)
    builder.PrependFloat64(a01)
    builder.PrependFloat64(a00)
    return builder.Offset()
