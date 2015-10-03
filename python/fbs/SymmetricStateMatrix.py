# automatically generated, do not modify

# namespace: fbs

import flatbuffers

class SymmetricStateMatrix(object):
    __slots__ = ['_tab']

    # SymmetricStateMatrix
    def Init(self, buf, pos):
        self._tab = flatbuffers.table.Table(buf, pos)

    # SymmetricStateMatrix
    def Q00(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(0))
    # SymmetricStateMatrix
    def Q01(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(8))
    # SymmetricStateMatrix
    def Q02(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(16))
    # SymmetricStateMatrix
    def Q03(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(24))
    # SymmetricStateMatrix
    def Q04(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(32))
    # SymmetricStateMatrix
    def Q11(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(40))
    # SymmetricStateMatrix
    def Q12(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(48))
    # SymmetricStateMatrix
    def Q13(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(56))
    # SymmetricStateMatrix
    def Q14(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(64))
    # SymmetricStateMatrix
    def Q22(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(72))
    # SymmetricStateMatrix
    def Q23(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(80))
    # SymmetricStateMatrix
    def Q24(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(88))
    # SymmetricStateMatrix
    def Q33(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(96))
    # SymmetricStateMatrix
    def Q34(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(104))
    # SymmetricStateMatrix
    def Q44(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(112))

def CreateSymmetricStateMatrix(builder, q00, q01, q02, q03, q04, q11, q12, q13, q14, q22, q23, q24, q33, q34, q44):
    builder.Prep(8, 120)
    builder.PrependFloat64(q44)
    builder.PrependFloat64(q34)
    builder.PrependFloat64(q33)
    builder.PrependFloat64(q24)
    builder.PrependFloat64(q23)
    builder.PrependFloat64(q22)
    builder.PrependFloat64(q14)
    builder.PrependFloat64(q13)
    builder.PrependFloat64(q12)
    builder.PrependFloat64(q11)
    builder.PrependFloat64(q04)
    builder.PrependFloat64(q03)
    builder.PrependFloat64(q02)
    builder.PrependFloat64(q01)
    builder.PrependFloat64(q00)
    return builder.Offset()
