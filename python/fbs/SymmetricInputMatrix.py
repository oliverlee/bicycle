# automatically generated, do not modify

# namespace: fbs

import flatbuffers

class SymmetricInputMatrix(object):
    __slots__ = ['_tab']

    # SymmetricInputMatrix
    def Init(self, buf, pos):
        self._tab = flatbuffers.table.Table(buf, pos)

    # SymmetricInputMatrix
    def R00(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(0))
    # SymmetricInputMatrix
    def R01(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(8))
    # SymmetricInputMatrix
    def R11(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(16))

def CreateSymmetricInputMatrix(builder, r00, r01, r11):
    builder.Prep(8, 24)
    builder.PrependFloat64(r11)
    builder.PrependFloat64(r01)
    builder.PrependFloat64(r00)
    return builder.Offset()
