# automatically generated, do not modify

# namespace: fbs

import flatbuffers

class AuxiliaryState(object):
    __slots__ = ['_tab']

    # AuxiliaryState
    def Init(self, buf, pos):
        self._tab = flatbuffers.table.Table(buf, pos)

    # AuxiliaryState
    def X0(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(0))
    # AuxiliaryState
    def X1(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(8))
    # AuxiliaryState
    def X2(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(16))

def CreateAuxiliaryState(builder, x0, x1, x2):
    builder.Prep(8, 24)
    builder.PrependFloat64(x2)
    builder.PrependFloat64(x1)
    builder.PrependFloat64(x0)
    return builder.Offset()
