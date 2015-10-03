# automatically generated, do not modify

# namespace: fbs

import flatbuffers

class State(object):
    __slots__ = ['_tab']

    # State
    def Init(self, buf, pos):
        self._tab = flatbuffers.table.Table(buf, pos)

    # State
    def X0(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(0))
    # State
    def X1(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(8))
    # State
    def X2(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(16))
    # State
    def X3(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(24))
    # State
    def X4(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(32))

def CreateState(builder, x0, x1, x2, x3, x4):
    builder.Prep(8, 40)
    builder.PrependFloat64(x4)
    builder.PrependFloat64(x3)
    builder.PrependFloat64(x2)
    builder.PrependFloat64(x1)
    builder.PrependFloat64(x0)
    return builder.Offset()
