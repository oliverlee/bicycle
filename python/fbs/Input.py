# automatically generated, do not modify

# namespace: fbs

import flatbuffers

class Input(object):
    __slots__ = ['_tab']

    # Input
    def Init(self, buf, pos):
        self._tab = flatbuffers.table.Table(buf, pos)

    # Input
    def U0(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(0))
    # Input
    def U1(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(8))

def CreateInput(builder, u0, u1):
    builder.Prep(8, 16)
    builder.PrependFloat64(u1)
    builder.PrependFloat64(u0)
    return builder.Offset()
