# automatically generated, do not modify

# namespace: fbs

import flatbuffers

class Output(object):
    __slots__ = ['_tab']

    # Output
    def Init(self, buf, pos):
        self._tab = flatbuffers.table.Table(buf, pos)

    # Output
    def Y0(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(0))
    # Output
    def Y1(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(8))

def CreateOutput(builder, y0, y1):
    builder.Prep(8, 16)
    builder.PrependFloat64(y1)
    builder.PrependFloat64(y0)
    return builder.Offset()
