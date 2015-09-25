# automatically generated, do not modify

# namespace: fbs

import flatbuffers

class FeedthroughMatrix(object):
    __slots__ = ['_tab']

    # FeedthroughMatrix
    def Init(self, buf, pos):
        self._tab = flatbuffers.table.Table(buf, pos)

    # FeedthroughMatrix
    def D00(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(0))
    # FeedthroughMatrix
    def D01(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(8))
    # FeedthroughMatrix
    def D10(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(16))
    # FeedthroughMatrix
    def D11(self): return self._tab.Get(flatbuffers.number_types.Float64Flags, self._tab.Pos + flatbuffers.number_types.UOffsetTFlags.py_type(24))

def CreateFeedthroughMatrix(builder, d00, d01, d10, d11):
    builder.Prep(8, 32)
    builder.PrependFloat64(d11)
    builder.PrependFloat64(d10)
    builder.PrependFloat64(d01)
    builder.PrependFloat64(d00)
    return builder.Offset()
