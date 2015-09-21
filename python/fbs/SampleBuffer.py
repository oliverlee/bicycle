# automatically generated, do not modify

# namespace: fbs

import flatbuffers

class SampleBuffer(object):
    __slots__ = ['_tab']

    # SampleBuffer
    def Init(self, buf, pos):
        self._tab = flatbuffers.table.Table(buf, pos)

    # SampleBuffer
    def Data(self, j):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(4))
        if o != 0:
            a = self._tab.Vector(o)
            return self._tab.Get(flatbuffers.number_types.Uint8Flags, a + flatbuffers.number_types.UOffsetTFlags.py_type(j * 1))
        return 0

    # SampleBuffer
    def DataLength(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(4))
        if o != 0:
            return self._tab.VectorLen(o)
        return 0

def SampleBufferStart(builder): builder.StartObject(1)
def SampleBufferAddData(builder, data): builder.PrependUOffsetTRelativeSlot(0, flatbuffers.number_types.UOffsetTFlags.py_type(data), 0)
def SampleBufferStartDataVector(builder, numElems): return builder.StartVector(1, numElems, 1)
def SampleBufferEnd(builder): return builder.EndObject()
