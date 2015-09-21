# automatically generated, do not modify

# namespace: fbs

import flatbuffers

class SampleLog(object):
    __slots__ = ['_tab']

    @classmethod
    def GetRootAsSampleLog(cls, buf, offset):
        n = flatbuffers.encode.Get(flatbuffers.packer.uoffset, buf, offset)
        x = SampleLog()
        x.Init(buf, n + offset)
        return x


    # SampleLog
    def Init(self, buf, pos):
        self._tab = flatbuffers.table.Table(buf, pos)

    # SampleLog
    def Samples(self, j):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(4))
        if o != 0:
            x = self._tab.Vector(o)
            x += flatbuffers.number_types.UOffsetTFlags.py_type(j) * 4
            x = self._tab.Indirect(x)
            from .SampleBuffer import SampleBuffer
            obj = SampleBuffer()
            obj.Init(self._tab.Bytes, x)
            return obj
        return None

    # SampleLog
    def SamplesLength(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(4))
        if o != 0:
            return self._tab.VectorLen(o)
        return 0

def SampleLogStart(builder): builder.StartObject(1)
def SampleLogAddSamples(builder, samples): builder.PrependUOffsetTRelativeSlot(0, flatbuffers.number_types.UOffsetTFlags.py_type(samples), 0)
def SampleLogStartSamplesVector(builder, numElems): return builder.StartVector(4, numElems, 4)
def SampleLogEnd(builder): return builder.EndObject()
