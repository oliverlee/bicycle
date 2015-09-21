# automatically generated, do not modify

# namespace: fbs

import flatbuffers

class Sample(object):
    __slots__ = ['_tab']

    @classmethod
    def GetRootAsSample(cls, buf, offset):
        n = flatbuffers.encode.Get(flatbuffers.packer.uoffset, buf, offset)
        x = Sample()
        x.Init(buf, n + offset)
        return x


    # Sample
    def Init(self, buf, pos):
        self._tab = flatbuffers.table.Table(buf, pos)

    # Sample
    def Timestamp(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(4))
        if o != 0:
            return self._tab.Get(flatbuffers.number_types.Uint32Flags, o + self._tab.Pos)
        return 0

    # Sample
    def Bicycle(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(6))
        if o != 0:
            x = self._tab.Indirect(o + self._tab.Pos)
            from .Bicycle import Bicycle
            obj = Bicycle()
            obj.Init(self._tab.Bytes, x)
            return obj
        return None

    # Sample
    def Kalman(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(8))
        if o != 0:
            x = self._tab.Indirect(o + self._tab.Pos)
            from .Kalman import Kalman
            obj = Kalman()
            obj.Init(self._tab.Bytes, x)
            return obj
        return None

    # Sample
    def Lqr(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(10))
        if o != 0:
            x = self._tab.Indirect(o + self._tab.Pos)
            from .Lqr import Lqr
            obj = Lqr()
            obj.Init(self._tab.Bytes, x)
            return obj
        return None

    # Sample
    def State(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(12))
        if o != 0:
            x = o + self._tab.Pos
            from .State import State
            obj = State()
            obj.Init(self._tab.Bytes, x)
            return obj
        return None

    # Sample
    def Input(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(14))
        if o != 0:
            x = o + self._tab.Pos
            from .Input import Input
            obj = Input()
            obj.Init(self._tab.Bytes, x)
            return obj
        return None

    # Sample
    def Output(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(16))
        if o != 0:
            x = o + self._tab.Pos
            from .Output import Output
            obj = Output()
            obj.Init(self._tab.Bytes, x)
            return obj
        return None

    # Sample
    def Measurement(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(18))
        if o != 0:
            x = o + self._tab.Pos
            from .Output import Output
            obj = Output()
            obj.Init(self._tab.Bytes, x)
            return obj
        return None

def SampleStart(builder): builder.StartObject(8)
def SampleAddTimestamp(builder, timestamp): builder.PrependUint32Slot(0, timestamp, 0)
def SampleAddBicycle(builder, bicycle): builder.PrependUOffsetTRelativeSlot(1, flatbuffers.number_types.UOffsetTFlags.py_type(bicycle), 0)
def SampleAddKalman(builder, kalman): builder.PrependUOffsetTRelativeSlot(2, flatbuffers.number_types.UOffsetTFlags.py_type(kalman), 0)
def SampleAddLqr(builder, lqr): builder.PrependUOffsetTRelativeSlot(3, flatbuffers.number_types.UOffsetTFlags.py_type(lqr), 0)
def SampleAddState(builder, state): builder.PrependStructSlot(4, flatbuffers.number_types.UOffsetTFlags.py_type(state), 0)
def SampleAddInput(builder, input): builder.PrependStructSlot(5, flatbuffers.number_types.UOffsetTFlags.py_type(input), 0)
def SampleAddOutput(builder, output): builder.PrependStructSlot(6, flatbuffers.number_types.UOffsetTFlags.py_type(output), 0)
def SampleAddMeasurement(builder, measurement): builder.PrependStructSlot(7, flatbuffers.number_types.UOffsetTFlags.py_type(measurement), 0)
def SampleEnd(builder): return builder.EndObject()
