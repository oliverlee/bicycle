# automatically generated, do not modify

# namespace: fbs

import flatbuffers

class Lqr(object):
    __slots__ = ['_tab']

    # Lqr
    def Init(self, buf, pos):
        self._tab = flatbuffers.table.Table(buf, pos)

    # Lqr
    def Horizon(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(4))
        if o != 0:
            return self._tab.Get(flatbuffers.number_types.Uint32Flags, o + self._tab.Pos)
        return 0

    # Lqr
    def Reference(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(6))
        if o != 0:
            x = o + self._tab.Pos
            from .State import State
            obj = State()
            obj.Init(self._tab.Bytes, x)
            return obj
        return None

    # Lqr
    def StateCost(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(8))
        if o != 0:
            x = o + self._tab.Pos
            from .SymmetricStateMatrix import SymmetricStateMatrix
            obj = SymmetricStateMatrix()
            obj.Init(self._tab.Bytes, x)
            return obj
        return None

    # Lqr
    def InputCost(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(10))
        if o != 0:
            x = o + self._tab.Pos
            from .SymmetricInputMatrix import SymmetricInputMatrix
            obj = SymmetricInputMatrix()
            obj.Init(self._tab.Bytes, x)
            return obj
        return None

    # Lqr
    def HorizonCost(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(12))
        if o != 0:
            x = o + self._tab.Pos
            from .SymmetricStateMatrix import SymmetricStateMatrix
            obj = SymmetricStateMatrix()
            obj.Init(self._tab.Bytes, x)
            return obj
        return None

    # Lqr
    def LqrGain(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(14))
        if o != 0:
            x = o + self._tab.Pos
            from .LqrGainMatrix import LqrGainMatrix
            obj = LqrGainMatrix()
            obj.Init(self._tab.Bytes, x)
            return obj
        return None

def LqrStart(builder): builder.StartObject(6)
def LqrAddHorizon(builder, horizon): builder.PrependUint32Slot(0, horizon, 0)
def LqrAddReference(builder, reference): builder.PrependStructSlot(1, flatbuffers.number_types.UOffsetTFlags.py_type(reference), 0)
def LqrAddStateCost(builder, stateCost): builder.PrependStructSlot(2, flatbuffers.number_types.UOffsetTFlags.py_type(stateCost), 0)
def LqrAddInputCost(builder, inputCost): builder.PrependStructSlot(3, flatbuffers.number_types.UOffsetTFlags.py_type(inputCost), 0)
def LqrAddHorizonCost(builder, horizonCost): builder.PrependStructSlot(4, flatbuffers.number_types.UOffsetTFlags.py_type(horizonCost), 0)
def LqrAddLqrGain(builder, lqrGain): builder.PrependStructSlot(5, flatbuffers.number_types.UOffsetTFlags.py_type(lqrGain), 0)
def LqrEnd(builder): return builder.EndObject()
