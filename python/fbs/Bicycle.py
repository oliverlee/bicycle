# automatically generated, do not modify

# namespace: fbs

import flatbuffers

class Bicycle(object):
    __slots__ = ['_tab']

    # Bicycle
    def Init(self, buf, pos):
        self._tab = flatbuffers.table.Table(buf, pos)

    # Bicycle
    def V(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(4))
        if o != 0:
            return self._tab.Get(flatbuffers.number_types.Float64Flags, o + self._tab.Pos)
        return 0

    # Bicycle
    def Dt(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(6))
        if o != 0:
            return self._tab.Get(flatbuffers.number_types.Float64Flags, o + self._tab.Pos)
        return 0

    # Bicycle
    def M(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(8))
        if o != 0:
            x = o + self._tab.Pos
            from .SecondOrderMatrix import SecondOrderMatrix
            obj = SecondOrderMatrix()
            obj.Init(self._tab.Bytes, x)
            return obj
        return None

    # Bicycle
    def C1(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(10))
        if o != 0:
            x = o + self._tab.Pos
            from .SecondOrderMatrix import SecondOrderMatrix
            obj = SecondOrderMatrix()
            obj.Init(self._tab.Bytes, x)
            return obj
        return None

    # Bicycle
    def K0(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(12))
        if o != 0:
            x = o + self._tab.Pos
            from .SecondOrderMatrix import SecondOrderMatrix
            obj = SecondOrderMatrix()
            obj.Init(self._tab.Bytes, x)
            return obj
        return None

    # Bicycle
    def K2(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(14))
        if o != 0:
            x = o + self._tab.Pos
            from .SecondOrderMatrix import SecondOrderMatrix
            obj = SecondOrderMatrix()
            obj.Init(self._tab.Bytes, x)
            return obj
        return None

    # Bicycle
    def Ad(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(16))
        if o != 0:
            x = o + self._tab.Pos
            from .StateMatrix import StateMatrix
            obj = StateMatrix()
            obj.Init(self._tab.Bytes, x)
            return obj
        return None

    # Bicycle
    def Bd(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(18))
        if o != 0:
            x = o + self._tab.Pos
            from .InputMatrix import InputMatrix
            obj = InputMatrix()
            obj.Init(self._tab.Bytes, x)
            return obj
        return None

    # Bicycle
    def Cd(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(20))
        if o != 0:
            x = o + self._tab.Pos
            from .OutputMatrix import OutputMatrix
            obj = OutputMatrix()
            obj.Init(self._tab.Bytes, x)
            return obj
        return None

    # Bicycle
    def Dd(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(22))
        if o != 0:
            x = o + self._tab.Pos
            from .FeedthroughMatrix import FeedthroughMatrix
            obj = FeedthroughMatrix()
            obj.Init(self._tab.Bytes, x)
            return obj
        return None

def BicycleStart(builder): builder.StartObject(10)
def BicycleAddV(builder, v): builder.PrependFloat64Slot(0, v, 0)
def BicycleAddDt(builder, dt): builder.PrependFloat64Slot(1, dt, 0)
def BicycleAddM(builder, M): builder.PrependStructSlot(2, flatbuffers.number_types.UOffsetTFlags.py_type(M), 0)
def BicycleAddC1(builder, C1): builder.PrependStructSlot(3, flatbuffers.number_types.UOffsetTFlags.py_type(C1), 0)
def BicycleAddK0(builder, K0): builder.PrependStructSlot(4, flatbuffers.number_types.UOffsetTFlags.py_type(K0), 0)
def BicycleAddK2(builder, K2): builder.PrependStructSlot(5, flatbuffers.number_types.UOffsetTFlags.py_type(K2), 0)
def BicycleAddAd(builder, Ad): builder.PrependStructSlot(6, flatbuffers.number_types.UOffsetTFlags.py_type(Ad), 0)
def BicycleAddBd(builder, Bd): builder.PrependStructSlot(7, flatbuffers.number_types.UOffsetTFlags.py_type(Bd), 0)
def BicycleAddCd(builder, Cd): builder.PrependStructSlot(8, flatbuffers.number_types.UOffsetTFlags.py_type(Cd), 0)
def BicycleAddDd(builder, Dd): builder.PrependStructSlot(9, flatbuffers.number_types.UOffsetTFlags.py_type(Dd), 0)
def BicycleEnd(builder): return builder.EndObject()
