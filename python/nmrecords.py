import numpy as np
from numpy.ma.mrecords import MaskedRecords
from np_types import flatten_dtype


class NestedMaskedRecords(MaskedRecords):
    def __new__(cls, shape, dtype=None, buf=None, offset=0, strides=None,
                formats=None, names=None, titles=None, byteorder=None,
                aligned=False, mask=np.ma.nomask, hard_mask=False,
                fill_value=None, keep_mask=True, copy=False, **options):
        flattened_dtype = flatten_dtype(dtype)
        self = MaskedRecords.__new__(cls, shape, dtype=flattened_dtype, buf=buf,
                                 offset=offset, strides=strides,
                                 formats=formats, names=names,
                                 titles=titles, byteorder=byteorder,
                                 aligned=aligned, mask=mask,
                                 hard_mask=hard_mask, keep_mask=keep_mask,
                                 copy=copy, **options)
        if fill_value is not None:
            self._fill_value = fill_value
        return self

    def __getattr__(self, attr):
        attributes = [name for name in self.dtype.names
                      if name.startswith(attr + '.')]
        if attributes:
            return self._subfield_view(attributes)
        raise AttributeError("'{}' object has no attribute '{}'".format(
            self.__class__.__name__, attr))

    def __getitem__(self, index):
        obj = super().__getitem__(index)
        obj._fill_value = np.array(self.fill_value, dtype=obj.dtype)
        obj = obj.view(nmrecarray)
        return obj

    def __setitem__(self, index, value):
        MaskedRecords.__setitem__(self, indx, value)

    def _subfield_view(self, fields):
        ndtype_dict = {}
        nmdtype_dict = {}
        for name in fields:
            subname = name.split('.', 1)[1]
            ndtype_dict[subname] = self.dtype.fields[name]
            nmdtype_dict[subname] = self._mask.dtype.fields[name]
        ndtype = np.dtype(ndtype_dict)
        fill_value = np.array(self.fill_value, dtype=ndtype)
        obj = self.__class__(self.shape, dtype=ndtype, buf=self, offset=0,
                strides=self.strides, fill_value=fill_value)

        # set subview mask to be a view of self mask
        nmdtype = np.dtype(nmdtype_dict)
        mask = np.ndarray(self._mask.shape, nmdtype, self._mask, 0,
                self._mask.strides)
        obj._mask = mask
        return obj

nmrecarray = NestedMaskedRecords
