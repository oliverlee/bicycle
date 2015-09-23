import numpy as np
import numpy.ma.mrecords as mr
from numpy.ma.mrecords import MaskedRecords
from np_types import flatten_dtype


class NestedMaskedRecords(MaskedRecords):
    def __new__(cls, shape, dtype=None, buf=None, offset=0, strides=None,
                formats=None, names=None, titles=None, byteorder=None,
                aligned=False, mask=np.ma.nomask, hard_mask=False,
                fill_value=None, keep_mask=True, copy=False, **options):
        flattened_dtype = flatten_dtype(dtype)
        self = mr.mrecarray.__new__(cls, shape, dtype=flattened_dtype, buf=buf,
                                 offset=offset, strides=strides,
                                 formats=formats, names=names,
                                 titles=titles, byteorder=byteorder,
                                 aligned=aligned,)
        if fill_value is not None:
            self._fill_value = fill_value
        return self

    def __getattribute__(self, attr):
        if attr == 'dtype': # don't want to recurse infinitely
            return super().__getattribute__(attr)

        attributes = [name for name in self.dtype.names
                      if name.startswith(attr + '.')]
        if attributes:
            return self._subfield_view(attributes)
        return super().__getattribute__(attr)

    def _subfield_view(self, fields):
        ndtype_dict = {}
        for name in fields:
            subname = name.split('.', 1)[1]
            ndtype_dict[subname] = self.dtype.fields[name]
        ndtype = np.dtype(ndtype_dict)
        fill_value = np.array(self.fill_value, dtype=ndtype)
        return self.__class__(self.shape, dtype=ndtype, buf=self, offset=0,
                strides=self.strides, fill_value=fill_value)

nmrecarray = NestedMaskedRecords
