#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
from numpy.ma.mrecords import MaskedRecords
from nptypes import _flatten_dtype


class NestedMaskedRecords(MaskedRecords):
    def __new__(cls, shape, dtype=None, buf=None, offset=0, strides=None,
                formats=None, names=None, titles=None, byteorder=None,
                aligned=False, mask=np.ma.nomask, hard_mask=False,
                fill_value=None, keep_mask=True, copy=False, **options):
        flattened_dtype = _flatten_dtype(dtype)
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
        _mask = np.ndarray.__getattribute__(self, '_mask')
        _data = np.ndarray.view(self, self.__dict__['_baseclass'])
        obj = np.array(_data[index], copy=False).view(nmrecarray)
        obj._mask = np.array(_mask[index], copy=False).view(np.recarray)
        obj._fill_value = np.array(self.fill_value, dtype=obj.dtype)
        return obj

    def __setattr__(self, attr, value):
        if value is np.ma.masked:
            obj = self.__getattr__(attr) # only subrecords
            if isinstance(obj, NestedMaskedRecords):
                for name in obj.dtype.names:
                    obj[name] = value
                return
        MaskedRecords.__setattr__(self, attr, value)

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
