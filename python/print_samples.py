#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import convert


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(("Usage: {} <sample_log_file>\n\nPrint sample log " +
               "data.").format(__file__));
        print("\t<sample_log_file>\tFile containing samples in " +
                "flatbuffers binary format.")
        sys.exit(1)

    samples = convert.load_sample_log(sys.argv[1])
    print(repr(samples))
    sys.exit(0)
