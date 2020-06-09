#!/usr/bin/env python3

import numpy
from scipy.signal import firwin, remez, kaiser_atten, kaiser_beta

taps=7
rate=62500
bandwidth=15000

coefficients = firwin(
        numtaps=taps,
        cutoff=bandwidth,
        fs=rate,
        pass_zero=False,
        scale=True,
    )

print(
    """// Filter defined in `make_coefficients.py` with the following parameters:
// taps: {}
// rate: {}
// bandwidth: {}
#define FIR_STAGES {}
static float fir_coefficients[FIR_STAGES] = {{""".format(
        taps, rate, bandwidth, taps
    )
)
for coef in coefficients:
    print("    {}f,".format(coef))
print("};")
