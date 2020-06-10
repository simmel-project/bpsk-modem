#!/usr/bin/env python3

import numpy
from scipy.signal import firwin, remez, kaiser_atten, kaiser_beta, bessel, iirfilter

order = 2
rate = 62500
bandwidth = 1000
b, a = iirfilter(order, bandwidth / rate, btype='lowpass', ftype='butter', output='ba')

print(
    """// Filter defined in `make_iir_lpf.py` with the following parameters:
// order: {}
// rate: {}
// bandwidth: {}
#define LPF_A_STAGES {}
#define LPF_B_STAGES {}
static float lpf_a_coefficients[LPF_A_STAGES] = {{""".format(
        order, rate, bandwidth, len(a), len(b)
    )
)
for coef in a:
    print("    {}f,".format(coef))
print("};")

print(
    """
static float lpf_b_coefficients[LPF_b_STAGES] = {{""")
for coef in b:
    print("    {}f,".format(coef))
print("};")
