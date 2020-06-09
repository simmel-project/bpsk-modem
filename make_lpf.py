#!/usr/bin/env python3

import numpy
from scipy.signal import firwin, remez, kaiser_atten, kaiser_beta


def create_filter(rate, tone, taps, bandwidth, window="hamming"):
    # rate = rate * 0.5
    # tone = tone / rate
    # bandwidth = bandwidth / rate
    fl = tone - (bandwidth / 2)
    fh = tone + (bandwidth / 2)
    return firwin(
        numtaps=taps,
        cutoff=bandwidth,
        fs=rate,
        pass_zero='lowpass',
        window=window,
        scale=True,
    )


def bandpass_kaiser(ntaps, lowcut, highcut, fs, width):
    nyq = 0.5 * fs
    atten = kaiser_atten(ntaps, width / nyq)
    beta = kaiser_beta(atten)
    taps = firwin(
        ntaps,
        [lowcut, highcut],
        nyq=nyq,
        pass_zero="bandpass",
        window=("kaiser", beta),
        scale=True,
    )
    return taps


def bandpass_remez(ntaps, lowcut, highcut, fs, width):
    delta = 0.5 * width
    edges = [
        0,
        lowcut - delta,
        lowcut + delta,
        highcut - delta,
        highcut + delta,
        0.5 * fs,
    ]
    taps = remez(ntaps, edges, [0, 1, 0], Hz=fs)
    return taps


taps = 16
rate = 11025
tone = 1000
bandwidth = 50
coefficients = create_filter(rate=rate, tone=tone, bandwidth=bandwidth, taps=taps, window="hamming")
# coefficients = bandpass_remez(taps, (1000 - bandwidth / 2), (1000 + bandwidth / 2), rate, 1.0)
# coefficients = bandpass_kaiser(taps, (1000 - bandwidth / 2), (1000 + bandwidth / 2), rate, 1.0)

print(
    """// Filter defined in `make_coefficients.py` with the following parameters:
// taps: {}
// rate: {}
// tone: {}
// bandwidth: {}
#define LPF_STAGES {}
static float lpf_coefficients[FIR_STAGES] = {{""".format(
        taps, rate, tone, bandwidth, taps
    )
)
for coef in coefficients:
    print("    {}f,".format(coef))
print("};")
