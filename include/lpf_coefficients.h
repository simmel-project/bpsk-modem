// Filter defined in `make_coefficients.py` with the following parameters:
// taps: 8
// rate: 11025
// tone: 1000
// bandwidth: 100
#define LPF_STAGES 8
static float lpf_coefficients[FIR_STAGES] = {
    0.020612755292777035f,
    0.0654506605746566f,
    0.166409803382952f,
    0.24752678074961434f,
    0.24752678074961434f,
    0.166409803382952f,
    0.0654506605746566f,
    0.020612755292777035f,
};
