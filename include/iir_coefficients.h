// Filter defined in `make_iir_lpf.py` with the following parameters:
// order: 2
// rate: 62500
// bandwidth: 1000
#define LPF_A_STAGES 3
#define LPF_B_STAGES 3
static float lpf_a_coefficients[LPF_A_STAGES] = {
    1.0f,
    -1.9289422632520332f,
    0.9313816821269024f,
};

static float lpf_b_coefficients[LPF_b_STAGES] = {{
    0.0006098547187172994f,
    0.0012197094374345988f,
    0.0006098547187172994f,
};
