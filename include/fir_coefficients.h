// Filter defined in `make_coefficients.py` with the following parameters:
// taps: 7
// rate: 62500
// bandwidth: 15000
#define FIR_STAGES 7
static float fir_coefficients[FIR_STAGES] = {
    0.008506453934460999f,
    -0.006308691020576217f,
    -0.24955956070960691f,
    0.5305111684908603f,
    -0.24955956070960703f,
    -0.006308691020576225f,
    0.008506453934460999f,
};
