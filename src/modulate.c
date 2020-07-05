#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include "modulate.h"
const char *char_to_varcode(char c);

void modulate_init(struct modulate_state *state, uint32_t carrier,
                   uint32_t rate, float baud,
                   void (*write)(void *arg, void *data, unsigned int count),
                   void *arg) {
    state->cfg.rate = rate;
    state->cfg.carrier = carrier;
    state->cfg.baud = baud;
    state->cfg.pll_incr = (float)carrier / (float)rate;
    state->cfg.write = write;
    state->cfg.write_arg = arg;
    state->cfg.omega = (2.0 * M_PI * carrier) / rate;
    state->bit_pll = 0.0;
    state->baud_pll = 0.0;
    state->polarity = 0;
    return;
}

static void send_bit(struct modulate_state *state, int bit) {
    int16_t low = -32768/4;
    int16_t high = 32767/4;
    int sign = 1;

    printf("%d", bit);

    state->polarity ^= !bit;
    if (state->polarity) {
        sign = -1;
        low = 32767/4;
        high = -32768/4;
    }

    int bit_count = 0;

// #define SINE_MODE

    // Resetting these greatly helps demodulation when in SINE mode
#ifdef SINE_MODE
    state->bit_pll = 0;
    state->baud_pll = 0;
#endif

    while (bit_count < 32) {
#ifdef SINE_MODE
        int16_t num = cosf(state->bit_pll) * sign * 32767;
        state->cfg.write(state->cfg.write_arg, &num, sizeof(num));
        state->bit_pll += state->cfg.omega;
#else
        if (state->baud_pll > 0.5) {
            // printf("writing high %d: @ %p ", high, &high);
            state->cfg.write(state->cfg.write_arg, &high, sizeof(high));
        } else {
            // printf("writing low %d: @ %p ", low, &low);
            state->cfg.write(state->cfg.write_arg, &low, sizeof(low));
        }
#endif
        state->baud_pll += state->cfg.pll_incr;
        if (state->baud_pll > 1.0) {
            bit_count++;
            state->baud_pll -= 1.0;
        }
    }
}

void modulate_string(struct modulate_state *state, const char *string) {
    char c;

    printf("Sending: ");
    for (unsigned int i = 0; i < 8; i++) {
        send_bit(state, 0);
    }

    while ((c = *string++) != '\0') {
        const char *bit_pattern = char_to_varcode(c);

        int bit = 0;
        while ((bit = *bit_pattern++) != '\0') {
            send_bit(state, bit != '0');
        }
        send_bit(state, 0);
        send_bit(state, 0);
    }
    printf("\n");
}
