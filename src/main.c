
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h>
#include <math.h>

#include "arm_math.h"

#define SAMPLE_RATE 11025
#define CARRIER_TONE 1000
#define BAUD_RATE (31.25f)
#define PLL_INCR (BAUD_RATE / (float)(SAMPLE_RATE))

#define SAMPLES_PER_PERIOD 100 // Must evenly divide CARRIER_TONE

#define clear() printf("\033[H\033[J")
#define gotoxy(x, y) printf("\033[%d;%dH", (y), (x))

// The threshold at which a bit is determined to be a reversal
const float detection_threshold = 1800000.0f; // 0.18f

#include "../fir_coefficients.h"

static float fir_state[SAMPLES_PER_PERIOD + FIR_STAGES - 1];

// uint8_t decoder(uint16_t data, uint8_t enable) {
//     uint8_t result = data & 0x00ff;
//     if (enable) {
//         uint8_t h[5];
//         h[0] = (data ^ (data >> 1) ^ (data >> 3) ^ (data >> 5) ^ (data >> 7)
//         ^
//                 (data >> 8)) &
//                0x0001;
//         h[1] = (data ^ (data >> 2) ^ (data >> 3) ^ (data >> 6) ^ (data >> 7)
//         ^
//                 (data >> 9)) &
//                0x0001;
//         h[2] = (data ^ (data >> 4) ^ (data >> 5) ^ (data >> 6) ^ (data >> 7)
//         ^
//                 (data >> 10)) &
//                0x0001;
//         h[3] = ((data >> 1) ^ (data >> 2) ^ (data >> 3) ^ (data >> 4) ^
//                 (data >> 5) ^ (data >> 6) ^ (data >> 7) ^ (data >> 11)) &
//                0x0001;
//         h[4] = (data ^ (data >> 1) ^ (data >> 2) ^ (data >> 3) ^ (data >> 4)
//         ^
//                 (data >> 5) ^ (data >> 6) ^ (data >> 7) ^ (data >> 8) ^
//                 (data >> 9) ^ (data >> 10) ^ (data >> 11) ^ (data >> 12)) &
//                0x0001;
//         uint8_t checkData = ((h[4] & !h[3] & h[2] & h[1] & h[0]) |
//                              ((h[4] & h[3] & !h[2] & !h[1] & h[0]) << 1) |
//                              ((h[4] & h[3] & !h[2] & h[1] & !h[0]) << 2) |
//                              ((h[4] & h[3] & !h[2] & h[1] & h[0]) << 3) |
//                              ((h[4] & h[3] & h[2] & !h[1] & !h[0]) << 4) |
//                              ((h[4] & h[3] & h[2] & !h[1] & h[0]) << 5) |
//                              ((h[4] & h[3] & h[2] & h[1] & !h[0]) << 6) |
//                              ((h[4] & h[3] & h[2] & h[1] & h[0]) << 7));
//         result = result ^ checkData;
//     }
//     return result;
// }

float make_carrier(float *carrier, size_t count, float freq, float sample_rate,
                   float32_t phase) {
    size_t i = 0;
    float32_t omega = 2.0 * M_PI * freq / sample_rate;
    for (i = 0; i < count; i++) {
        carrier[i] = sinf(phase);
        phase += omega;
    }
    return phase;
}

extern char varcode_to_char(uint32_t c);
static void print_char(uint32_t c) {
    printf("%c", varcode_to_char(c >> 2));
}

void demodulate_u16(int16_t *samples, size_t count) {
    arm_fir_instance_f32 fir;
    float32_t carrier[SAMPLES_PER_PERIOD];
    float32_t phase = 0;
    unsigned int j;

    // int iter;
    // int16_t s_max = 0;
    // int16_t s_min = 0;
    // for (iter = 0; iter < count; iter++) {
    //     if (samples[iter] > s_max) {
    //         s_max = samples[iter];
    //     }
    //     if (samples[iter] < s_min) {
    //         s_min = samples[iter];
    //     }
    // }
    // printf("MAX: %d  MIN: %d\n", s_max, s_min);

    arm_fir_init_f32(&fir, FIR_STAGES, fir_coefficients, fir_state,
                     SAMPLES_PER_PERIOD);

    int last_state = 0;
    size_t sample_offset;

    // clear();
    int sample_count = 0;

    float bit_pll = 0.6;

    float32_t demodulated[SAMPLES_PER_PERIOD];
    float32_t test_input[SAMPLES_PER_PERIOD];
    float32_t filtered_samples[SAMPLES_PER_PERIOD];
    float32_t *output;
    float32_t prev_bit = 0;
    float32_t prev_prev_bit = 0;

    int bit_acc = 0;

    for (sample_offset = 0; sample_offset < count;
         sample_offset += SAMPLES_PER_PERIOD) {

        phase = make_carrier(carrier, SAMPLES_PER_PERIOD, CARRIER_TONE,
                             SAMPLE_RATE, phase);

        // Convert this series of samples from int16_t (aka q15) to
        // float.
        arm_q15_to_float(&samples[sample_offset], test_input,
                         SAMPLES_PER_PERIOD);

        if (1) {
            // Perform coherent demodulation
            arm_mult_f32(test_input, carrier, demodulated, SAMPLES_PER_PERIOD);

            // Apply our bandpass filter
            arm_fir_f32(&fir, demodulated, filtered_samples,
                        SAMPLES_PER_PERIOD);
            output = filtered_samples;
        } else {
            // Apply our bandpass filter
            float32_t filtered_samples[SAMPLES_PER_PERIOD];
            arm_fir_f32(&fir, test_input, filtered_samples, SAMPLES_PER_PERIOD);

            // Perform coherent demodulation
            float32_t demodulated[SAMPLES_PER_PERIOD];
            arm_mult_f32(filtered_samples, carrier, demodulated,
                         SAMPLES_PER_PERIOD);
            output = demodulated;
        }

        for (j = 0; j < SAMPLES_PER_PERIOD; j++) {

            if (bit_pll < 0.5 && (bit_pll + PLL_INCR) >= 0.5) {
                float32_t sum = output[j] + prev_bit + prev_prev_bit;
                int state = (sum) < 0.0;
                int bit = !(state ^ last_state);
                last_state = state;

                bit_acc = (bit_acc << 1) | bit;
                if ((bit_acc & 3) == 0) {
                    print_char(bit_acc);
                    bit_acc = 0;
                }
            }
            // printf(fmt, output[j]);
            bit_pll += PLL_INCR;
            if (bit_pll >= 1) {
                bit_pll -= 1;
                // printf(" samples: %d\n B:", sample_count);
                sample_count = 0;
            } else {
                sample_count++;
            }

            prev_prev_bit = prev_bit;
            prev_bit = output[j];
            // int this_sample =
            //     ((demodulated[j * OVERSAMPLE] < THRESHOLD) ? 0x01 :
            //     0x00);
            // int bit = 1;
            // if (this_sample != last_sample) {
            //     bit = 0;
            // }
            // last_sample = this_sample;
            // fec_data_old |= bit << j;
            // printf("%d", bit);
        }
        // fec_data_new |=
        //     (fec_data_old
        //      << 10); // for 11.5 phase delay, when using 24-stage FIR
        // printf("  FEC data: %04x ", fec_data_new);

        // recovered_data[i] = decoder(fec_data_new, 1);
        // printf(" Recovered: %02x\n", recovered_data[i]);
    }
    printf("\n");
    return;
}

void filter_u16(int16_t *src, int16_t *dest, size_t count) {
    arm_fir_instance_f32 fir;
    float32_t carrier[SAMPLES_PER_PERIOD];
    float32_t phase = 0;

    // // Generate a carrier tone
    // make_carrier(carrier, SAMPLES_PER_PERIOD, CARRIER_TONE, SAMPLE_RATE);

    arm_fir_init_f32(&fir, FIR_STAGES, fir_coefficients, fir_state,
                     SAMPLES_PER_PERIOD);

    size_t sample_offset;

    for (sample_offset = 0; (sample_offset + SAMPLES_PER_PERIOD) < count;
         sample_offset += SAMPLES_PER_PERIOD) {
        phase = make_carrier(carrier, SAMPLES_PER_PERIOD, CARRIER_TONE,
                             SAMPLE_RATE, phase);
        // arm_float_to_q15(carrier, dest, SAMPLES_PER_PERIOD);
        // dest += SAMPLES_PER_PERIOD;
        // continue;

        // Convert this series of samples from int16_t (aka q15) to float.
        float32_t test_input[SAMPLES_PER_PERIOD];
        float32_t test_input_2[SAMPLES_PER_PERIOD];
        arm_q15_to_float(src, test_input, SAMPLES_PER_PERIOD);
        arm_q15_to_float(src + 1, test_input_2, SAMPLES_PER_PERIOD);
        src += SAMPLES_PER_PERIOD;
        // arm_mult_f32(test_input, test_input_2, test_input, SAMPLES_PER_PERIOD);

        float32_t *output;

        if (1) {
            // Perform coherent demodulation
            static float32_t demodulated[SAMPLES_PER_PERIOD];
            arm_mult_f32(test_input, carrier, demodulated, SAMPLES_PER_PERIOD);

            // Apply our bandpass filter
            static float32_t filtered_samples[SAMPLES_PER_PERIOD];
            arm_fir_f32(&fir, demodulated, filtered_samples,
                        SAMPLES_PER_PERIOD);
            output = filtered_samples;
        } else {
            // Apply our bandpass filter
            static float32_t filtered_samples[SAMPLES_PER_PERIOD];
            arm_fir_f32(&fir, test_input, filtered_samples, SAMPLES_PER_PERIOD);

            // Perform coherent demodulation
            static float32_t demodulated[SAMPLES_PER_PERIOD];
            arm_mult_f32(filtered_samples, carrier, demodulated,
                         SAMPLES_PER_PERIOD);
            output = demodulated;
        }

        arm_float_to_q15(output, dest, SAMPLES_PER_PERIOD);
        dest += SAMPLES_PER_PERIOD;
    }
    return;
}

int main(int argc, char **argv) {
    char *wave_file_name = "samples/PSK31_sample.wav";
    char *output_file_name = "filtered.wav";
    struct stat statbuf;
    uint8_t wave_header[44];
    int16_t *wave_buffer;
    int16_t *wave_filtered;

    if (argc > 1) {
        wave_file_name = argv[1];
    }

    printf("Opening %s for demodulation...\n", wave_file_name);
    int fd = open(wave_file_name, O_RDONLY);
    if (fd == -1) {
        perror("unable to open wave file");
        return 1;
    }

    int out_fd = open(output_file_name, O_WRONLY | O_CREAT | O_TRUNC, 0777);
    if (out_fd == -1) {
        perror("unable to open filtered file");
        return 1;
    }

    if (-1 == fstat(fd, &statbuf)) {
        perror("unable to determine size of buffer");
        return 1;
    }

    if (read(fd, wave_header, sizeof(wave_header)) != sizeof(wave_header)) {
        fprintf(stderr, "couldn't read wave header");
    }

    wave_buffer = malloc(statbuf.st_size - sizeof(wave_header));
    if ((unsigned int)read(fd, wave_buffer,
                           statbuf.st_size - sizeof(wave_header)) !=
        statbuf.st_size - sizeof(wave_header)) {
        fprintf(stderr, "short read of wave file\n");
        return 1;
    }

    wave_filtered = malloc(statbuf.st_size - sizeof(wave_header));
    memset(wave_filtered, 0, statbuf.st_size - sizeof(wave_header));
    filter_u16(wave_buffer, wave_filtered,
               (statbuf.st_size - sizeof(wave_header)) / sizeof(*wave_buffer));
    if (write(out_fd, wave_header, sizeof(wave_header)) == -1) {
        perror("error");
    }
    if (write(out_fd, wave_filtered, statbuf.st_size - sizeof(wave_header)) ==
        -1) {
        perror("error");
    }

    demodulate_u16((int16_t *)(wave_buffer + 44), (statbuf.st_size - 44) / 2);

    printf("Done demodulating\n");
    return 0;
}
