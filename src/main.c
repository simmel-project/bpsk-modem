
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

#define SAMPLES_PER_PERIOD 20 // Must evenly divide CARRIER_TONE

#define clear() printf("\033[H\033[J")
#define gotoxy(x, y) printf("\033[%d;%dH", (y), (x))

// The threshold at which a bit is determined to be a reversal
const float detection_threshold = 1800000.0f; // 0.18f

#include "../fir_coefficients.h"
#include "../lpf_coefficients.h"

static float fir_state[SAMPLES_PER_PERIOD + FIR_STAGES - 1];
static float i_lpf_state[SAMPLES_PER_PERIOD + FIR_STAGES - 1];
static float q_lpf_state[SAMPLES_PER_PERIOD + FIR_STAGES - 1];

uint8_t wave_header[44];

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

void write_wav(int16_t *data, size_t len, const char *name) {
    int out_fd = open(name, O_WRONLY | O_CREAT | O_TRUNC, 0777);
    if (out_fd == -1) {
      perror("unable to open filtered file");
      return 1;
    }
    if (write(out_fd, wave_header, sizeof(wave_header)) == -1) {
        perror("error");
    }
    if (write(out_fd, data, len) ==
        -1) {
        perror("error");
    }
    close(out_fd);
}

void write_wav_stereo(int16_t *left, int16_t *right, size_t len, const char *name) {
    int out_fd = open(name, O_WRONLY | O_CREAT | O_TRUNC, 0777);
    if (out_fd == -1) {
      perror("unable to open filtered file");
      return 1;
    }
    if (write(out_fd, wave_header, sizeof(wave_header)) == -1) {
        perror("error");
    }
    for(int i = 0; i < len; i++ ) {
      if (write(out_fd, &(left[i]), 2) == -1) {
        perror("error");
      }
      if (write(out_fd, &(right[i]), 2) == -1) {
        perror("error");
      }
    }
    close(out_fd);
}

void filter_u16(int16_t *src, int16_t *dest, size_t count) {
    arm_fir_instance_f32 fir;
    float32_t carrier[SAMPLES_PER_PERIOD];
    float32_t phase = 0;
    int16_t buf[SAMPLES_PER_PERIOD];

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

struct nco_state {
  float32_t samplerate; // Hz
  float32_t freq;       // Hz
  
  float32_t phase;      // rad
} nco_st;

void nco(float32_t control, uint32_t timestep, float32_t *i, float32_t *q) {
  // control is a number from +1 to -1, which translates to -pi to +pi additional phase per time step
  // timestep is the current time step, expressed in terms of samples since t=0
  // i is a pointer to the in-phase return value
  // q is a pointer to the quadrature return value

  nco_st.phase += (control / PI);
  
  *i = cos( (((float32_t)timestep) * nco_st.freq * 2 * PI) / nco_st.samplerate + nco_st.phase);
  *q = sin( (((float32_t)timestep) * nco_st.freq * 2 * PI) / nco_st.samplerate + nco_st.phase);
}


int main(int argc, char **argv) {
    char *wave_file_name = "samples/PSK31_sample.wav";
    char *output_file_name = "filtered.wav";
    struct stat statbuf;
    int16_t *wave_buffer;
    int16_t *wave_filtered;
    int16_t *wave_upsampled;
    int16_t *wave_upsampled_filtered;

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

    int baselen = statbuf.st_size - sizeof(wave_header);
    wave_buffer = malloc(baselen * sizeof(int16_t));
    if ((unsigned int)read(fd, wave_buffer, baselen) !=
        statbuf.st_size - sizeof(wave_header)) {
        fprintf(stderr, "short read of wave file\n");
        return 1;
    }

    wave_filtered = malloc(baselen * sizeof(int16_t));
    memset(wave_filtered, 0, baselen * sizeof(int16_t));

    int samplerate = *((unsigned int *) &(wave_header[24]));
    printf("samplerate: %d\n", samplerate);
    
    // double the sampling rate through interpolation and filtering
    baselen = baselen * 2;
    wave_upsampled = malloc(baselen * sizeof(int16_t));
    wave_upsampled_filtered = malloc(baselen * sizeof(int16_t));

    memset(wave_upsampled, 0, baselen * sizeof(int16_t));
    memset(wave_upsampled_filtered, 0, baselen * sizeof(int16_t));
    samplerate = samplerate * 2;
    *((unsigned int *) &(wave_header[24])) = samplerate;

    for( int i = 0; i < baselen / 2; i ++ ) {
      wave_upsampled[i*2] = wave_buffer[i];
      wave_upsampled[i*2+1] = 0;
    }
    
    arm_fir_instance_f32 fir;
    arm_fir_init_f32(&fir, FIR_STAGES, fir_coefficients, fir_state,
                     SAMPLES_PER_PERIOD);
    for (int sample_offset = 0; sample_offset < baselen;
         sample_offset += SAMPLES_PER_PERIOD) {
      // Apply our bandpass filter
      float32_t firwindow[SAMPLES_PER_PERIOD];
      arm_q15_to_float(&(wave_upsampled[sample_offset]), firwindow, SAMPLES_PER_PERIOD);
      static float32_t filtered_samples[SAMPLES_PER_PERIOD];
      arm_fir_f32(&fir, firwindow, filtered_samples,
		  SAMPLES_PER_PERIOD);
      arm_float_to_q15(filtered_samples, &(wave_upsampled_filtered[sample_offset]), SAMPLES_PER_PERIOD);
    }
    
    write_wav(wave_upsampled_filtered, baselen, "upsampled_filtered.wav");

    // now try to run a PLL to "lock" onto the signal
    nco_st.samplerate = (float32_t) samplerate;
    nco_st.freq = 1000.00;
    nco_st.phase = 0.0;

    /////// test the NCO
    int16_t *i_int;
    int16_t *q_int;
    i_int = malloc(baselen * sizeof(int16_t));
    q_int = malloc(baselen * sizeof(int16_t));
    memset(i_int, 0, baselen);
    memset(q_int, 0, baselen);

    for( int i = 0; i < baselen; i++ ) {
      float32_t i_flt;
      float32_t q_flt;

      nco(0.0, (uint32_t) i, &i_flt, &q_flt);
      i_int[i] = (int16_t) (i_flt * 8000);
      q_int[i] = (int16_t) (q_flt * 8000);
    }
    wave_header[22] = 2; // stereo
    uint32_t byterate = *((uint32_t *) &(wave_header[28]));
    *((uint32_t *) &(wave_header[28])) = byterate * 2;
    uint32_t length = *((uint32_t *) &(wave_header[40]));
    *((uint32_t *) &(wave_header[40])) = length * 4; // *4 because we forgot to expand it when we doubled sample rate
    write_wav_stereo(i_int, q_int, baselen, "quadrature.wav");
    ///////

    // multiply the NCO output times input signal
    int16_t *i_loop;
    int16_t *q_loop;
    i_loop = malloc(baselen * sizeof(int16_t));
    q_loop = malloc(baselen * sizeof(int16_t));
    memset(i_loop, 0, baselen);
    memset(q_loop, 0, baselen);

    arm_fir_instance_f32 i_lpf;
    arm_fir_instance_f32 q_lpf;
    arm_fir_init_f32(&i_lpf, LPF_STAGES, lpf_coefficients, i_lpf_state,
                     SAMPLES_PER_PERIOD);
    arm_fir_init_f32(&q_lpf, LPF_STAGES, lpf_coefficients, q_lpf_state,
                     SAMPLES_PER_PERIOD);
    float32_t nco_error = 0.0;
    for (int sample_offset = 0; sample_offset < baselen;
         sample_offset += SAMPLES_PER_PERIOD) {
      
      float32_t loopwindow[SAMPLES_PER_PERIOD];
      arm_q15_to_float(&(wave_upsampled_filtered[sample_offset]), loopwindow, SAMPLES_PER_PERIOD);

      float32_t i_samps[SAMPLES_PER_PERIOD];
      float32_t q_samps[SAMPLES_PER_PERIOD];
      for(int i = 0; i < SAMPLES_PER_PERIOD; i++) {
	nco(nco_error, (uint32_t) (i + sample_offset), &(i_samps[i]), &(q_samps[i]));
      }
      //arm_q15_to_float(&(i_int[sample_offset]), i_samps, SAMPLES_PER_PERIOD);
      //arm_q15_to_float(&(q_int[sample_offset]), q_samps, SAMPLES_PER_PERIOD);
	
      static float32_t i_mult_samps[SAMPLES_PER_PERIOD];
      static float32_t q_mult_samps[SAMPLES_PER_PERIOD];
      arm_mult_f32(loopwindow, i_samps, i_mult_samps, SAMPLES_PER_PERIOD);
      arm_mult_f32(loopwindow, q_samps, q_mult_samps, SAMPLES_PER_PERIOD);
      
      static float32_t i_lpf_samples[SAMPLES_PER_PERIOD];
      static float32_t q_lpf_samples[SAMPLES_PER_PERIOD];
      arm_fir_f32(&i_lpf, i_mult_samps, i_lpf_samples, SAMPLES_PER_PERIOD);
      arm_fir_f32(&q_lpf, q_mult_samps, q_lpf_samples, SAMPLES_PER_PERIOD);
      
      arm_float_to_q15(i_lpf_samples, &(i_loop[sample_offset]), SAMPLES_PER_PERIOD);
      arm_float_to_q15(q_lpf_samples, &(q_loop[sample_offset]), SAMPLES_PER_PERIOD);

      float32_t errorwindow[SAMPLES_PER_PERIOD];
      arm_mult_f32(i_lpf_samples, q_lpf_samples, errorwindow, SAMPLES_PER_PERIOD);
      float32_t avg = 0;
      for(int i = 0; i < SAMPLES_PER_PERIOD; i++) {
	avg += errorwindow[i];
      }
      avg /= ((float32_t) SAMPLES_PER_PERIOD);
      nco_error = -(avg);
      // printf("err: %0.04f\n", nco_error);

    }
    printf("\n");
    write_wav_stereo(i_loop, q_loop, baselen, "quadrature_loop.wav");
    
    
    /*
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
    */
    return 0;
}
