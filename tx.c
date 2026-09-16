/*
 tx.c
 生成 16-bit PCM WAV 的 2-FSK 测试信号，包含前导 + 伪随机负载。
 可选参数: ./tx out.wav [snr_db] [num_payload_bits]
 如果 snr_db is given, 在写入 wav 前把噪声按目标 SNR 加入（AWGN, dB）。
 采样率 44100, f0=1200Hz, f1=2200Hz, bit_dur=0.05s
*/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

#define FS 44100
#define F0 1200.0
#define F1 2200.0
#define BIT_DUR 0.05   // seconds per bit
#define PREAMBLE_BITS 32
#define AMPLITUDE 0.9  // peak amplitude (scaled before int16)

static double gaussian_rand() {
    // Box-Muller
    static int has_spare = 0;
    static double spare;
    if (has_spare) {
        has_spare = 0;
        return spare;
    }
    has_spare = 1;
    double u, v, s;
    do {
        u = ((double)rand() / RAND_MAX) * 2.0 - 1.0;
        v = ((double)rand() / RAND_MAX) * 2.0 - 1.0;
        s = u*u + v*v;
    } while (s == 0 || s >= 1.0);
    s = sqrt(-2.0 * log(s) / s);
    spare = v * s;
    return u * s;
}

void write_wav_header(FILE *f, int32_t samples) {
    int32_t subchunk1 = 16;
    int16_t audio_format = 1;
    int16_t num_channels = 1;
    int32_t byte_rate = FS * num_channels * 2;
    int16_t block_align = num_channels * 2;
    int16_t bits_per_sample = 16;
    int32_t subchunk2 = samples * num_channels * 2;
    int32_t chunk_size = 4 + (8 + subchunk1) + (8 + subchunk2);

    fseek(f, 0, SEEK_SET);
    fwrite("RIFF", 1, 4, f);
    fwrite(&chunk_size, 4, 1, f);
    fwrite("WAVE", 1, 4, f);
    fwrite("fmt ", 1, 4, f);
    fwrite(&subchunk1, 4, 1, f);
    fwrite(&audio_format, 2, 1, f);
    fwrite(&num_channels, 2, 1, f);
    uint32_t temp_fs = FS;
    fwrite(&temp_fs, 4, 1, f);

    fwrite(&byte_rate, 4, 1, f);
    fwrite(&block_align, 2, 1, f);
    fwrite(&bits_per_sample, 2, 1, f);
    fwrite("data", 1, 4, f);
    fwrite(&subchunk2, 4, 1, f);
}

int main(int argc, char **argv) {
    srand((unsigned)time(NULL));
    const char *out = "out.wav";
    double target_snr_db = 1e9;
    int payload_bits = 1024;

    if (argc >= 2) out = argv[1];
    if (argc >= 3) target_snr_db = atof(argv[2]);
    if (argc >= 4) payload_bits = atoi(argv[3]);

    int nibits = PREAMBLE_BITS + payload_bits;
    int samples_per_bit = (int)round(BIT_DUR * FS);
    int total_samples = nibits * samples_per_bit;

    double *buf = (double*)malloc(sizeof(double) * total_samples);
    int16_t *pcm = (int16_t*)malloc(sizeof(int16_t) * total_samples);
    if (!buf || !pcm) { fprintf(stderr, "alloc failed\n"); return 1; }

    // build bits: preamble (1010...), then PRBS
    unsigned char *bits = (unsigned char*)malloc(nibits);
    for (int i = 0; i < PREAMBLE_BITS; ++i) bits[i] = (i % 2) ? 0 : 1;
    // simple LFSR for payload
    unsigned int lfsr = 0xACE1u;
    for (int i = PREAMBLE_BITS; i < nibits; ++i) {
        lfsr = (lfsr >> 1) ^ (-(int)(lfsr & 1u) & 0xB400u);
        bits[i] = lfsr & 1;
    }

    // generate samples
    for (int bit = 0; bit < nibits; ++bit) {
        double f = bits[bit] ? F1 : F0;
        for (int n = 0; n < samples_per_bit; ++n) {
            int idx = bit * samples_per_bit + n;
            double t = (double)idx / FS;
            double s = sin(2.0 * M_PI * f * t);
            // Hanning window per bit to reduce spectral leakage
            double w = 0.5 * (1.0 - cos(2.0 * M_PI * n / (samples_per_bit - 1)));
            buf[idx] = AMPLITUDE * s * w;
        }
    }

    // compute signal power
    double sig_power = 0;
    for (int i = 0; i < total_samples; ++i) sig_power += buf[i]*buf[i];
    sig_power /= total_samples;

    // add AWGN if requested
    if (target_snr_db < 1e8) {
        double snr_lin = pow(10.0, target_snr_db / 10.0);
        double noise_power = sig_power / snr_lin;
        double sigma = sqrt(noise_power);
        for (int i = 0; i < total_samples; ++i) {
            double n = gaussian_rand() * sigma;
            buf[i] += n;
        }
    }

    // convert to int16
    for (int i = 0; i < total_samples; ++i) {
        double v = buf[i];
        if (v > 1.0) v = 1.0;
        if (v < -1.0) v = -1.0;
        pcm[i] = (int16_t)lrint(v * 32767.0);
    }

    // write wav
    FILE *f = fopen(out, "wb");
    if (!f) { perror("fopen"); return 1; }
    // placeholder header
    for (int i = 0; i < 44; ++i) fputc(0, f);
    fwrite(pcm, sizeof(int16_t), total_samples, f);
    write_wav_header(f, total_samples);
    fclose(f);

    // also write gold bits to file for BER calculation by receiver
    FILE *g = fopen("gold.bits", "wb");
    fwrite(bits, 1, nibits, g);
    fclose(g);

    printf("WAV written: %s  bits=%d samples=%d SNR_db=%g\n", out, nibits, total_samples,
           (target_snr_db<1e8)?target_snr_db:INFINITY);

    free(buf); free(pcm); free(bits);
    return 0;
}