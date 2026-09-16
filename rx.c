/*
 rx.c
 读取 16-bit PCM WAV 文件，对 2-FSK 信号做解调（Goertzel），自动同步前导并输出 BER（如果提供 gold.bits）。
 用法: ./rx in.wav [gold.bits]
*/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#define FS 44100
#define F0 1200.0
#define F1 2200.0
#define BIT_DUR 0.05
#define PREAMBLE_BITS 32

// Simple WAV reader: assumes 16-bit PCM mono
int16_t *read_wav(const char *path, int *out_samples) {
    FILE *f = fopen(path, "rb");
    if (!f) { perror("open"); return NULL; }
    char riff[4];
    fread(riff,1,4,f);
    if (strncmp(riff,"RIFF",4) != 0) { fclose(f); fprintf(stderr,"not RIFF\n"); return NULL; }
    fseek(f, 22, SEEK_SET); // numChannels at 22
    uint16_t numChannels;
    fread(&numChannels, 2, 1, f);
    if (numChannels != 1) { fclose(f); fprintf(stderr,"need mono WAV\n"); return NULL; }
    fseek(f, 24, SEEK_SET); // sample rate
    uint32_t sampleRate;
    fread(&sampleRate,4,1,f);
    if (sampleRate != FS) { /* warn but continue */ }
    // find "data" chunk
    fseek(f, 12, SEEK_SET);
    char id[5]; id[4]=0;
    uint32_t size;
    while (1) {
        if (fread(id,1,4,f) != 4) { fclose(f); fprintf(stderr,"no data chunk\n"); return NULL; }
        fread(&size,4,1,f);
        if (strncmp(id,"data",4)==0) break;
        fseek(f, size, SEEK_CUR);
    }
    int samples = size / 2;
    int16_t *pcm = (int16_t*)malloc(samples * sizeof(int16_t));
    if (!pcm) { fclose(f); return NULL; }
    fread(pcm, sizeof(int16_t), samples, f);
    fclose(f);
    *out_samples = samples;
    return pcm;
}

// Goertzel: compute magnitude^2 for target frequency on samples[start..start+N-1]
double goertzel_power(const double *x, int start, int N, double target_f) {
    double k = (0.5 + ((N * target_f) / FS));
    double omega = 2.0 * M_PI * target_f / FS;
    double coeff = 2.0 * cos(omega);
    double s_prev = 0.0, s_prev2 = 0.0, s;
    for (int i = 0; i < N; ++i) {
        double s_in = x[start + i];
        s = s_in + coeff * s_prev - s_prev2;
        s_prev2 = s_prev;
        s_prev = s;
    }
    double power = s_prev2*s_prev2 + s_prev*s_prev - coeff*s_prev*s_prev2;
    return power;
}

int main(int argc, char **argv) {
    if (argc < 2) { fprintf(stderr,"usage: %s in.wav [gold.bits]\n", argv[0]); return 1; }
    const char *wav = argv[1];
    const char *goldf = (argc>=3)?argv[2]:NULL;

    int samples;
    int16_t *pcm = read_wav(wav, &samples);
    if (!pcm) return 1;

    // convert to double [-1,1]
    double *x = (double*)malloc(sizeof(double)*samples);
    for (int i = 0; i < samples; ++i) x[i] = pcm[i] / 32768.0;

    int samples_per_bit = (int)round(BIT_DUR * FS);
    int max_bits = samples / samples_per_bit;
    if (max_bits < PREAMBLE_BITS) { fprintf(stderr,"wav too short\n"); return 1; }

    // Try offsets 0..samples_per_bit-1 and pick one with best preamble match
    int best_offset = 0;
    int best_matches = -1;
    for (int offset = 0; offset < samples_per_bit; ++offset) {
        int matches = 0;
        for (int b = 0; b < PREAMBLE_BITS; ++b) {
            int start = offset + b * samples_per_bit;
            if (start + samples_per_bit > samples) break;
            double p0 = goertzel_power(x, start, samples_per_bit, F0);
            double p1 = goertzel_power(x, start, samples_per_bit, F1);
            int bit = (p1 > p0) ? 1 : 0;
            int expected = (b % 2) ? 0 : 1; // preamble 1010...
            if (bit == expected) matches++;
        }
        if (matches > best_matches) { best_matches = matches; best_offset = offset; }
    }

    // demodulate all bits using best_offset
    int total_bits = (samples - best_offset) / samples_per_bit;
    unsigned char *bits = (unsigned char*)malloc(total_bits);
    for (int b = 0; b < total_bits; ++b) {
        int start = best_offset + b * samples_per_bit;
        if (start + samples_per_bit > samples) { total_bits = b; break; }
        double p0 = goertzel_power(x, start, samples_per_bit, F0);
        double p1 = goertzel_power(x, start, samples_per_bit, F1);
        bits[b] = (p1 > p0) ? 1 : 0;
    }

    printf("best_offset=%d samples_per_bit=%d total_bits=%d preamble_matches=%d/%d\n",
           best_offset, samples_per_bit, total_bits, best_matches, PREAMBLE_BITS);

    // if gold bits provided, compute BER (compare starting at preamble)
    if (goldf) {
        FILE *g = fopen(goldf,"rb");
        if (!g) perror("open gold");
        else {
            // read gold
            fseek(g,0,SEEK_END); int glen = ftell(g); fseek(g,0,SEEK_SET);
            unsigned char *gold = (unsigned char*)malloc(glen);
            fread(gold,1,glen,g); fclose(g);
            int cmp_bits = (glen < total_bits) ? glen : total_bits;
            int errs = 0;
            for (int i = 0; i < cmp_bits; ++i) if (bits[i] != gold[i]) errs++;
            printf("Compared %d bits, errors=%d, BER=%g\n", cmp_bits, errs, (double)errs / cmp_bits);
            free(gold);
        }
    } else {
        // else write decoded bits to file
        FILE *out = fopen("decoded.bits","wb");
        fwrite(bits,1,total_bits,out);
        fclose(out);
        printf("decoded bits written to decoded.bits\n");
    }

    free(pcm); free(x); free(bits);
    return 0;
}