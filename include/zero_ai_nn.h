#ifndef ZERO_AI_NN_H
#define ZERO_AI_NN_H

#include "zero_core.h"
#include <stdbool.h>

#define ZERO_NN_INPUT_DIM   16
#define ZERO_NN_HIDDEN1_DIM 32
#define ZERO_NN_HIDDEN2_DIM 32
#define ZERO_NN_OUTPUT_DIM  3

typedef struct ZeroNeuralNet {
    float w1[ZERO_NN_HIDDEN1_DIM][ZERO_NN_INPUT_DIM];
    float b1[ZERO_NN_HIDDEN1_DIM];
    float w2[ZERO_NN_HIDDEN2_DIM][ZERO_NN_HIDDEN1_DIM];
    float b2[ZERO_NN_HIDDEN2_DIM];
    float w3[ZERO_NN_OUTPUT_DIM][ZERO_NN_HIDDEN2_DIM];
    float b3[ZERO_NN_OUTPUT_DIM];
} ZeroNeuralNet;

void zero_nn_init_zeros(ZeroNeuralNet *net);
void zero_nn_forward(const ZeroNeuralNet *net, const float in[ZERO_NN_INPUT_DIM], float out[ZERO_NN_OUTPUT_DIM]);

#endif // ZERO_AI_NN_H
