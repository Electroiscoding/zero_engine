#include "zero_ai_nn.h"
#include <string.h>
#include <math.h>

static inline float fast_relu(float x) {
    return x > 0.0f ? x : 0.0f;
}

static inline float fast_sigmoid(float x) {
    if (x > 12.0f) return 1.0f;
    if (x < -12.0f) return 0.0f;
    return 1.0f / (1.0f + expf(-x));
}

void zero_nn_init_zeros(ZeroNeuralNet *net) {
    if (!net) return;
    memset(net, 0, sizeof(ZeroNeuralNet));
}

void zero_nn_forward(const ZeroNeuralNet *net, const float in[ZERO_NN_INPUT_DIM], float out[ZERO_NN_OUTPUT_DIM]) {
    if (!net || !in || !out) return;

    float h1[ZERO_NN_HIDDEN1_DIM];
    for (int i = 0; i < ZERO_NN_HIDDEN1_DIM; ++i) {
        float sum = net->b1[i];
        for (int j = 0; j < ZERO_NN_INPUT_DIM; ++j) {
            sum += net->w1[i][j] * in[j];
        }
        h1[i] = fast_relu(sum);
    }

    float h2[ZERO_NN_HIDDEN2_DIM];
    for (int i = 0; i < ZERO_NN_HIDDEN2_DIM; ++i) {
        float sum = net->b2[i];
        for (int j = 0; j < ZERO_NN_HIDDEN1_DIM; ++j) {
            sum += net->w2[i][j] * h1[j];
        }
        h2[i] = fast_relu(sum);
    }

    float z_steer = net->b3[0];
    for (int j = 0; j < ZERO_NN_HIDDEN2_DIM; ++j) {
        z_steer += net->w3[0][j] * h2[j];
    }
    out[0] = tanhf(z_steer);

    float z_throttle = net->b3[1];
    for (int j = 0; j < ZERO_NN_HIDDEN2_DIM; ++j) {
        z_throttle += net->w3[1][j] * h2[j];
    }
    out[1] = fast_sigmoid(z_throttle);

    float z_brake = net->b3[2];
    for (int j = 0; j < ZERO_NN_HIDDEN2_DIM; ++j) {
        z_brake += net->w3[2][j] * h2[j];
    }
    out[2] = fast_sigmoid(z_brake);
}
