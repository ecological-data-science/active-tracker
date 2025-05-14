
#pragma once


#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_log.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"

#include "debug.h"
// class for the classifier
#define N_CHANNELS 5
#define SEG_LENGTH 50


#define BIT_LENGTH 4        // we write 4 predictions to a single byte as we need 2 bits per prediction

#define N_INPUTS SEG_LENGTH*N_CHANNELS
#define N_OUTPUTS 4

class TFLiteClassifier {
public:
    TFLiteClassifier();
    ~TFLiteClassifier();

    int classify(const float* input);
    void begin();

private:
    // Model data
    const tflite::Model* model;
    tflite::MicroInterpreter* interpreter;
    TfLiteTensor* input;
    TfLiteTensor* output;

    // Tensor arena
    static constexpr int kTensorArenaSize = 4 * 1024;
    uint8_t tensor_arena[kTensorArenaSize];

    int getClassificationResult();

    float *outputs;
};
