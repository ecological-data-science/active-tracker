
#pragma once

#include "constants.h"
#include "hello_world_float_model_data.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_log.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"

class TFLiteClassifier {
public:
    TFLiteClassifier();
    ~TFLiteClassifier();

    void classify(const float* input, float* output);

private:
    // Model data
    const tflite::Model* model;
    tflite::MicroInterpreter* interpreter;
    TfLiteTensor* input;
    TfLiteTensor* output;

    // Tensor arena
    static constexpr int kTensorArenaSize = 2 * 1024;
    uint8_t tensor_arena[kTensorArenaSize];

    // Inference count
    int inference_count;
};
