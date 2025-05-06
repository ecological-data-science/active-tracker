
#include "tflite_classifier.h"
#include "tf_model.h"

TFLiteClassifier::TFLiteClassifier() {
}

void TFLiteClassifier::begin() {
  printf("starting setup\n");
  tflite::InitializeTarget();
  printf("target initialized\n");

  // Map the model into a usable data structure. This doesn't involve any
  // copying or parsing, it's a very lightweight operation.
  model = tflite::GetModel(model_tflite);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    printf("Exiting setup\n");
    printf("Schema version mismatch\n");
    return;
  }

  // This pulls in all the operation implementations we need.
  // NOLINTNEXTLINE(runtime-global-variables)
  static tflite::MicroMutableOpResolver<6> resolver;
  resolver.AddFullyConnected();
  resolver.AddRelu();
  resolver.AddMaxPool2D();
  resolver.AddConv2D();
  resolver.AddReshape();
  resolver.AddSoftmax();

  // Build an interpreter to run the model with.
  static tflite::MicroInterpreter static_interpreter(
      model, resolver, tensor_arena, kTensorArenaSize);
  interpreter = &static_interpreter;

  // Allocate memory from the tensor_arena for the model's tensors.
  TfLiteStatus allocate_status = interpreter->AllocateTensors();
  if (allocate_status != kTfLiteOk) {
    printf("Exiting setup\n");
    printf("AllocateTensors() failed\n");
    return;
  }

  // Obtain pointers to the model's input and output tensors.
  input = interpreter->input(0);
  output = interpreter->output(0);

  outputs = (float *)calloc(N_OUTPUTS, sizeof(float));
  printf("Exiting setup\n");
  printf("Setup complete\n");
}

TFLiteClassifier::~TFLiteClassifier() { free(outputs); }

int TFLiteClassifier::getClassificationResult() {

  float maxProba = outputs[0];
  int classification = 0;

  for (uint16_t i = 1; i < N_OUTPUTS; i++) {
    if (outputs[i] > maxProba) {
      classification = i;
      maxProba = outputs[i];
    }
  }
  return classification;
}

// The name of this function is important for Arduino compatibility.
int TFLiteClassifier::classify(const float *input_data) {
  // Calculate an x value to feed into the model. We compare the current
  // inference_count to the number of inferences per cycle to determine
  // our position within the range of possible x values the model was
  // trained on, and use this to calculate a value.

  // Quantize the input from floating-point to integer
  // int8_t x_quantized = x / input->params.scale + input->params.zero_point;
  // printf("scale: %f, zero_point: %d\n", input->params.scale,
  // input->params.zero_point); printf("x: %f, x_quantized: %d\n",
  // static_cast<double>(x), x_quantized); Place the quantized input in the
  // model's input tensor input->data.int8[0] = x_quantized;
  for (uint16_t i = 0; i < N_INPUTS; i++)
    input->data.f[i] = input_data[i];
  // Print the quantized input value
  // printf("x_quantized: %d\n", input->data.int8[0]);

  // Run inference, and report any error
  TfLiteStatus invoke_status = interpreter->Invoke();
  if (invoke_status != kTfLiteOk) {
    return 0;
  }

  // // Obtain the quantized output from model's output tensor
  // int8_t y_quantized = output->data.int8[0];
  // printf("y_quantized: %d\n", y_quantized);
  // // Dequantize the output from integer to floating-point
  // float y = (y_quantized - output->params.zero_point) * output->params.scale;
  for (uint16_t i = 0; i < N_OUTPUTS; i++)
    outputs[i] = output->data.f[i];

  // Output the results. A custom HandleOutput function can be implemented
  // for each supported hardware target.

  // Increment the inference_counter, and reset it if we have reached
  // the total number per cycle
  return getClassificationResult();
}
