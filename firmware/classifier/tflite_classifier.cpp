
#include "tflite_classifier.h"
#include "tf_model.h"

TFLiteClassifier::TFLiteClassifier() {
}

void TFLiteClassifier::begin() {
  DEBUG_PRINT(("starting tflite setup\n"));
  tflite::InitializeTarget();
  DEBUG_PRINT(("target initialized\n"));

  // Map the model into a usable data structure. This doesn't involve any
  // copying or parsing, it's a very lightweight operation.
  model = tflite::GetModel(model_tflite);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
        DEBUG_PRINT(("Exiting setup\n"));
        DEBUG_PRINT(("Schema version mismatch\n"));
    return;
  }

  // This pulls in all the operation implementations we need.
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
        DEBUG_PRINT(("Exiting setup\n"));
        DEBUG_PRINT(("AllocateTensors() failed\n"));
    return;
  }

  // Obtain pointers to the model's input and output tensors.
  input = interpreter->input(0);
  output = interpreter->output(0);

  outputs = (float *)calloc(N_OUTPUTS, sizeof(float));
  DEBUG_PRINT(("tflite setup complete\n"));
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

int TFLiteClassifier::classify(const float *input_data) {

  for (uint16_t i = 0; i < N_INPUTS; i++)
    input->data.f[i] = input_data[i];

  // Run inference, and report any error
  TfLiteStatus invoke_status = interpreter->Invoke();
  if (invoke_status != kTfLiteOk) {
    return 0;
  }

  for (uint16_t i = 0; i < N_OUTPUTS; i++)
    outputs[i] = output->data.f[i];

  return getClassificationResult();
}
