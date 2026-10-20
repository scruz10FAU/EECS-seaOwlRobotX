// tflite_gpu_shim: a tiny C ABI over the TFLite C API + OpenCL GPU delegate,
// for Python (ctypes, see tflite_gpu.py).
//
// Why: Python's tflite_runtime has no GPU delegate, and loading a delegate built
// against one TFLite version into another version's interpreter would mix struct
// layouts. Linking interpreter + delegate into one .so keeps them the same
// version; only the bg_* functions below are exported (CMakeLists.txt hides
// everything else). TFLite itself is built from source in the beacon_overlay
// image (../beacon_overlay/Dockerfile), independent of the drone's qrb5165-tflite.
//
// The delegate dlopen()s libOpenCL.so at run time, so the container needs
// /dev/kgsl-3d0 and the Adreno user-space libraries (see run.sh).

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <string>

#include "tensorflow/lite/core/c/c_api.h"
#include "tensorflow/lite/delegates/gpu/delegate.h"

#define BG_EXPORT extern "C" __attribute__((visibility("default")))

struct bg_t {
  TfLiteModel* model = nullptr;
  TfLiteInterpreterOptions* opts = nullptr;
  TfLiteInterpreter* interp = nullptr;
  TfLiteDelegate* gpu = nullptr;
  std::string log;  // everything the error reporter saw
};

static void reporter(void* user, const char* fmt, va_list args) {
  char buf[1024];
  vsnprintf(buf, sizeof(buf), fmt, args);
  auto* h = static_cast<bg_t*>(user);
  h->log += buf;
  h->log += '\n';
  fprintf(stderr, "[tflite_gpu_shim] %s\n", buf);
}

static void set_err(char* err, int errlen, const std::string& msg) {
  if (err && errlen > 0) snprintf(err, errlen, "%s", msg.c_str());
}

static void release(bg_t* h) {
  if (h->interp) TfLiteInterpreterDelete(h->interp);
  if (h->opts) TfLiteInterpreterOptionsDelete(h->opts);
  if (h->gpu) TfLiteGpuDelegateV2Delete(h->gpu);
  h->interp = nullptr;
  h->opts = nullptr;
  h->gpu = nullptr;
}

// Build interpreter options (+ GPU delegate when use_gpu) and the interpreter.
static bool build(bg_t* h, int use_gpu, int threads, int fp16, int sustained) {
  h->opts = TfLiteInterpreterOptionsCreate();
  TfLiteInterpreterOptionsSetNumThreads(h->opts, threads);
  TfLiteInterpreterOptionsSetErrorReporter(h->opts, reporter, h);
  if (use_gpu) {
    TfLiteGpuDelegateOptionsV2 o = TfLiteGpuDelegateOptionsV2Default();
    o.experimental_flags |= TFLITE_GPU_EXPERIMENTAL_FLAGS_CL_ONLY;  // GL needs an EGL display
    o.inference_preference = sustained ? TFLITE_GPU_INFERENCE_PREFERENCE_SUSTAINED_SPEED
                                       : TFLITE_GPU_INFERENCE_PREFERENCE_FAST_SINGLE_ANSWER;
    if (fp16) {  // same as benchmark_model's default --gpu_precision_loss_allowed=true
      o.is_precision_loss_allowed = 1;
      o.inference_priority1 = TFLITE_GPU_INFERENCE_PRIORITY_MIN_LATENCY;
      o.inference_priority2 = TFLITE_GPU_INFERENCE_PRIORITY_MIN_MEMORY_USAGE;
      o.inference_priority3 = TFLITE_GPU_INFERENCE_PRIORITY_MAX_PRECISION;
    }
    h->gpu = TfLiteGpuDelegateV2Create(&o);
    if (!h->gpu) return false;
    TfLiteInterpreterOptionsAddDelegate(h->opts, h->gpu);
  }
  // Returns null if the delegate fails to apply to the graph.
  h->interp = TfLiteInterpreterCreate(h->model, h->opts);
  if (!h->interp) return false;
  return TfLiteInterpreterAllocateTensors(h->interp) == kTfLiteOk;
}

// Load a model. With use_gpu, falls back to CPU if the delegate can't be
// created or applied -- check bg_gpu_active(). Returns null (and fills err) only
// if the model can't run at all.
BG_EXPORT bg_t* bg_create(const char* path, int use_gpu, int threads, int fp16, int sustained,
                          char* err, int errlen) {
  auto* h = new bg_t;
  h->model = TfLiteModelCreateFromFile(path);
  if (!h->model) {
    set_err(err, errlen, std::string("cannot load model ") + path);
    delete h;
    return nullptr;
  }
  if (!build(h, use_gpu, threads, fp16, sustained)) {
    std::string why = h->log.empty() ? "unknown" : h->log;
    release(h);
    if (!use_gpu || !build(h, 0, threads, 0, 0)) {
      set_err(err, errlen, "interpreter build failed: " + h->log);
      release(h);
      TfLiteModelDelete(h->model);
      delete h;
      return nullptr;
    }
    set_err(err, errlen, "GPU delegate failed, running on CPU: " + why);
  }
  return h;
}

BG_EXPORT int bg_gpu_active(const bg_t* h) { return h->gpu != nullptr; }

BG_EXPORT int bg_num_tensors(const bg_t* h, int is_output) {
  return is_output ? TfLiteInterpreterGetOutputTensorCount(h->interp)
                   : TfLiteInterpreterGetInputTensorCount(h->interp);
}

static const TfLiteTensor* tensor(const bg_t* h, int is_output, int idx) {
  if (idx < 0 || idx >= bg_num_tensors(h, is_output)) return nullptr;
  return is_output ? TfLiteInterpreterGetOutputTensor(h->interp, idx)
                   : TfLiteInterpreterGetInputTensor(h->interp, idx);
}

// type: TfLiteType enum (1 = float32, 3 = uint8, 9 = int8, 10 = float16).
// Returns the tensor's byte size, or -1 on a bad index / too many dims.
BG_EXPORT long bg_tensor_info(const bg_t* h, int is_output, int idx, int* type, int* dims,
                              int maxdims, int* ndims, float* scale, int* zero_point) {
  const TfLiteTensor* t = tensor(h, is_output, idx);
  if (!t) return -1;
  int n = TfLiteTensorNumDims(t);
  if (n > maxdims) return -1;
  for (int i = 0; i < n; ++i) dims[i] = TfLiteTensorDim(t, i);
  *ndims = n;
  *type = TfLiteTensorType(t);
  TfLiteQuantizationParams q = TfLiteTensorQuantizationParams(t);
  *scale = q.scale;
  *zero_point = q.zero_point;
  return static_cast<long>(TfLiteTensorByteSize(t));
}

BG_EXPORT int bg_set_input(bg_t* h, int idx, const void* data, size_t bytes) {
  if (idx < 0 || idx >= bg_num_tensors(h, 0)) return -1;
  TfLiteTensor* t = TfLiteInterpreterGetInputTensor(h->interp, idx);
  return TfLiteTensorCopyFromBuffer(t, data, bytes) == kTfLiteOk ? 0 : -1;
}

BG_EXPORT int bg_invoke(bg_t* h) {
  return TfLiteInterpreterInvoke(h->interp) == kTfLiteOk ? 0 : -1;
}

BG_EXPORT int bg_get_output(const bg_t* h, int idx, void* data, size_t bytes) {
  const TfLiteTensor* t = tensor(h, 1, idx);
  if (!t) return -1;
  return TfLiteTensorCopyToBuffer(t, data, bytes) == kTfLiteOk ? 0 : -1;
}

BG_EXPORT void bg_destroy(bg_t* h) {
  if (!h) return;
  release(h);
  if (h->model) TfLiteModelDelete(h->model);
  delete h;
}
