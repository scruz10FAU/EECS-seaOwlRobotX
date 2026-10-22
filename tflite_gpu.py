"""
tflite_gpu — TFLite + OpenCL GPU delegate on VOXL2, through ctypes.

Wraps libtflite_gpu_shim.so (voxl/beacon_gpu; built with TFLite from source in
the beacon_overlay image, at /opt/beacon_gpu/) in the subset of
tf.lite.Interpreter that TFLiteHexagonDetector uses, so the detector's
letterbox/decode/NMS are shared between CPU (tflite_runtime) and GPU.
use_gpu=False runs the same TFLite on the CPU.

Measured on 192.168.50.121 with the old host build against qrb5165-tflite 2.17
(YOLOv11n_diamond_detect_float16, fp16, sustained):
invoke ~38 ms, fully delegated ("Created 1 GPU delegate kernels"). Container
TFLite 2.19 build on 192.168.50.146 (2-class pose model, fp16): 38.1 ms median,
fully delegated, within 0.3 px of TF 2.19 on detected anchors.

Usage:
    from tflite_gpu import Interpreter
    it = Interpreter("model.tflite", "libtflite_gpu_shim.so")
    print(it.gpu_active, it.message)
"""

import ctypes as C
from typing import List

import numpy as np

# TfLiteType enum -> numpy dtype (tensorflow/lite/core/c/c_api_types.h)
_DTYPES = {1: np.float32, 2: np.int32, 3: np.uint8, 4: np.int64, 6: np.bool_,
           7: np.int16, 9: np.int8, 10: np.float16}
_MAXDIMS = 8


def _load(lib_path: str):
    L = C.CDLL(lib_path)
    L.bg_create.restype = C.c_void_p
    L.bg_create.argtypes = [C.c_char_p, C.c_int, C.c_int, C.c_int, C.c_int, C.c_char_p, C.c_int]
    L.bg_gpu_active.argtypes = [C.c_void_p]
    L.bg_num_tensors.argtypes = [C.c_void_p, C.c_int]
    L.bg_tensor_info.restype = C.c_long
    L.bg_tensor_info.argtypes = [C.c_void_p, C.c_int, C.c_int, C.POINTER(C.c_int),
                                 C.POINTER(C.c_int), C.c_int, C.POINTER(C.c_int),
                                 C.POINTER(C.c_float), C.POINTER(C.c_int)]
    L.bg_set_input.argtypes = [C.c_void_p, C.c_int, C.c_void_p, C.c_size_t]
    L.bg_invoke.argtypes = [C.c_void_p]
    L.bg_get_output.argtypes = [C.c_void_p, C.c_int, C.c_void_p, C.c_size_t]
    L.bg_destroy.argtypes = [C.c_void_p]
    return L


class Interpreter:
    """Minimal tf.lite.Interpreter look-alike. Tensor "index" is the position in
    the input or output list (inputs and outputs are numbered separately)."""

    def __init__(self, model_path: str, lib_path: str, num_threads: int = 4,
                 use_gpu: bool = True, fp16: bool = True, sustained: bool = True):
        self._lib = _load(lib_path)
        err = C.create_string_buffer(4096)
        self._h = self._lib.bg_create(model_path.encode(), int(use_gpu), int(num_threads),
                                      int(fp16), int(sustained), err, len(err))
        self.message = err.value.decode(errors="replace")
        if not self._h:
            raise RuntimeError(f"tflite_gpu: {self.message}")
        self.gpu_active = bool(self._lib.bg_gpu_active(self._h))
        self._inputs = self._details(0)
        self._outputs = self._details(1)

    def _details(self, is_output: int) -> List[dict]:
        out = []
        for i in range(self._lib.bg_num_tensors(self._h, is_output)):
            t, n, z = C.c_int(), C.c_int(), C.c_int()
            s = C.c_float()
            dims = (C.c_int * _MAXDIMS)()
            nbytes = self._lib.bg_tensor_info(self._h, is_output, i, C.byref(t), dims, _MAXDIMS,
                                              C.byref(n), C.byref(s), C.byref(z))
            if nbytes < 0:
                raise RuntimeError(f"tflite_gpu: bad tensor {'out' if is_output else 'in'}[{i}]")
            if t.value not in _DTYPES:
                raise RuntimeError(f"tflite_gpu: unsupported TfLiteType {t.value}")
            out.append({"index": i, "shape": np.array(dims[:n.value], dtype=np.int32),
                        "dtype": _DTYPES[t.value], "quantization": (s.value, z.value),
                        "nbytes": int(nbytes)})
        return out

    def allocate_tensors(self):
        pass  # done in bg_create

    def get_input_details(self) -> List[dict]:
        return [dict(d) for d in self._inputs]

    def get_output_details(self) -> List[dict]:
        return [dict(d) for d in self._outputs]

    def set_tensor(self, index: int, value: np.ndarray):
        d = self._inputs[index]
        a = np.ascontiguousarray(value, dtype=d["dtype"])
        if a.nbytes != d["nbytes"]:
            raise ValueError(f"tflite_gpu: input {index} expects {tuple(d['shape'])}, "
                             f"got {value.shape}")
        if self._lib.bg_set_input(self._h, index, a.ctypes.data, a.nbytes) != 0:
            raise RuntimeError("tflite_gpu: set_input failed")

    def invoke(self):
        if self._lib.bg_invoke(self._h) != 0:
            raise RuntimeError("tflite_gpu: invoke failed")

    def get_tensor(self, index: int) -> np.ndarray:
        d = self._outputs[index]
        a = np.empty(tuple(d["shape"]), dtype=d["dtype"])
        if self._lib.bg_get_output(self._h, index, a.ctypes.data, a.nbytes) != 0:
            raise RuntimeError("tflite_gpu: get_output failed")
        return a

    def __del__(self):
        h, self._h = getattr(self, "_h", None), None
        if h:
            self._lib.bg_destroy(h)
