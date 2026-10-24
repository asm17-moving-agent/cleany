#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include <algorithm>
#include <cmath>
#include <cstring>

namespace {
struct Buffer {
  Py_buffer value{};
  bool acquired = false;
  ~Buffer() { if (acquired) PyBuffer_Release(&value); }
  bool read(PyObject * object, const char * format, int dimensions) {
    if (PyObject_GetBuffer(object, &value, PyBUF_FORMAT | PyBUF_C_CONTIGUOUS) != 0) {
      return false;
    }
    acquired = true;
    if (value.ndim != dimensions || std::strcmp(value.format, format) != 0) {
      PyErr_SetString(PyExc_ValueError, "Unexpected depth-filter buffer shape or dtype");
      return false;
    }
    return true;
  }
};

PyObject * filter(PyObject *, PyObject * args) {
  PyObject * objects[5];
  if (!PyArg_ParseTuple(args, "OOOOO", &objects[0], &objects[1], &objects[2],
      &objects[3], &objects[4])) { return nullptr; }
  Buffer rays, origins, rotations, halves, visible;
  if (!rays.read(objects[0], "d", 2) || !origins.read(objects[1], "d", 2) ||
      !rotations.read(objects[2], "d", 3) || !halves.read(objects[3], "d", 2) ||
      !visible.read(objects[4], "?", 1)) { return nullptr; }
  const auto count = rays.value.shape[0], boxes = origins.value.shape[0];
  if (rays.value.shape[1] != 3 || origins.value.shape[1] != 3 ||
      rotations.value.shape[0] != boxes || rotations.value.shape[1] != 3 ||
      rotations.value.shape[2] != 3 || halves.value.shape[0] != boxes ||
      halves.value.shape[1] != 3 || visible.value.shape[0] != count) {
    PyErr_SetString(PyExc_ValueError, "Inconsistent depth-filter dimensions");
    return nullptr;
  }
  PyObject * output = PyBytes_FromStringAndSize(nullptr, count);
  if (!output) { return nullptr; }
  auto * keep = PyBytes_AS_STRING(output);
  const auto * initial = static_cast<const unsigned char *>(visible.value.buf);
  const auto * delta = static_cast<const double *>(rays.value.buf);
  const auto * starts = static_cast<const double *>(origins.value.buf);
  const auto * rotation = static_cast<const double *>(rotations.value.buf);
  const auto * half = static_cast<const double *>(halves.value.buf);
  Py_BEGIN_ALLOW_THREADS
  for (Py_ssize_t i = 0; i < count; ++i) {
    keep[i] = initial[i] ? 1 : 0;
    if (!keep[i]) { continue; }
    for (Py_ssize_t box = 0; box < boxes; ++box) {
      double enter = 0., leave = 1.;
      bool outside = false;
      for (int axis = 0; axis < 3; ++axis) {
        const double origin = starts[box*3+axis];
        const double direction = delta[i*3]*rotation[box*9+axis] +
          delta[i*3+1]*rotation[box*9+3+axis] +
          delta[i*3+2]*rotation[box*9+6+axis];
        const double bound = half[box*3+axis];
        if (std::abs(direction) < 1e-12) {
          if (std::abs(origin) > bound) { outside = true; break; }
        } else {
          const double low = (-bound-origin)/direction;
          const double high = (bound-origin)/direction;
          enter = std::max(enter, std::min(low, high));
          leave = std::min(leave, std::max(low, high));
          if (enter > leave) { outside = true; break; }
        }
      }
      if (!outside && enter <= leave) { keep[i] = 0; break; }
    }
  }
  Py_END_ALLOW_THREADS
  return output;
}
PyMethodDef methods[] = {{"filter", filter, METH_VARARGS,
  "Closed-segment self occlusion on validated contiguous buffers."}, {nullptr, nullptr, 0, nullptr}};
PyModuleDef module = {PyModuleDef_HEAD_INIT, "_cleany_depth_filter", nullptr, -1, methods};
}  // namespace
PyMODINIT_FUNC PyInit__cleany_depth_filter() { return PyModule_Create(&module); }
