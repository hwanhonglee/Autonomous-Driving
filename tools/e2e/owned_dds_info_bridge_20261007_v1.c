/* HH_261007 - Read genuine RMW sender metadata through the existing public RCL ABI.
 * Project-private extension, not a modification of rclpy or ROS installations.
 * Callers supply an exact Python Subscription, NEVER a pointer/address integer.
 * The owned single-thread executor and external ROOT admission remain mandatory.
 */
#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <rcl/subscription.h>
#include <rcl/error_handling.h>
#include <rmw/types.h>

typedef void * (*create_fn)(void);
typedef void (*destroy_fn)(void *);
typedef PyObject * (*convert_fn)(void *);

static int exact_imported_type(PyObject *value, const char *module_name,
                               const char *class_name)
{
    PyObject *module = PyImport_ImportModule(module_name);
    if (module == NULL) return -1;
    PyObject *cls = PyObject_GetAttrString(module, class_name);
    Py_DECREF(module);
    if (cls == NULL) return -1;
    int exact = PyType_Check(cls) && Py_TYPE(value) == (PyTypeObject *)cls;
    Py_DECREF(cls);
    if (!exact) PyErr_Format(PyExc_TypeError, "exact genuine %s.%s required", module_name, class_name);
    return exact ? 0 : -1;
}

static PyObject *checked_capsule(PyObject *metaclass, const char *name)
{
    PyObject *capsule = PyObject_GetAttrString(metaclass, name);
    if (capsule == NULL) return NULL;
    /* HH_261007 - Generated rosidl_generator_py functions use unnamed capsules. */
    if (!PyCapsule_IsValid(capsule, NULL)) {
        Py_DECREF(capsule);
        PyErr_Format(PyExc_TypeError, "actual initialized generated message capsule %s required", name);
        return NULL;
    }
    return capsule;
}

static int close_handle(PyObject *handle)
{
    /* HH_261007 - Always leave the genuine Destroyable scope while retaining any original error. */
    PyObject *kind = NULL, *value = NULL, *trace = NULL;
    PyErr_Fetch(&kind, &value, &trace);
    PyObject *closed = PyObject_CallMethod(handle, "__exit__", "OOO",
        kind ? kind : Py_None, value ? value : Py_None, trace ? trace : Py_None);
    if (closed == NULL) {
        /* HH_261007 - Preserve the take failure as context if scope cleanup itself fails. */
        PyObject *cleanup_kind = NULL, *cleanup_value = NULL, *cleanup_trace = NULL;
        PyErr_Fetch(&cleanup_kind, &cleanup_value, &cleanup_trace);
        PyErr_NormalizeException(&kind, &value, &trace);
        PyErr_NormalizeException(&cleanup_kind, &cleanup_value, &cleanup_trace);
        if (value != NULL && cleanup_value != NULL) {
            Py_INCREF(value);
            PyException_SetContext(cleanup_value, value);
        }
        Py_XDECREF(kind); Py_XDECREF(value); Py_XDECREF(trace);
        PyErr_Restore(cleanup_kind, cleanup_value, cleanup_trace);
        return -1;
    }
    Py_DECREF(closed);
    PyErr_Restore(kind, value, trace);
    return PyErr_Occurred() ? -1 : 0;
}

static PyObject *take_owned(PyObject *module, PyObject *subscription)
{
    (void)module;
    PyObject *raw = NULL, *handle = NULL, *msg_type = NULL, *pointer = NULL;
    PyObject *create_capsule = NULL, *destroy_capsule = NULL, *convert_capsule = NULL;
    PyObject *python_message = NULL, *metadata = NULL, *result = NULL;
    void *ros_message = NULL;
    destroy_fn destroy = NULL;
    bool entered = false;

    if (exact_imported_type(subscription, "rclpy.subscription", "Subscription") < 0) goto done;
    raw = PyObject_GetAttrString(subscription, "raw");
    if (raw == NULL) goto done;
    if (raw != Py_False) { PyErr_SetString(PyExc_TypeError, "raw=False typed owned subscription required"); goto done; }
    handle = PyObject_GetAttrString(subscription, "handle");
    if (handle == NULL) goto done;
    if (exact_imported_type(handle, "rclpy._rclpy_pybind11", "Subscription") < 0) goto done;
    PyObject *entered_value = PyObject_CallMethod(handle, "__enter__", NULL);
    if (entered_value == NULL) goto done;
    Py_DECREF(entered_value); entered = true;

    /* HH_261007 - The public native property is the actual rcl_subscription_t address; never supplied by caller. */
    pointer = PyObject_GetAttrString(handle, "pointer");
    if (pointer == NULL) goto done;
    if (!PyLong_CheckExact(pointer)) { PyErr_SetString(PyExc_TypeError, "native Subscription.pointer integer required"); goto done; }
    const rcl_subscription_t *native = (const rcl_subscription_t *)PyLong_AsVoidPtr(pointer);
    if (PyErr_Occurred()) goto done;
    if (native == NULL || !rcl_subscription_is_valid(native)) {
        PyErr_SetString(PyExc_RuntimeError, "actual native owned subscription invalid"); goto done;
    }
    msg_type = PyObject_GetAttrString(subscription, "msg_type");
    if (msg_type == NULL) goto done;
    create_capsule = checked_capsule((PyObject *)Py_TYPE(msg_type), "_CREATE_ROS_MESSAGE");
    destroy_capsule = checked_capsule((PyObject *)Py_TYPE(msg_type), "_DESTROY_ROS_MESSAGE");
    convert_capsule = checked_capsule((PyObject *)Py_TYPE(msg_type), "_CONVERT_TO_PY");
    if (create_capsule == NULL || destroy_capsule == NULL || convert_capsule == NULL) goto done;
    create_fn create = (create_fn)PyCapsule_GetPointer(create_capsule, NULL);
    destroy = (destroy_fn)PyCapsule_GetPointer(destroy_capsule, NULL);
    convert_fn convert = (convert_fn)PyCapsule_GetPointer(convert_capsule, NULL);
    if (create == NULL || destroy == NULL || convert == NULL || PyErr_Occurred()) goto done;
    ros_message = create();
    if (ros_message == NULL) { if (!PyErr_Occurred()) PyErr_NoMemory(); goto done; }
    rmw_message_info_t info = rmw_get_zero_initialized_message_info();
    /* HH_261007 - Keep the GIL and handle scope: rcl_take is NOT thread-safe for concurrent consumers. */
    rcl_ret_t taken = rcl_take(native, ros_message, &info, NULL);
    if (taken == RCL_RET_SUBSCRIPTION_TAKE_FAILED) { Py_INCREF(Py_None); result = Py_None; goto done; }
    if (taken != RCL_RET_OK) {
        PyErr_Format(PyExc_RuntimeError, "genuine rcl_take failed (%d): %s", taken, rcl_get_error_string().str);
        rcl_reset_error(); goto done;
    }
    python_message = convert(ros_message);
    if (python_message == NULL) { if (!PyErr_Occurred()) PyErr_SetString(PyExc_RuntimeError, "generated convert_to_py returned NULL"); goto done; }
    if (info.publisher_gid.implementation_identifier == NULL) {
        PyErr_SetString(PyExc_RuntimeError, "actual RMW GID implementation unavailable; never substitute graph identity"); goto done;
    }
    metadata = Py_BuildValue("{s:L,s:L,s:K,s:K,s:y#,s:s,s:O}",
        "source_timestamp", (long long)info.source_timestamp,
        "received_timestamp", (long long)info.received_timestamp,
        "publication_sequence_number", (unsigned long long)info.publication_sequence_number,
        "reception_sequence_number", (unsigned long long)info.reception_sequence_number,
        "publisher_gid", (const char *)info.publisher_gid.data, (Py_ssize_t)RMW_GID_STORAGE_SIZE,
        "implementation_identifier", info.publisher_gid.implementation_identifier,
        "from_intra_process", info.from_intra_process ? Py_True : Py_False);
    if (metadata == NULL) goto done;
    result = PyTuple_Pack(2, python_message, metadata);

done:
    if (ros_message != NULL && destroy != NULL) destroy(ros_message);
    if (entered && close_handle(handle) < 0) { Py_XDECREF(result); result = NULL; }
    Py_XDECREF(raw); Py_XDECREF(handle); Py_XDECREF(msg_type); Py_XDECREF(pointer);
    Py_XDECREF(create_capsule); Py_XDECREF(destroy_capsule); Py_XDECREF(convert_capsule);
    Py_XDECREF(python_message); Py_XDECREF(metadata);
    return result;
}

static PyObject *abi_info(PyObject *module, PyObject *unused)
{
    (void)module; (void)unused;
    return Py_BuildValue("{s:n,s:n,s:n,s:n}", "pointer_bytes", (Py_ssize_t)sizeof(void *),
        "rcl_subscription_bytes", (Py_ssize_t)sizeof(rcl_subscription_t),
        "rmw_message_info_bytes", (Py_ssize_t)sizeof(rmw_message_info_t),
        "publisher_gid_bytes", (Py_ssize_t)RMW_GID_STORAGE_SIZE);
}

static PyMethodDef methods[] = {
    {"take_owned", (PyCFunction)take_owned, METH_O, "HH_261007 - Genuine typed take with actual RMW publisher GID, or None."},
    {"abi_info", abi_info, METH_NOARGS, "HH_261007 - Installed-header ABI sizes; no subscription or ROS initialization."},
    {NULL, NULL, 0, NULL}
};
static struct PyModuleDef module_definition = {
    PyModuleDef_HEAD_INIT, "_hh_owned_dds_info_261007", "HH_261007 - Project-private genuine RMW take metadata.", -1, methods,
    NULL, NULL, NULL, NULL
};
PyMODINIT_FUNC PyInit__hh_owned_dds_info_261007(void)
{
    return PyModule_Create(&module_definition);
}
