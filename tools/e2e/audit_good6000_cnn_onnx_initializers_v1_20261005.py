"""HH_261005 - Read bounded ONNX protobuf wire fields against original14 FP32 state; no ONNX/native/Torch imports or execution.

Field IDs follow ONNX v1.8.0 primary schema, not guessed parser output:
https://github.com/onnx/onnx/blob/v1.8.0/onnx/onnx.proto
This narrow initializer/static-IO audit is not a full ONNX semantic checker.
"""

import hashlib
import math
from pathlib import Path


def need(value, message):
    if not value: raise RuntimeError(message)


def varint(data, offset):
    value = 0
    for shift in range(0, 70, 7):
        need(offset < len(data), "truncated protobuf varint")
        byte = data[offset]; offset += 1
        need(shift != 63 or byte <= 1, "protobuf uint64 overflow")
        value |= (byte & 127) << shift
        if byte < 128: return value, offset
    raise RuntimeError("oversized protobuf varint")


def fields(data):
    need(len(data) <= 32 * 1024**2, "bounded protobuf message required")
    offset, rows = 0, []
    while offset < len(data):
        tag, offset = varint(data, offset)
        number, wire = tag >> 3, tag & 7
        need(0 < number < 2**29 and wire in (0, 1, 2, 5), "invalid/unsupported protobuf field")
        if wire == 0: value, offset = varint(data, offset)
        else:
            if wire == 2: length, offset = varint(data, offset)
            else: length = 8 if wire == 1 else 4
            need(offset + length <= len(data), "truncated protobuf field")
            value, offset = data[offset:offset + length], offset + length
        rows.append((number, wire, value))
        need(len(rows) <= 10000, "bounded protobuf field count required")
    return rows


def repeated(rows, number, wire):
    selected = [(kind, value) for field, kind, value in rows if field == number]
    need(all(kind == wire for kind, value in selected), "protobuf field wire kind differs")
    return [value for kind, value in selected]


def one(rows, number, wire, default=None):
    selected = repeated(rows, number, wire)
    need(len(selected) <= 1, "duplicate singular protobuf field")
    if not selected:
        need(default is not None, "required protobuf field missing")
        return default
    return selected[0]


def dims(rows):
    result = []
    for number, wire, value in rows:
        if number == 1:
            if wire == 0: result.append(value)
            elif wire == 2:
                offset = 0
                while offset < len(value):
                    item, offset = varint(value, offset); result.append(item)
            else: raise RuntimeError("invalid tensor dimension wire kind")
    need(result and len(result) <= 8 and all(0 < value < 2**31 for value in result), "fixed positive bounded tensor dimensions required")
    return result


def tensor(data):
    rows = fields(data)
    need(set(number for number, wire, value in rows) <= {1, 2, 4, 8, 9, 12, 14}
        and one(rows, 2, 0) == 1 and one(rows, 14, 0, 0) == 0, "inline FP32 initializer only; external/sparse/segmented data denied")
    name = one(rows, 8, 2).decode("utf8")
    need(name and len(name) <= 256 and "\0" not in name, "bounded unique initializer name required")
    shape = dims(rows)
    raw = repeated(rows, 9, 2)
    floats = [(wire, value) for number, wire, value in rows if number == 4]
    need((len(raw) == 1 and not floats) or (not raw and floats and all(wire in (2, 5) for wire, value in floats)),
        "exactly one raw or packed/unpacked FP32 data representation required")
    payload = raw[0] if raw else b"".join(value for wire, value in floats)
    need(len(payload) == math.prod(shape) * 4, "complete FP32 initializer byte count required")
    return name, dict(shape=shape, dtype="float32", raw_bytes=len(payload), raw_sha256=hashlib.sha256(payload).hexdigest())


def value_info(data):
    rows = fields(data)
    name = one(rows, 1, 2).decode("utf8")
    tensor_type = fields(one(fields(one(rows, 2, 2)), 1, 2))
    need(one(tensor_type, 1, 0) == 1, "static graph IO must be FP32")
    shape = fields(one(tensor_type, 2, 2))
    result = []
    for dimension in repeated(shape, 1, 2):
        values = fields(dimension)
        need(not any(number == 2 for number, wire, value in values), "dynamic IO dimension denied")
        result.append(one(values, 1, 0))
    need(result and all(0 < value < 2**31 for value in result), "fixed positive graph IO dimensions required")
    return dict(name=name, dtype="float32", shape=result)


def inspect(data, batch, original14):
    need(batch in (6, 30) and type(batch) is int and len(original14) == 14, "exact fixed batch and original14 state required")
    model = fields(data)
    need(not any(number == 20 for number, wire, value in model), "training/mutable graph denied")
    need(3 <= one(model, 1, 0) <= 7, "reviewed ONNX1.8 IR schema range required")
    imports = repeated(model, 8, 2)
    need(len(imports) == 1 and one(fields(imports[0]), 1, 2, b"") == b"" and one(fields(imports[0]), 2, 0) == 12,
        "exact standard ONNX opset12 required")
    graph = fields(one(model, 7, 2))
    need(not any(number in (14, 15) for number, wire, value in graph), "quantized/sparse graph denied")
    initializers = {}
    for item in repeated(graph, 5, 2):
        name, row = tensor(item)
        need(name not in initializers, "duplicate initializer name")
        initializers[name] = row
    need(set(initializers) == set(original14) and all(row == {key: original14[name][key]
        for key in ("shape", "dtype", "raw_bytes", "raw_sha256")} for name, row in initializers.items()),
        "all original14 initializer names/shapes/dtypes/raw bytes must match GOOD6000")
    inputs = [value_info(item) for item in repeated(graph, 11, 2)]
    outputs = [value_info(item) for item in repeated(graph, 12, 2)]
    need(inputs == [dict(name="normalized_images", dtype="float32", shape=[batch, 3, 180, 320])]
        and outputs == [dict(name="cnn_features", dtype="float32", shape=[batch, 60, 128])], "exact standalone CNN input/output ABI differs")
    operators = {}
    for node in repeated(graph, 1, 2):
        rows = fields(node)
        need(one(rows, 7, 2, b"") == b"", "custom operator/plugin domain denied")
        name = one(rows, 4, 2).decode("utf8")
        operators[name] = operators.get(name, 0) + 1
    return dict(batch=batch, exact14_initializers_verified=True, initializers=initializers, input=inputs[0], output=outputs[0],
        operator_counts=operators, ONNX_sha256=hashlib.sha256(data).hexdigest(),
        independent_full_ONNX_semantics_or_TensorRT_support_or_numerical_parity_verified=False)
