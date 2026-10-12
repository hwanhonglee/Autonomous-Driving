"""HH_261005 - Scalar/fake ABI and source tests only; no NumPy/Torch/actual cameras or policy activation."""
import ast
import hashlib
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[2]
HELPER = ROOT / "learning/portable_e2e/temporal_immutable_camera_digest_research_v1_20261005.py"
ORIGINAL = ROOT / "learning/portable_e2e/temporal_simulation_research_v2_20261001.py"
SCOPE = ("owned_episode", "val", "rig", "a" * 64)


class FakeArray:
    def __init__(self, owner, *, base=None, shape=(360, 640, 4), writeable=False, owndata=False):
        self.owner = owner
        self.base = owner if base is None else base
        self.shape, self.dtype, self.nbytes = shape, "uint8", len(owner)
        self.strides = (2560, 4, 1) if len(shape) == 3 else (1,)
        self.flags = SimpleNamespace(c_contiguous=True, writeable=writeable, owndata=owndata)
        self.__array_interface__ = {"data": (id(owner), not writeable)}

    def reshape(self, shape):
        return FakeArray(self.owner, base=self, shape=shape)

    def tobytes(self, order):
        assert order == "C"
        return bytes(bytearray(self.owner))


FAKE_NP = SimpleNamespace(ndarray=FakeArray, uint8="uint8",
                          frombuffer=lambda owner, dtype: FakeArray(owner, shape=(len(owner),)))


@pytest.fixture
def helper(monkeypatch):
    spec = importlib.util.spec_from_file_location("_immutable_digest_scalar_fixture", HELPER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, "_numpy", lambda: FAKE_NP)
    calls = []

    def original(camera):
        calls.append(camera)
        raw = camera.payload.owner if type(camera.payload) is FakeArray else bytes(camera.payload)
        return hashlib.sha256(camera.encoding.encode("ascii") + raw).hexdigest()

    monkeypatch.setattr(module, "_original_digest", original)
    return module, calls


def camera(payload, stamp=1, encoding="bgra8"):
    return SimpleNamespace(payload=payload, timestamp_ns=stamp, encoding=encoding)


def pixels():
    return bytes(bytearray(360 * 640 * 4))


def test_no_heavy_top_level_import_or_live_wiring():
    tree = ast.parse(HELPER.read_text())
    top = [node for node in tree.body if isinstance(node, (ast.Import, ast.ImportFrom))]
    assert all((node.module if isinstance(node, ast.ImportFrom) else node.names[0].name)
               in {"__future__", "collections", "contextlib"} for node in top)
    assert "setattr(" not in HELPER.read_text() and "forward_cached" not in HELPER.read_text()
    original = next(node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef)
                    and node.name == "_original_digest")
    assert ast.unparse(original.body[-1].value) == "TemporalSimulationResearchPolicyV2._camera_digest(camera)"


def test_genuine_jpeg_encoding_prefix_math_without_project_import():
    tree = ast.parse(ORIGINAL.read_text())
    fn = next(node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef) and node.name == "_camera_digest")
    fn.decorator_list = []
    namespace = {"hashlib": hashlib, "_require": lambda ok, text: ok or (_ for _ in ()).throw(ValueError(text)),
                 "ContractError": ValueError}
    exec(compile(ast.Module(body=[fn], type_ignores=[]), str(ORIGINAL), "exec"), namespace)
    assert namespace["_camera_digest"](camera(b"actual_jpeg_fixture", encoding="jpeg")) == hashlib.sha256(
        b"jpegactual_jpeg_fixture").hexdigest()


def test_default_deny_rehash_and_explicit_context_hit(helper):
    module, calls = helper
    cache = module.ImmutableCameraDigestCache()
    value = camera(FakeArray(pixels()))
    assert cache.digest(value) == cache.digest(value)
    assert len(calls) == 2 and not cache._entries
    with cache.context(SCOPE):
        first = cache.digest(value, scope=SCOPE)
        assert cache.digest(value, scope=SCOPE) == first and len(calls) == 3
        assert cache._entries[next(iter(cache._entries))][0] is value.payload
        assert cache.digest(value) == first and len(calls) == 4
    assert not cache._entries and cache.retained_owner_bytes == 0 and cache._scope is None


@pytest.mark.parametrize("mutation", ["shape", "dtype", "strides", "writeable", "contiguous", "nbytes", "owner", "pointer", "base"])
def test_cached_metadata_mutation_denied(helper, mutation):
    module, _ = helper
    array = FakeArray(pixels()); value = camera(array)
    with module.ImmutableCameraDigestCache().context(SCOPE) as cache:
        cache.digest(value, scope=SCOPE)
        if mutation == "shape": array.shape = (360, 640, 3)
        elif mutation == "dtype": array.dtype = "int8"
        elif mutation == "strides": array.strides = (1, 640, 4)
        elif mutation == "writeable": array.flags.writeable = True
        elif mutation == "contiguous": array.flags.c_contiguous = False
        elif mutation == "nbytes": array.nbytes -= 1
        elif mutation == "owner":
            array.owner = pixels(); array.base = array.owner
            array.__array_interface__["data"] = (id(array.owner), True)
        elif mutation == "pointer": array.__array_interface__["data"] = (1, True)
        else: array.base = bytearray(array.owner)
        with pytest.raises(ValueError, match="cached camera"):
            cache.digest(value, scope=SCOPE)


@pytest.mark.parametrize("kind", ["mutable", "bytearray_owner", "memoryview_owner", "unknown_owner", "deep_base", "array_subclass"])
def test_unknown_or_mutable_bgra_always_original(helper, kind):
    module, calls = helper
    array = FakeArray(pixels())
    if kind == "mutable": array.flags.writeable = True
    elif kind == "bytearray_owner": array.base = bytearray(array.owner)
    elif kind == "memoryview_owner": array.base = memoryview(array.owner)
    elif kind == "unknown_owner": array.base = object()
    elif kind == "deep_base":
        for _ in range(4): array = FakeArray(array.owner, base=array)
    else:
        class Subclass(FakeArray): pass
        array = Subclass(array.owner)
    # HH_261005 - The fake genuine fallback accepts its exact synthetic payload bytes; no actual NumPy import occurs.
    if kind == "array_subclass":
        module._original_digest = lambda source: calls.append(source) or "genuine_fallback"
    with module.ImmutableCameraDigestCache().context(SCOPE) as cache:
        value = camera(array)
        assert cache.digest(value, scope=SCOPE) == cache.digest(value, scope=SCOPE)
        assert len(calls) == 2 and not cache._entries


def test_owned_copy_retains_pixels_and_exact_bytes_owner(helper):
    module, _ = helper
    source = FakeArray(pixels(), writeable=True, owndata=True)
    result = module.own_observed_bgra(source)
    proof = module.immutable_bgra_owner(result)
    assert proof[0] == source.owner and proof[0] is not source.owner and not result.flags.writeable


def test_copy_metadata_race_denied(helper):
    module, _ = helper
    source = FakeArray(pixels(), writeable=True)
    old = source.tobytes
    def changing(order):
        source.shape = (1,)
        return old(order)
    source.tobytes = changing
    with pytest.raises(ValueError, match="metadata changed"):
        module.own_observed_bgra(source)


def test_scope_change_exception_and_reset_clear_strong_refs(helper):
    module, _ = helper
    cache = module.ImmutableCameraDigestCache()
    with pytest.raises(ValueError, match="context changed"):
        with cache.context(SCOPE):
            cache.digest(camera(FakeArray(pixels())), scope=SCOPE)
            cache.digest(camera(FakeArray(pixels())), scope=("other", *SCOPE[1:]))
    assert not cache._entries and cache.retained_owner_bytes == 0 and cache._scope is None
    with cache.context(SCOPE):
        with pytest.raises(ValueError, match="nonnested"):
            with cache.context(SCOPE): pass


def test_sixty_source_lru_and_exact_byte_budget(helper):
    module, calls = helper
    with module.ImmutableCameraDigestCache().context(SCOPE) as cache:
        owner = pixels()
        values = [camera(FakeArray(owner), stamp=i) for i in range(61)]
        for value in values: cache.digest(value, scope=SCOPE)
        assert len(cache._entries) == 60 and cache.retained_owner_bytes == 55_296_000
        assert not any(entry[0] is values[0].payload for entry in cache._entries.values())
        cache.digest(values[0], scope=SCOPE)
        assert len(calls) == 62 and len(cache._entries) == 60


@pytest.mark.parametrize("mutable", [False, True])
def test_jpeg_exact_bytes_only_and_byte_overflow(helper, mutable):
    module, calls = helper
    with module.ImmutableCameraDigestCache().context(SCOPE) as cache:
        source = camera(bytearray(b"jpeg") if mutable else b"jpeg", encoding="jpeg")
        assert cache.digest(source, scope=SCOPE) == cache.digest(source, scope=SCOPE)
        assert len(calls) == (2 if mutable else 1)
        cache.reset()
        owner = bytes(bytearray(2 * 1024**2))
        for i in range(28):
            cache.digest(camera(owner, stamp=i, encoding="jpeg"), scope=SCOPE)
        assert len(cache._entries) == 26 and cache.retained_owner_bytes <= module.MAX_OWNER_BYTES


def test_original_fallback_failure_not_suppressed(helper):
    module, _ = helper
    def denied(source):
        raise ValueError("genuine original source rejection")
    module._original_digest = denied
    with module.ImmutableCameraDigestCache().context(SCOPE) as cache:
        with pytest.raises(ValueError, match="genuine original"):
            cache.digest(camera(FakeArray(pixels(), writeable=True)), scope=SCOPE)
        assert not cache._entries


@pytest.mark.parametrize("stamp", [True, -1, 1.0])
def test_invalid_timestamp_never_reuses_cached_identity(helper, stamp):
    module, calls = helper
    value = camera(FakeArray(pixels()))
    with module.ImmutableCameraDigestCache().context(SCOPE) as cache:
        cache.digest(value, scope=SCOPE)
        value.timestamp_ns = stamp
        cache.digest(value, scope=SCOPE)
        assert len(calls) == 2


@pytest.mark.parametrize("scope", [None, [], ("e", "denied", "r", "a" * 64), ("e", "val", "r", "g" * 64)])
def test_invalid_scope_denied(helper, scope):
    module, _ = helper
    with pytest.raises(ValueError, match="valid nonnested"):
        with module.ImmutableCameraDigestCache().context(scope): pass
