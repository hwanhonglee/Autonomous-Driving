"""HH_261004 - Bind fresh authenticated buffer-snapshot recording and count38 restoration without changing driving math."""

from contextlib import contextmanager
import ast
import hashlib
import inspect
from pathlib import Path
import textwrap
from types import FunctionType

from . import collect_temporal_route_joint_town03_straight_full209_v2_20261004 as previous
from . import collect_temporal_route_joint_town03_straight_full209_v3_20261004 as archived
from . import parallel_exact_jpeg_after_ack_v2_20261004 as jpeg
from . import two_phase_owned_signal_restoration_v2_20261004 as signals


SCHEMA=previous.SCHEMA
OWNER_SCHEMA="portable_e2e.owned_route_joint_town03_known_dev_straight_full209.20261004.v4"
PARENT_SHA256="d35ee2942e63d9a24a6a19340b59ea998a85c11f4d5a21d50a011366922b1dac"
ARCHIVED_V3_SHA256="3a1822e37adc92af44115291fa47441f6fbd408cd0a57296bd514693ea23bf65"
SIGNALS_SHA256="e09d031ba4303d6c4968fea95ac7fcd99c9d3b24eb23823b6b0e8f2bc9f17b34"
JPEG_SHA256="b99709556d4e6e632626b388924b3b367cee5e7f88140cc4f9d32b9bba640259"
_active=False
_guard_module=previous.previous.callback.previous
_original_owner_guard=_guard_module._actual_fixture_owner_schema
_signal_class=signals.TwoPhaseGreenSignalFixture
_signal_constructor=_signal_class.__init__
_signal_base=_signal_class.__bases__[0]
_base_constructor=_signal_base.__init__


def __getattr__(name):return getattr(previous,name)


def verify_frozen():
    # HH_261004 - Authenticate both prior captures and the newly reviewed signal protocol; no old failure is overwritten.
    for module,sha in ((previous,PARENT_SHA256),(archived,ARCHIVED_V3_SHA256),(signals,SIGNALS_SHA256),(jpeg,JPEG_SHA256)):
        path=Path(module.__file__)
        previous.frozen._require(path.resolve()==path and not path.is_symlink() and path.is_file()
            and hashlib.sha256(path.read_bytes()).hexdigest()==sha,"frozen full209 or signal revision changed")
    previous.verify_frozen();jpeg.verify_frozen();signals.verify_frozen()


def _actual_fixture_owner_schema():
    """HH_261004 - Audit the genuinely executed inherited constructor, never add an unused owner-schema alias."""
    fixture=previous.fixture;require=previous.frozen._require
    current=fixture.GreenSignalFixture
    if current is _signal_base:
        require(current.__init__ is _base_constructor,"unknown original fixture constructor")
        return _original_owner_guard()
    require(current is _signal_class and current.__init__ is _signal_constructor
        and current.__bases__==(_signal_base,) and _signal_base.__init__ is _base_constructor,
        "only the exact pinned two-phase class and genuine original base constructor are admitted")
    owner=fixture.assert_owned_capture.__globals__.get("OWNER_SCHEMA")
    constructor_owner=_base_constructor.__globals__.get("OWNER_SCHEMA")
    require(owner==constructor_owner,"fixture admission and genuinely executed super-constructor owner schemas differ")
    return owner


def _verify_constructor_seam():
    # HH_261004 - The immutable wrapper must call lexical super().__init__; its original class closure is never cloned or changed.
    node=ast.parse(textwrap.dedent(inspect.getsource(_signal_constructor))).body[0]
    previous.frozen._require(sum(isinstance(value,ast.Call)
        and ast.unparse(value)=="super().__init__(*args, **kwargs)" for value in ast.walk(node))==1
        and inspect.getclosurevars(_signal_constructor).nonlocals.get("__class__") is _signal_class,
        "pinned two-phase constructor must execute its genuine lexical super initializer")


@contextmanager
def activated(model,args):
    """HH_261004 - Install only fresh helpers before genuine V2 capture, then assert actual recorder/factory ownership."""
    global _active
    verify_frozen();_verify_constructor_seam()
    previous.frozen._require(not _active and not previous._active
        and _guard_module._actual_fixture_owner_schema is _original_owner_guard,
        "fresh full209 V4 collector cannot nest or replace an unknown constructor guard")
    saved=previous.OWNER_SCHEMA;_active=True
    try:
        previous.OWNER_SCHEMA=OWNER_SCHEMA
        _guard_module._actual_fixture_owner_schema=_actual_fixture_owner_schema
        with jpeg.activated(),signals.activated(),previous.activated(model,args):
            recorder=jpeg.assert_runtime_recorder(previous.collector)
            previous.frozen._require(recorder.output==args.output.with_name(args.output.name+".partial")
                and previous.fixture.GreenSignalFixture is signals.TwoPhaseGreenSignalFixture
                and previous.fixture.assert_owned_capture.__globals__["OWNER_SCHEMA"]==OWNER_SCHEMA
                and previous.SCHEMA==SCHEMA,"actual V4 recorder/fixture/owner/raw ABI differs")
            yield
    finally:
        _guard_module._actual_fixture_owner_schema=_original_owner_guard
        previous.OWNER_SCHEMA=saved;_active=False


def audit_parallel_recordings(recordings,cameras,owned_sources,callback,timings,states,*,episode,raw_pins,digest):
    """HH_261004 - Retain every frozen source/join/write/next-tick gate, adding complete immutable snapshot proofs."""
    function=archived.audit_parallel_recordings
    private=FunctionType(function.__code__,{**function.__globals__,"jpeg":jpeg},
        function.__name__,function.__defaults__,function.__closure__)
    private.__kwdefaults__=dict(function.__kwdefaults__ or {})
    result=private(recordings,cameras,owned_sources,callback,timings,states,episode=episode,raw_pins=raw_pins,digest=digest)
    require=previous.frozen._require
    matched=0
    for record,camera in zip(recordings,cameras):
        for role,row in zip(jpeg.CAMERA_ORDER,record["parallel_encoding_research"]["encode_rows"]):
            abi=row.get("source_ABI",{});size=640*360*4
            require(type(abi) is dict and type(abi.get("image_type")) is str
                and abi.get("raw_data_type") in ("builtins.bytes","builtins.bytearray","builtins.memoryview")
                and abi.get("buffer_protocol_observed") is True and type(abi.get("readonly")) is bool
                and abi.get("contiguous") is True and abi.get("c_contiguous") is True
                and type(abi.get("itemsize")) is int and abi["itemsize"]==1
                and abi.get("nbytes")==abi.get("source_len")==size
                and type(abi.get("format")) is str and type(abi.get("ndim")) is int
                and type(abi.get("shape")) is list and type(abi.get("strides")) is list,
                "complete actual original contiguous byte-buffer ABI required")
            actual=dict(frame=camera["frame"],timestamp=camera["source_timestamps"][role],width=640,height=360)
            require(all(abi.get(key)==value for key,value in actual.items())
                and row.get("source_after_encoding_metadata")==actual,
                "actual original frame/stamp/geometry cannot be retagged by a private encoding view")
            require(all(row.get(key) is True for key in ("private_snapshot_immutable","source_stable_through_encoding",
                "original_image_identity_preserved","private_snapshot_used_only_for_encoder","source_metadata_unchanged_after_encoding"))
                and type(row.get("immutable_snapshot_byte_count")) is int and row["immutable_snapshot_byte_count"]==size
                and type(row.get("private_encoding_image_object_id")) is int and row["private_encoding_image_object_id"]>0
                and row["private_encoding_image_object_id"]!=row["image_object_id"],
                "actual encoder must consume an equal immutable private snapshot, never replace its source image")
            require(row.get("source_pre_snapshot_BGRA_encoding_prefixed_sha256")
                ==row.get("immutable_snapshot_BGRA_encoding_prefixed_sha256")
                ==row.get("source_post_encoding_BGRA_encoding_prefixed_sha256")
                ==row["source_BGRA_encoding_prefixed_sha256"],
                "full pre/snapshot/post bytes must equal the genuine selected callback SHA")
            clocks=[row[key] for key in ("encoding_started_monotonic_ns","source_snapshot_started_monotonic_ns",
                "source_snapshot_finished_monotonic_ns","encoding_finished_monotonic_ns")]
            require(all(type(value) is int and value>0 for value in clocks) and clocks==sorted(clocks),
                "snapshot must occur inside actual joined worker encode clocks")
            matched+=1
    require(matched==6*len(cameras),"every actual snapshot including failed-tail denominator required")
    result.update(snapshot_schema=jpeg.SCHEMA,all_original_source_ABI_and_pre_snapshot_post_hashes_checked=True,
        actual_immutable_snapshot_count=matched,actual_source_class_in_frozen_V3_claimed=False,
        frozen_V3_zero_JPEG_failure_preserved=True)
    return result
