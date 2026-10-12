# HH_260906 - Locate the repository independently of the relocated test directory.
from pathlib import Path

# HH_260906 - Keep logical source IDs, but open canonical nonsymlink files in the moved workspace.
from learning.portable_e2e.workspace_paths import workspace_path as _hh_source_path


ROOT = next(p for p in Path(__file__).resolve().parents if (p / ".git").exists())
SCRIPT = _hh_source_path(ROOT, "scripts") / "e2e" / "setup_cuda_12_8.sh"


def test_cuda_setup_includes_spconv_curand_build_and_runtime_dependencies() -> None:
    source = SCRIPT.read_text(encoding="utf-8")

    assert "libcurand-12-8" in source
    assert "libcurand-dev-12-8" in source
    assert '${toolkit_root}/include/curand.h' in source
    assert '${toolkit_root}/lib64/libcurand.so' in source
    assert "cuda-nvrtc-12-8" in source
    assert "cuda-nvrtc-dev-12-8" in source
    assert '${toolkit_root}/include/nvrtc.h' in source
    assert '${toolkit_root}/lib64/libnvrtc.so' in source
    assert "libcublas-12-8" in source
    assert "libcublas-dev-12-8" in source
    assert '${toolkit_root}/include/cublasLt.h' in source
    assert '${toolkit_root}/lib64/libcublasLt.so' in source


def test_cuda_setup_validates_curand_before_replacing_existing_root() -> None:
    source = SCRIPT.read_text(encoding="utf-8")

    validation = source.index('${staged_toolkit}/include/curand.h')
    nvrtc_validation = source.index('${staged_toolkit}/include/nvrtc.h')
    cublas_validation = source.index('${staged_toolkit}/include/cublasLt.h')
    replacement = source.index('rm -rf -- "${extract_root}"')
    assert validation < replacement
    assert nvrtc_validation < replacement
    assert cublas_validation < replacement
