from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "e2e" / "setup_cuda_12_8.sh"


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
