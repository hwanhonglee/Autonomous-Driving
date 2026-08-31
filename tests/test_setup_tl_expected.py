from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SETUP = ROOT / "scripts" / "e2e" / "setup_tl_expected.sh"


def test_setup_pins_official_jammy_libexpected_package() -> None:
    source = SETUP.read_text(encoding="utf-8")

    assert 'package="libexpected-dev"' in source
    assert 'version="1.0.0~dfsg-2"' in source
    assert (
        'checksum="b6aa7e265d3f0a3ae10744c00089bb6bb6415b4973d3bbdf04a7f6b86576da92"'
        in source
    )
    assert 'apt-get download "${package}=${version}"' in source
    assert 'dpkg-deb -x "${archive}" "${stage}"' in source


def test_setup_validates_header_before_atomic_replacement() -> None:
    source = SETUP.read_text(encoding="utf-8")

    validation = source.index('staged_header="${stage}/usr/include/tl/expected.hpp"')
    replacement = source.index('rm -rf -- "${extract_root}"')
    assert validation < replacement
    assert "TL_EXPECTED_VERSION_MAJOR" in source


def test_environment_exposes_only_the_local_native_include_layout() -> None:
    environment = (ROOT / "scripts" / "e2e" / "env.sh").read_text(
        encoding="utf-8"
    )

    assert "data/vendor/tl-expected-1.0.0/root/usr" in environment
    assert 'include/tl/expected.hpp" ]]' in environment
    assert 'CPATH="${AUTOWARE_E2E_TL_EXPECTED_ROOT}/include' in environment
    assert 'CPATH="/opt/ros/humble/include' not in environment


def test_full_build_prepares_dependency_before_sourcing_environment() -> None:
    build = (ROOT / "scripts" / "e2e" / "build_full.sh").read_text(
        encoding="utf-8"
    )

    setup = build.index("scripts/e2e/setup_tl_expected.sh")
    environment = build.index("source scripts/e2e/env.sh")
    assert setup < environment


def test_diffusion_planner_keeps_official_libexpected_contract() -> None:
    package = (
        ROOT
        / "src"
        / "universe"
        / "autoware_universe"
        / "planning"
        / "autoware_diffusion_planner"
    )
    inference = (
        package
        / "include"
        / "autoware"
        / "diffusion_planner"
        / "inference"
        / "inference.hpp"
    ).read_text(encoding="utf-8")
    package_xml = (package / "package.xml").read_text(encoding="utf-8")

    assert "#include <tl/expected.hpp>" in inference
    assert "<depend>libexpected-dev</depend>" in package_xml
