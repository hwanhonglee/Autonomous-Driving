"""HH_260906 - Resolve immutable source-manifest paths without recreating root aliases."""
from pathlib import Path

SOURCE_DIRECTORIES = {
    'src': 'autonomy/src',
    'autoware_e2e_vad_launch': 'autonomy/autoware_e2e_vad_launch',
    'cmake': 'autonomy/cmake',
    'include': 'autonomy/include',
    'patches': 'autonomy/patches',
    'build': 'autonomy/build',
    'install': 'autonomy/install',
    'log': 'autonomy/log',
    'scripts': 'tools',
    'portable_e2e': 'learning/portable_e2e',
    'config': 'learning/config',
    'tests': 'learning/tests',
    'workflows': 'docs/workflows',
    'data': 'storage/assets',
    'datasets': 'storage/datasets',
    'artifacts': 'storage/experiments',
}


def source_path(root, relative):
    """HH_260906 - Preserve historical manifest bytes while reading their relocated files."""
    relative=Path(relative)
    if relative.is_absolute() or '..' in relative.parts or not relative.parts:
        raise ValueError('workspace-relative non-traversing source path required')
    first=relative.parts[0]
    path=Path(root)/SOURCE_DIRECTORIES.get(first,first)/Path(*relative.parts[1:])
    if not path.resolve().is_relative_to(Path(root).resolve()):
        raise ValueError('source path escaped workspace')
    return path


def recorded_path(root, value):
    """HH_260906 - Resolve old absolute records only inside this exact workspace root."""
    root = Path(root).absolute()
    value = Path(value)
    if not value.is_absolute() or '..' in value.parts:
        raise ValueError('absolute non-traversing recorded path required')
    try:
        relative = value.relative_to(root)
    except ValueError:
        return value
    if value == root:
        return root
    return source_path(root, relative)


def workspace_path(root, relative):
    """HH_260906 - Resolve local manifest IDs; leave genuinely frozen external trees unchanged."""
    root = Path(root)
    if (root/'learning/portable_e2e').is_dir() and (root/'tools/e2e').is_dir():
        value = Path(relative)
        return recorded_path(root, value) if value.is_absolute() else source_path(root, value)
    return root/relative
