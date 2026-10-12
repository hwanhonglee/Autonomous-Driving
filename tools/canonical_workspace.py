"""HH_260906 - Explicit per-module workspace boundaries, never a global Path/open monkeypatch.

Historical source IDs and bytes remain unchanged. Only repository-owned module
root variables and Path constants are relocated. Filesystem checks still use
ordinary pathlib paths, so symlink, traversal and digest checks remain in force.
"""
import ast
import hashlib
import importlib.abc
import importlib.machinery
from pathlib import Path
import sys
import types

MOVES = {
    'src':'autonomy/src', 'autoware_e2e_vad_launch':'autonomy/autoware_e2e_vad_launch',
    'build':'autonomy/build', 'install':'autonomy/install', 'log':'autonomy/log',
    'cmake':'autonomy/cmake', 'include':'autonomy/include', 'patches':'autonomy/patches',
    'portable_e2e':'learning/portable_e2e', 'config':'learning/config', 'tests':'learning/tests',
    'scripts':'tools', 'data':'storage/assets', 'datasets':'storage/datasets',
    'artifacts':'storage/experiments', 'workflows':'docs/workflows',
}


def relocate(root, value):
    root, value = Path(root), Path(value)
    try:
        relative = value.relative_to(root)
    except ValueError:
        return value
    if not relative.parts or relative.parts[0] not in MOVES:
        return value
    if '..' in relative.parts:
        raise ValueError('parent traversal through a workspace alias is forbidden')
    return root/MOVES[relative.parts[0]]/Path(*relative.parts[1:])


def boundary(root):
    root = Path(root)
    class WorkspaceRoot(type(root)):
        # HH_260906 - Mapping happens only at this exact root, before original I/O checks.
        def __truediv__(self, other):
            value = Path(self)/other
            return relocate(root, value) if Path(self) == root else value

        def joinpath(self, *parts):
            value = Path(self).joinpath(*parts)
            return relocate(root, value) if Path(self) == root else value
    return WorkspaceRoot(root)


def adapt(namespace, root):
    root = Path(root)
    origin = namespace.get('__file__')
    if not origin or not Path(origin).absolute().is_relative_to(root/'tools'):
        return
    def convert(value):
        if isinstance(value, Path):
            mapped = boundary(root) if value == root else relocate(root, value)
            return mapped if value == root or mapped != value else value
        # HH_260906 - Preserve untouched constant identities and custom collection types.
        if type(value) in (tuple, list):
            items = [convert(item) for item in value]
            return value if all(a is b for a,b in zip(items,value)) else type(value)(items)
        if type(value) is dict:
            items = {key:convert(item) for key,item in value.items()}
            return value if all(items[key] is item for key,item in value.items()) else items
        if type(value) is set:
            items = {convert(item) for item in value}
            return value if items == value else items
        return value
    for name, value in tuple(namespace.items()):
        # HH_260906 - Ignore Python internals, modules, functions and logical string identifiers.
        if not name.startswith('__') and name.isupper():
            namespace[name] = convert(value)
    namespace['__workspace_layout_adapter__'] = {
        'path':str(Path(__file__).resolve()),
        'sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'source_bytes_modified':False,
    }


class Loader(importlib.abc.Loader):
    def __init__(self, original, root):
        self.original, self.root = original, root

    def create_module(self, spec):
        return self.original.create_module(spec)

    def exec_module(self, module):
        self.original.exec_module(module)
        adapt(module.__dict__, self.root)

    def __getattr__(self, name):
        return getattr(self.original, name)


class Finder(importlib.abc.MetaPathFinder):
    def __init__(self, root):
        self.root = Path(root)

    def find_spec(self, fullname, path=None, target=None):
        if not fullname.startswith(('scripts.e2e.', 'scripts.analysis.')):
            return None
        spec = importlib.machinery.PathFinder.find_spec(fullname, path, target)
        if spec and spec.origin and spec.loader and Path(spec.origin).is_relative_to(self.root/'tools'):
            spec.loader = Loader(spec.loader, self.root)
        return spec


def install(root):
    if not any(isinstance(finder, Finder) and finder.root == root for finder in sys.meta_path):
        sys.meta_path.insert(0, Finder(root))


def execute(path, root, argv, package=None):
    # HH_260906 - Execute the unmodified entry guard only after adapting its workspace constants.
    path = Path(path)
    tree = ast.parse(path.read_bytes(), filename=str(path))
    guards = []
    body = []
    for node in tree.body:
        test = node.test if isinstance(node, ast.If) else None
        if (isinstance(test, ast.Compare) and isinstance(test.left, ast.Name)
                and test.left.id == '__name__' and len(test.ops) == 1 and isinstance(test.ops[0], ast.Eq)
                and len(test.comparators) == 1 and isinstance(test.comparators[0], ast.Constant)
                and test.comparators[0].value == '__main__'):
            guards.append(node)
        else:
            if guards:
                raise ValueError('entry guard must be last; refusing to reorder executable statements')
            body.append(node)
    # HH_260906 - Register the actual namespace for dataclasses and restore the caller on exit.
    module = types.ModuleType('__main__')
    namespace = module.__dict__
    namespace.update(__file__=str(path), __package__=package, __builtins__=__builtins__)
    previous, previous_argv = sys.modules.get('__main__'), sys.argv
    try:
        sys.modules['__main__'] = module
        sys.argv = [str(path), *argv]
        exec(compile(ast.Module(body=body, type_ignores=[]), str(path), 'exec'), namespace)
        adapt(namespace, root)
        exec(compile(ast.Module(body=guards, type_ignores=[]), str(path), 'exec'), namespace)
    finally:
        sys.argv = previous_argv
        if previous is None:
            sys.modules.pop('__main__', None)
        else:
            sys.modules['__main__'] = previous
