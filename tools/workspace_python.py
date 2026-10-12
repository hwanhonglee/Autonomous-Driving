#!/usr/bin/env python3
"""HH_260906 - Run workspace Python from canonical files without rewriting frozen scientific sources.

The historical ``scripts`` import name remains an explicit Python namespace,
not a filesystem alias. An explicit module-root adapter relocates Path constants;
it does not rewrite recorded strings or globally intercept file I/O.
"""
from __future__ import annotations

import argparse
import importlib.util
from pathlib import Path
import runpy
import sys

# HH_260906 - Keep the path adapter outside the CARLA-owned tools package namespace.
_adapter_spec = importlib.util.spec_from_file_location('hh_canonical_workspace', Path(__file__).with_name('canonical_workspace.py'))
_adapter = importlib.util.module_from_spec(_adapter_spec)
_adapter_spec.loader.exec_module(_adapter)

ROOT = Path(__file__).resolve().parents[1]


def configure(root=ROOT):
    root = root.resolve(strict=True)
    tools = root / 'tools'
    learning = root / 'learning'
    if not (tools / 'e2e').is_dir() or not (learning / 'portable_e2e').is_dir():
        raise RuntimeError('canonical workspace layout required')
    if 'scripts' in sys.modules:
        module = sys.modules['scripts']
        if list(getattr(module, '__path__', [])) != [str(tools)]:
            raise RuntimeError('scripts namespace already loaded from another search path; use a fresh process')
    else:
        spec = importlib.util.spec_from_loader('scripts', loader=None, is_package=True)
        module = importlib.util.module_from_spec(spec)
        module.__path__ = [str(tools)]
        spec.submodule_search_locations = module.__path__
        sys.modules['scripts'] = module
    portable = sys.modules.get('portable_e2e')
    if portable is not None and Path(portable.__file__).absolute().parent != learning / 'portable_e2e':
        raise RuntimeError('portable_e2e already loaded through a legacy or foreign path; use a fresh process')
    # HH_260906 - Put learning before the working directory; do not create a top-level tools package.
    preferred = [str(learning), str(root)]
    sys.path[:] = preferred + [entry for entry in sys.path if entry not in preferred]
    _adapter.install(root)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('-m', '--module', help='Run a module, e.g. pytest or portable_e2e.train')
    parser.add_argument('arguments', nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    if args.arguments[:1] == ['--']:
        args.arguments = args.arguments[1:]
    if not args.module and not args.arguments:
        parser.error('supply -m MODULE or a Python script inside tools/')
    configure()
    if args.module:
        if args.module.startswith(('scripts.e2e.', 'scripts.analysis.')):
            spec = importlib.util.find_spec(args.module)
            if not spec or not spec.origin:
                parser.error('workspace module not found')
            _adapter.execute(Path(spec.origin), ROOT, args.arguments,
                             package=args.module.rpartition('.')[0])
        else:
            sys.argv = [args.module, *args.arguments]
            runpy.run_module(args.module, run_name='__main__', alter_sys=True)
    else:
        path = Path(args.arguments[0]).absolute()
        if path != path.resolve() or not path.is_relative_to(ROOT / 'tools') or path.suffix != '.py' or not path.is_file():
            parser.error('script must be a canonical .py file inside tools/; legacy symlinks are not accepted')
        sys.argv = [str(path), *args.arguments[1:]]
        _adapter.execute(path, ROOT, args.arguments[1:])


if __name__ == '__main__':
    main()
