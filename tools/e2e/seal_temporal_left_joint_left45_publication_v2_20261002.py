"""HH_261002 - Fresh create-only caller with an explicit no-symlink output guard; preserve the original sealer and both actual attempts."""
from __future__ import annotations
import ast
import copy
from pathlib import Path
from scripts.e2e import seal_temporal_left_joint_left45_publication_20261002 as previous

previous.pub.require(previous.pub.recorded.digest(previous.__file__)==
    "827ed4a563a318bd053a67aa13f6dc40b87c3db9345436b37333bf76743348e8", "frozen original sealer changed")
original=ast.parse(Path(previous.__file__).read_bytes()); tree=copy.deepcopy(original)
changes=[]
identities={previous.SCHEMA:previous.SCHEMA.replace('.v1','.v2'),
    'storage/experiments/plans/2026-10-02/temporal_left_joint_left45_v1/ROOT_publication_caller.json':
    'storage/experiments/plans/2026-10-02/temporal_left_joint_left45_v1/ROOT_publication_caller_v2.json'}
for node in ast.walk(tree):
    if isinstance(node,ast.Constant) and type(node.value) is str and node.value in identities:
        changes.append((node,node.value)); node.value=identities[node.value]
previous.pub.require(len(changes)==2, "only fresh schema/output identities required")
main_node=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
guard=ast.parse('pub.archive.canonical(OUTPUT)').body[0]
main_node.body.insert(0,guard)
inverse=copy.deepcopy(tree);nodes=list(ast.walk(tree));inverse_nodes=list(ast.walk(inverse))
for node,before in changes:inverse_nodes[nodes.index(node)].value=before
next(n for n in inverse.body if isinstance(n,ast.FunctionDef) and n.name=='main').body.pop(0)
previous.pub.require(ast.dump(inverse)==ast.dump(original), "another original sealing check changed")
exec(compile(ast.fix_missing_locations(tree),__file__,'exec'),globals())
