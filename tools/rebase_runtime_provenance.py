"""HH_260906 - Refresh active build receipts only when every non-location fact is unchanged."""
import hashlib
import json
import os
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from scripts.e2e import mission_planner_build_provenance as mission
from scripts.e2e import vad_object_safety_build_provenance as vad

MOVES={name:'autonomy/'+name for name in ('src','patches','build','install','log')}


def relocate(value, is_path=False):
    fields={'path','installed_path','build_path','install_path','ros_package_prefix','source_paths','runtime_resolution'}
    if isinstance(value,dict):return {key:relocate(item,is_path or key in fields) for key,item in value.items()}
    if isinstance(value,list):return [relocate(item,is_path) for item in value]
    if not isinstance(value,str) or not is_path:return value
    prefix=str(ROOT)+'/' if value.startswith(str(ROOT)+'/') else ''
    relative=value[len(prefix):]
    first,slash,rest=relative.partition('/')
    return prefix+MOVES[first]+(slash+rest if slash else '') if first in MOVES else value


def payload_without_digest(value):
    value=dict(value)
    recorded=value.pop('source_runtime_contract_sha256',None)
    if recorded is not None:
        actual=hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':')).encode()).hexdigest()
        if actual!=recorded:raise ValueError('invalid original aggregate digest')
    return value


def main():
    lock=mission.acquire_workspace_lock()
    try:
        pending=[]
        for module in (mission,vad):
            before=module.PROVENANCE.read_bytes();old=json.loads(before);current=module.current_contract()
            if relocate(payload_without_digest(old))!=payload_without_digest(current):
                raise ValueError('non-path runtime fact changed: '+module.__name__)
            pending.append((module.PROVENANCE,before,current))
        archive=ROOT/'.workspace_archive/runtime_provenance_paths_20260922'
        archive.mkdir(exist_ok=False)
        records=[]
        for path,before,current in pending:
            (archive/path.name).write_bytes(before)
            after=(json.dumps(current,indent=2,sort_keys=True)+'\n').encode()
            staged=path.with_name(path.name+'.layout-staged')
            with staged.open('xb') as stream:stream.write(after)
            os.replace(staged,path)
            records.append(dict(path=str(path.relative_to(ROOT)),before_sha256=hashlib.sha256(before).hexdigest(),
                                after_sha256=hashlib.sha256(after).hexdigest(),non_location_facts_unchanged=True))
        (archive/'changes.json').write_text(json.dumps(records,indent=2))
        print('RUNTIME_RECEIPTS_REBASED_LOCATION_ONLY',len(records))
    finally:
        if lock is not None:lock.close()


if __name__=='__main__':main()
