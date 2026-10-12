#!/usr/bin/env python3
"""HH_260906 - Publish verified offline evidence without overwriting prior runs or touching live code."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

SOURCES=('scripts/e2e/analyze_m_braking_events.py','scripts/e2e/offline_turn_speed_envelope.py',
         'scripts/e2e/research_speed_reference.py','scripts/e2e/research_agent_reference_adapter.py',
         'scripts/e2e/replay_research_speed_reference.py','scripts/e2e/verify_speed_reference_scenarios.py',
         'scripts/e2e/audit_existing_train_targets.py','portable_e2e/runtime_contract.py',
         'tests/test_offline_braking_audits.py','tests/test_offline_turn_speed_envelope.py',
         'tests/test_research_speed_reference.py','tests/test_research_agent_reference_adapter.py')
AUDITS=('legacy_train_target_audit_v3.json','prepared_moving_train_v1_target_audit.json',
        'prepared_recovery_t07_s02_common10_v1_target_audit.json',
        'prepared_recovery_ct_s01_common10_v1_target_audit.json',
        'prepared_recovery_ct_s02_common10_v1_target_audit.json',
        'prepared_recovery_t01_s01_common10_v1_target_audit.json')


def digest(path):
    """HH_260906 - Bind exact evidence bytes for local reproducibility."""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    """HH_260906 - Recompute counts and require real passing test/scenario records before publishing."""
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--input',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    episodes=[]
    for name in AUDITS:
        report=json.loads((args.input/name).read_text())
        if report['training_executed'] or report['labels_modified'] or report['held_out_payloads_opened']:
            raise ValueError('offline scope mismatch')
        episodes.extend(report['episodes'])
    if len({e['episode_id'] for e in episodes})!=len(episodes):raise ValueError('duplicate audited episode')
    total=sum(e['sample_count'] for e in episodes);passing=sum(e['passing'] for e in episodes)
    if total!=2787 or passing!=2125:raise ValueError('unexpected frozen K1 TRAIN counts')
    matrix=json.loads((args.input/'scenario_matrix_v3/summary.json').read_text())
    if matrix['cases']!=72 or matrix['passed']!=72 or matrix['failed']:raise ValueError('scenario failures remain')
    for name,expected in matrix['code_sha256'].items():
        if digest(Path(name))!=expected:raise ValueError('matrix source changed')
    suites=ET.parse(args.input/'cpu_tests_v3.xml').getroot().findall('testsuite')
    if sum(int(s.attrib['tests']) for s in suites)!=203 or any(int(s.attrib[k]) for s in suites for k in ('failures','errors','skipped')):
        raise ValueError('test record is not the expected complete pass')
    args.output.mkdir(parents=True,exist_ok=False)
    hashes={}
    files=[(args.input/name,Path('audits')/name) for name in AUDITS]
    files.extend((args.input/name,Path(name)) for name in ('braking_events.json','scenario_matrix_v3/summary.json','cpu_tests_v3.xml'))
    files.extend((Path(name),Path('source')/name) for name in SOURCES)
    for source,relative in files:
        target=args.output/relative;target.parent.mkdir(parents=True,exist_ok=True)
        expected=digest(source);shutil.copyfile(source,target)
        if digest(source)!=expected or digest(target)!=expected:raise ValueError('copy/source changed')
        hashes[str(relative)]=expected
    summary=dict(status='OFFLINE_EVIDENCE_PUBLISHED_NOT_DEPLOYED',sample_count=total,passing=passing,failed=total-passing,
        episodes=[{k:v for k,v in e.items() if k not in ('failures','passing_ids')} for e in episodes],
        cpu_test_count=203,ideal_scenario_count=72,carla_executed=False,training_executed=False,
        hashes=hashes)
    with (args.output/'summary.json').open('x') as stream:json.dump(summary,stream,indent=2)
    print(json.dumps({k:v for k,v in summary.items() if k not in ('episodes','hashes')}))


if __name__=='__main__':main()
