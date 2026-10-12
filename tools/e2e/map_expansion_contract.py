"""HH_260906 - Require additive TRAIN expansion and byte-bound unchanged DEV."""

from collections import Counter


TOWN03_ROUTE_SHA256 = frozenset({
    "aee6b0fd6e5a6dfa882cd172b4cb10c2e0865d2729d3233f5f121bb4be0674d4",
    "8aa4040617f733e5df11910d3ac8113307e05a53334f29bc3387eb1370e2bf3c",
})
AI_V2_PARENT_SHA256 = "e9805574a252e72f8e81fcd434156da7782b25e8e965b8078f976cf4f234892c"


def validate_expansion(manifest,parent,parent_sha256):
    """HH_260906 - Reject changed old examples or evaluation splits before any GPU optimization."""
    if manifest.get('parent_manifest_sha256')!=parent_sha256:
        raise ValueError('parent manifest changed')
    if manifest['splits']['dev']!=parent['splits']['dev']:
        raise ValueError('DEV must remain byte-bound and unchanged')
    old=parent['splits']['train'];new=manifest['splits']['train']
    if (new['count']<=old['count'] or len(new['source_records'])!=new['count']
            or new['source_records'][:old['count']]!=old['source_records']):
        raise ValueError('old TRAIN records must remain an exact prefix')
    if any(new['source_hashes'].get(k)!=v for k,v in old['source_hashes'].items()):
        raise ValueError('old TRAIN sources changed')
    proof=manifest.get('expansion',{})
    if (proof.get('added_frames')!=new['count']-old['count']
            or proof.get('owner_exit_code')!=0 or proof.get('collision_events')!=0
            or proof.get('source_bytes_unchanged_and_archived') is not True
            or proof.get('physical_capture') is not True or proof.get('dev_used_for_training') is not False):
        raise ValueError('incomplete expansion qualification')
    if proof.get('split_mode') == 'town_disjoint_train_v1':
        # HH_260906 - Town coordinate frames are unrelated; require source-town and exact-route proof instead of fake XY distances.
        rows=new['source_records'][old['count']:]
        routes=proof.get('routes',[])
        counts=Counter(row.get('route_sha256') for row in rows)
        old_pixels=old.get('class_pixels',[])
        added_pixels=([sum(route['added_class_pixels'][i] for route in routes)
                       for i in range(7)]
                      if len(routes)==2 and all(len(route.get('added_class_pixels',[]))==7
                                                for route in routes) else [])
        if (parent_sha256!=AI_V2_PARENT_SHA256 or old['count']!=497
                or parent['splits']['dev']['count']!=118
                or parent['splits']['dev']['source_summary'].get('town')!='Town01'
                or old['source_summary'].get('town')!='Town01'
                or len(routes)!=2 or {r.get('route_sha256') for r in routes}!=TOWN03_ROUTE_SHA256
                or any(r.get('town')!='Town03' or r.get('independent_retained_frames',0)<120
                       or r.get('added_frames',0)<120 or r.get('owner_exit_code')!=0
                       or counts[r.get('route_sha256')]!=r.get('added_frames') for r in routes)
                or sum(r['added_frames'] for r in routes)!=len(rows)
                or len(old_pixels)!=7 or len(new.get('class_pixels',[]))!=7
                or len(added_pixels)!=7
                or any(new['class_pixels'][i]!=old_pixels[i]+added_pixels[i] for i in range(7))
                or proof.get('parent_town')!='Town01' or proof.get('added_town')!='Town03'
                or proof.get('held_out_town')!='Town07' or proof.get('same_town_dedup_m')!=0.25
                or proof.get('cross_town_xy_compared') is not False
                or proof.get('minimum_dev_distance_m') is not None
                or proof.get('minimum_old_train_distance_m') is not None
                or any(row.get('source_town')!='Town03'
                       or row.get('route_sha256') not in TOWN03_ROUTE_SHA256 for row in rows)):
            raise ValueError('cross-Town TRAIN split proof incomplete')
        return
    if proof.get('minimum_dev_distance_m',0)<80:
        raise ValueError('new same-Town route overlaps DEV corridor')
