"""Immutable map bundles, switched together with their visual bank and identity."""
import json, hashlib, os, shutil, time
from pathlib import Path

def catalog(root):
    return json.loads((Path(root)/'config/map_versions.json').read_text(encoding='utf-8'))

def busy(root):
    p=Path(root)/'work/map_switch.pending'
    return p.exists()

def validate(root, name):
    root=Path(root); c=catalog(root)
    if name not in c['versions']: raise ValueError('未知地图版本')
    v=c['versions'][name]
    for rel, expected in v['sha256'].items():
        p=(root/rel).resolve()
        if not p.is_relative_to(root.resolve()): raise ValueError('地图路径越界')
        if hashlib.sha256(p.read_bytes()).hexdigest()!=expected: raise ValueError('地图资产校验失败：'+rel)
    required=[v['visual_map'],v['calibration']]+[p for m in v['maps'].values() for p in m.values()]
    if any(p not in v['sha256'] for p in required): raise ValueError('资产缺少校验值')
    import numpy as np
    with np.load(root/v['visual_map'],allow_pickle=False) as bank:
        if str(bank['calibration_sha256'].item())!=v['sha256'][v['calibration']]:
            raise ValueError('视觉库和标定不匹配')
    return v

def atomic(path, data):
    path=Path(path); path.parent.mkdir(parents=True,exist_ok=True)
    temp=path.with_name(path.name+'.switch.tmp');temp.write_bytes(data);os.replace(temp,path)

def install(root,name):
    import yaml
    from site_identity import hashes
    root=Path(root);v=validate(root,name)
    atomic(root/'assets/output/demo/visual_map.npz',(root/v['visual_map']).read_bytes())
    atomic(root/'config/calibration.yaml',(root/v['calibration']).read_bytes())
    maps={}
    # Map activation starts Nav2 on lower; floor state must use that same map.
    for floor,m in v['maps'].items():
        meta=yaml.safe_load((root/m['yaml']).read_text());meta['image']=floor+'.pgm'
        atomic(root/f'maps/{floor}.pgm',(root/m['image']).read_bytes())
        atomic(root/f'maps/{floor}.yaml',yaml.safe_dump(meta).encode())
        maps[floor]={'yaml':f'maps/{floor}.yaml','image':f'maps/{floor}.pgm'}
    scenes=json.loads((root/'config/scenes.json').read_text())
    scenes['default']='old';scenes['scenes']['old'].update(label=v['label'],map_id=v['map_id'],maps=maps,entry=None,navigation_enabled=v.get('navigation_enabled',True))
    atomic(root/'config/scenes.json',json.dumps(scenes,ensure_ascii=False,indent=2).encode())
    site=yaml.safe_load((root/'config/site.yaml').read_text());site.update(entry=None,entry_map_id=v['map_id'])
    # Preserve existing operator validation policy; no new inspection is invented.
    site['map_version']=name
    atomic(root/'config/site.yaml',yaml.safe_dump(site,allow_unicode=True).encode())
    identity={'schema':1,'map_id':v['map_id'],'sha256':hashes(root),'entry_provenance':'Map version switch; saved entry cleared; visual initialization required.'}
    atomic(root/'config/site_identity.json',json.dumps(identity,indent=2).encode())
    c=catalog(root);c['active']=name
    atomic(root/'config/map_versions.json',json.dumps(c,ensure_ascii=False,indent=2).encode())
    from floor_routes import load_registry, reset_map_activation
    reset_map_activation(root, load_registry(root, scenes['scenes']['old']))

def request(root,name,navigation,preview):
    import subprocess, sys
    root=Path(root)
    with navigation.lock:
        if navigation.snapshot().get('active'):raise ValueError('请先结束导航再切换地图')
        for topic in ('/d1/sdk/supervised_cmd_vel','/d1/sdk/stair_cmd_vel'):
            if preview.node.get_publishers_info_by_topic(topic):raise ValueError('存在速度发布者，不能切换地图')
        snap=preview.snapshot()
        if snap.get('sdk_age',99)>1 or snap.get('sdk',{}).get('command_zero') is not True:raise ValueError('需要新鲜的零速度指令状态才能切换')
        if snap.get('sdk',{}).get('motion')!='Walk' or snap.get('sdk',{}).get('motion_pending') is not False:raise ValueError('请先恢复 Walk 并等待步态切换完成，再启用地图')
        if name not in catalog(root)['versions']:raise ValueError('未知地图版本')
        if catalog(root)['versions'][name].get('navigation_enabled',True) is False:raise ValueError('此地图包当前仅用于查看，尚未完成导航配置')
        pending=root/'work/map_switch.pending'
        try: fd=os.open(pending,os.O_CREAT|os.O_EXCL|os.O_WRONLY,0o600)
        except FileExistsError:raise ValueError('地图正在切换，请稍候')
        os.close(fd);navigation.plans.clear()
        try:
            with (root/'work/map_switch.log').open('ab') as log:
                p=subprocess.Popen([sys.executable,str(root/'scripts/switch_map.py'),name],cwd=root,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            atomic(pending,json.dumps({'pid':p.pid,'version':name,'time':time.time()}).encode())
        except Exception:
            pending.unlink(missing_ok=True);raise
        return {'ok':True,'version':name,'message':'正在切换地图与视觉库；网页将短暂重连，切换后需重新视觉定位。'}
