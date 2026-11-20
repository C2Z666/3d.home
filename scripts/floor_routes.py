"""Floor identity and directed stair connections. No ROS or motion side effects."""
import hashlib, json, math, os, time
from pathlib import Path
import yaml


def finite(value):
    return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)


def load_registry(root, scene):
    path = Path(root)/'config/floor_connections.yaml'
    data = yaml.safe_load(path.read_text(encoding='utf-8')) if path.exists() else {}
    if data.get('map_id') != scene['map_id']:
        data = dict(schema_version=1, map_id=scene['map_id'], initial_floor=next(iter(scene['maps'])),
                    floors={k:dict(label=k, map_key=k, base_z=None) for k in scene['maps']}, connections=[])
    if data.get('schema_version') != 1 or not data.get('floors'):
        raise ValueError('楼层连接配置版本或楼层表无效')
    keys = [v['map_key'] for v in data['floors'].values()]
    if len(set(keys)) != len(keys) or any(k not in scene['maps'] for k in keys):
        raise ValueError('楼层地图映射无效')
    if data['initial_floor'] not in data['floors']:
        raise ValueError('初始楼层不存在')
    ids = set()
    for c in data.get('connections', []):
        if c['id'] in ids or c['from_floor'] not in data['floors'] or c['to_floor'] not in data['floors']:
            raise ValueError('楼梯连接编号重复或楼层不存在')
        ids.add(c['id'])
        if c.get('direction') != 'up' or c.get('type') != 'stairs' or c.get('trigger') != 'mission_transition':
            raise ValueError('当前只支持显式任务触发的单向上楼连接')
        for name in ('entry', 'exit', 'preview_exit'):
            p = c.get(name)
            if p is None and name in ('exit', 'preview_exit'): continue
            if not isinstance(p, dict) or not all(finite(p.get(k)) for k in ('x', 'y', 'yaw')):
                raise ValueError('连接位姿无效: '+c['id']+'/'+name)
            if name in ('exit', 'preview_exit') and p.get('floor_id')!=c['to_floor']:
                raise ValueError('结束坐标必须明确绑定连接的目标楼层')
            if 'z' in p and not finite(p['z']):raise ValueError('楼层位姿 Z 必须为有限数值')
        line=c.get('arrival_line')
        if line is not None:
            if not isinstance(line,dict) or line.get('floor_id')!=c['to_floor'] or not all(finite(line.get(k)) for k in ('x','y','yaw','half_width_m')) or line['half_width_m']<=0:
                raise ValueError('平台到达线必须绑定目标楼层且参数有效')
            delta=math.atan2(math.sin(line['yaw']-c['entry']['yaw']),math.cos(line['yaw']-c['entry']['yaw']))
            if abs(delta)>.001:raise ValueError('平台到达线法线必须与预备点上楼方向一致')
            if c['completion'].get('combine')!='all':raise ValueError('到达线与水平稳定必须同时满足')
        ctl=c['control'];end=c['completion']
        if ctl.get('mode')!='Stair' or ctl.get('strategy')!='straight' or not isinstance(ctl.get('heading_correction'),bool):
            raise ValueError('楼梯控制要求 Stair 直行及显式纠偏开关')
        if not all(finite(v) and v>0 for k,v in ctl.items() if k not in ('mode','strategy','heading_correction')):
            raise ValueError('楼梯控制参数必须为正有限数值')
        if not 0<ctl['landing_speed']<=ctl['cruise_speed']<=.5:
            raise ValueError('楼梯速度要求 0 < landing <= cruise <= 0.5')
        from stair_monitor import monitor_config
        monitor_config(c)
        if end.get('combine') not in ('all','any') or not all(finite(v) and v>0 for k,v in end.items() if k!='combine'):
            raise ValueError('楼梯结束参数无效')
    data['revision'] = hashlib.sha256(json.dumps(data,sort_keys=True).encode()).hexdigest()
    return data


def floor_id(registry, name):
    if name in registry['floors']: return name
    for fid, spec in registry['floors'].items():
        if spec['map_key'] == name: return fid
    raise ValueError('未知楼层: '+str(name))


def read_state(root, registry):
    default = dict(map_id=registry['map_id'], actual_floor=registry['initial_floor'],
                   active_nav_floor=registry['initial_floor'], target_floor=None,
                   phase='ON_FLOOR', connection_id=None, source='initial_config', base_z_by_floor={})
    try:
        value=json.loads((Path(root)/'work/floor_state.json').read_text(encoding='utf-8'))
        if value.get('map_id')==registry['map_id']:
            default.update(value)
            for key in ('actual_floor','active_nav_floor'):
                if default[key] not in registry['floors']:default[key]=None
    except (OSError,ValueError): pass
    return default


def write_state(root, registry, **changes):
    value=read_state(root,registry);value.update(changes,updated_at=time.time())
    path=Path(root)/'work/floor_state.json';path.parent.mkdir(exist_ok=True)
    tmp=path.with_name(path.name+'.'+str(os.getpid())+'.tmp')
    tmp.write_text(json.dumps(value,ensure_ascii=False,allow_nan=False),encoding='utf-8');os.replace(tmp,path)
    return value


def reset_map_activation(root, registry):
    """Explicit map activation restarts both planners on lower, never an old task floor."""
    fid = floor_id(registry, 'lower')
    value = dict(map_id=registry['map_id'], actual_floor=fid, active_nav_floor=fid,
                 target_floor=None, phase='ON_FLOOR', connection_id=None,
                 from_floor=None, to_floor=None, reason='', source='map_activation',
                 base_z_by_floor={}, updated_at=time.time())
    path=Path(root)/'work/floor_state.json';path.parent.mkdir(exist_ok=True)
    tmp=path.with_name(path.name+'.'+str(os.getpid())+'.tmp')
    tmp.write_text(json.dumps(value,ensure_ascii=False,allow_nan=False),encoding='utf-8')
    os.replace(tmp,path)
    return value


def candidate_routes(registry, source, target, start, selected=None):
    """Parallel directed edges; estimated costs, actual Nav2 approach checked later."""
    if source==target: return [[]]
    result=[]
    def visit(floor, route, seen, at, cost):
        if floor==target:
            result.append((cost, route));return
        for c in registry.get('connections',[]):
            if not c.get('enabled') or c['from_floor']!=floor or c['to_floor'] in seen:continue
            if not route and selected and c['id']!=selected:continue
            e=c['entry'];distance=math.hypot(e['x']-at['x'],e['y']-at['y']) if at else 0.
            visit(c['to_floor'],route+[c],seen|{c['to_floor']},c.get('exit'),cost+distance+c.get('cost',8.))
    visit(source,[],{source},start,0.)
    return [r for _,r in sorted(result,key=lambda x:x[0])]


def public_state(root, registry):
    value=read_state(root,registry)
    value.update(floors=registry['floors'],connections=registry.get('connections',[]),revision=registry['revision'])
    return value
