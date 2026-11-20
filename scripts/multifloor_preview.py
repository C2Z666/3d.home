"""Compose a floor route with the existing Nav2 preview for its first segment."""
import copy
from preview_planner import PreviewPlanner, PreviewError
from floor_routes import load_registry, floor_id, read_state, candidate_routes
from scene_registry import scene_bundle


class MultiFloorPreview(PreviewPlanner):
    def plan(self,data):
        registry=load_registry(self.state.root,scene_bundle(self.state.root,self.scene))
        fs=read_state(self.state.root,registry);source=fs['actual_floor']
        if not source or source!=fs['active_nav_floor']:raise PreviewError('楼层过渡中或导航地图未就绪')
        target=floor_id(registry,data.get('floor'));transition_only=data.get('transition_only') is True
        if data.get('scene')!=self.scene:raise PreviewError('请切回当前启用场景')
        if target==source and not transition_only:return super().plan(data)
        if transition_only:
            selected=next((c for c in registry['connections'] if c['id']==data.get('connection_id') and c['enabled']),None)
            if not selected or selected['from_floor']!=source:raise PreviewError('所选楼梯不属于机器人当前楼层')
            target=selected['to_floor']
        routes=candidate_routes(registry,source,target,self.snapshot().get('pose'),data.get('connection_id'))
        if not routes:raise PreviewError('没有可用的有向楼梯连接；不会自动反向使用上楼路线')
        errors=[]
        # Each candidate's first approach must be physically reachable in Nav2.
        for connections in routes:
            if not connections:continue
            first=connections[0]
            request=dict(first['entry'],scene=self.scene,floor=registry['floors'][source]['map_key'])
            try:result=super().plan(request)
            except PreviewError as e:errors.append(str(e));continue
            original=copy.deepcopy({k:v for k,v in result.items() if k!='_path'})
            if transition_only:goal=dict(first.get('exit') or first['entry'])
            else:
                from floor_routes import finite
                if not all(finite(data.get(k)) for k in ('x','y','yaw')):raise PreviewError('目标坐标无效')
                goal={k:float(data[k]) for k in ('x','y','yaw')}
                meta=self.maps[self.scene][registry['floors'][target]['map_key']][0]
                if not (meta['origin'][0]<=goal['x']<meta['origin'][0]+meta['width']*meta['resolution'] and
                        meta['origin'][1]<=goal['y']<meta['origin'][1]+meta['height']*meta['resolution']):raise PreviewError('目标超出目标楼层地图')
            result.update(goal=goal,target_floor=target,source_floor=source,
                mission=dict(source_floor=source,target_floor=target,connections=connections,
                    transition_only=transition_only,registry_revision=registry['revision'],first_segment=original),
                note='已规划至首个预备点；虚线仅表示楼梯连接，后续平地段到层后按实测位置规划。'+
                     ('已配置平台到达线，需越线且水平稳定；出口星标仅作示意。' if all(c.get('arrival_line') for c in connections) else '未配置到达线的楼梯只采用爬升后的水平稳定判据。'))
            result['floor_reference_paths']=[]
            if not transition_only:
                endpoint=connections[-1].get('preview_exit') or connections[-1].get('exit')
                key=registry['floors'][target]['map_key']
                if endpoint:
                    try:
                        from static_floor_preview import static_path
                        points=static_path(*self.maps[self.scene][key],endpoint,goal)
                        result['floor_reference_paths'].append(dict(floor=key,points=points,reference_only=True))
                        result['note']+=' 已显示出口到目标的静态参考路线（出口附近允许35cm接入范围）；到层后重新规划执行。'
                    except ValueError as e:result['note']+=' 目标层参考路线未生成：'+str(e)+'。'
                else:result['note']+=' 出口未标定，暂不能预览目标层路线。'
            return result
        raise PreviewError('所有楼梯入口均无法规划：'+'；'.join(errors))
