"""Switch only the 2D Nav2 maps. Keep FAST-LIO/localization alive."""
import threading, time


def wait(future, timeout=6.):
    done=threading.Event();future.add_done_callback(lambda _:done.set())
    if not done.wait(timeout):raise RuntimeError('楼层地图服务超时')
    return future.result()


def activate_maps(node, root, scene, registry, fid, include_navigation=True):
    from nav2_msgs.srv import LoadMap, ClearEntireCostmap, GetCostmap
    from floor_routes import write_state
    key=registry['floors'][fid]['map_key'];path=scene['maps'][key]['yaml']
    write_state(root,registry,active_nav_floor=None)
    prefixes=['/d1/preview']+([''] if include_navigation else [])
    for prefix in prefixes:
        client=node.create_client(LoadMap,prefix+'/map_server/load_map')
        try:
            if not client.wait_for_service(timeout_sec=3.):raise RuntimeError('地图加载服务未就绪: '+prefix)
            request=LoadMap.Request();request.map_url=str(path)
            response=wait(client.call_async(request))
            if response.result!=0:raise RuntimeError('楼层地图加载失败: '+str(path))
            expected=response.map.info
        finally:node.destroy_client(client)
        names=[prefix+'/global_costmap/clear_entirely_global_costmap']
        if not prefix:names.append('/local_costmap/clear_entirely_local_costmap')
        for name in names:
            client=node.create_client(ClearEntireCostmap,name)
            try:
                if not client.wait_for_service(timeout_sec=3.):raise RuntimeError('代价地图服务未就绪: '+name)
                wait(client.call_async(ClearEntireCostmap.Request()))
            finally:node.destroy_client(client)
        client=node.create_client(GetCostmap,prefix+'/global_costmap/get_costmap')
        try:
            until=time.monotonic()+6
            while True:
                grid=wait(client.call_async(GetCostmap.Request())).map.metadata
                if (grid.size_x==expected.width and grid.size_y==expected.height and
                    abs(grid.resolution-expected.resolution)<1e-6 and
                    abs(grid.origin.position.x-expected.origin.position.x)<1e-5 and
                    abs(grid.origin.position.y-expected.origin.position.y)<1e-5):break
                if time.monotonic()>until:raise RuntimeError('代价地图尚未更新到目标楼层')
                time.sleep(.1)
        finally:node.destroy_client(client)
    write_state(root,registry,active_nav_floor=fid)
