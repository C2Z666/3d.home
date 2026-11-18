#!/usr/bin/env python3
"""Map viewer with planning preview and explicitly confirmed supervised navigation."""
import argparse, collections, io, json, math, os, threading, time, gzip
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse, parse_qs
import cv2, numpy as np, yaml
from scene_registry import load_scenes, scene_bundle, scene_list
from site_identity import verify
import map_versions

def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

class ViewerState:
    def __init__(self, root, map_id):
        self.root, self.map_id=root,map_id
        self.lock=threading.RLock(); self.values={}; self.trail=collections.deque(maxlen=1000)
        self.session=None; self.epoch=None; self.scene_name=None
    def select(self,name,map_id):
        with self.lock:
            self.scene_name=name; self.map_id=map_id
    def put(self,key,value,stamp=None):
        with self.lock: self.values[key]=(value,time.monotonic(),stamp)
    def snapshot(self,now,rosnow):
        with self.lock:
            out={'map_id':self.map_id,'ros_now':rosnow}
            for key,(value,wall,stamp) in self.values.items():
                out[key]=value;out[key+'_age']=now-wall
                if stamp is not None:out[key+'_source_age']=rosnow-stamp
            loc=out.get('localization',{})
            out['pose_current']=bool(out.get('localization_age',99)<.6 and loc.get('tracking_valid') is True and
                out.get('pose_age',99)<.6 and -.1<=out.get('pose_source_age',99)<.6 and loc.get('map_id')==self.map_id)
            out['navigation_ready']=bool(out['pose_current'] and loc.get('ready') is True)
            out['trajectory']=list(self.trail)
            return out

def map_assets(root,scene):
    maps={}
    for floor,item in scene['maps'].items():
        path=item['yaml']; c=yaml.safe_load(path.read_text())
        image=cv2.imdecode(np.frombuffer((path.parent/c['image']).read_bytes(),np.uint8),cv2.IMREAD_GRAYSCALE)
        if image is None:raise ValueError('cannot read map '+floor)
        ok,png=cv2.imencode('.png',image)
        if not ok:raise ValueError('cannot encode map '+floor)
        maps[floor]=(dict(width=image.shape[1],height=image.shape[0],resolution=c['resolution'],origin=c['origin']),png.tobytes())
    return maps

def handler(state,maps,camera,references,ros_now,scene_info,default_scene,preview=None,navigation=None):
    html=(state.root/'web/index.html').read_bytes()
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def send(self,code,body,kind):
            encodings = [part.strip().split(';') for part in self.headers.get('Accept-Encoding', '').lower().split(',')]
            accepts_gzip = any(parts[0] == 'gzip' and not any(p.strip() in ('q=0', 'q=0.0') for p in parts[1:]) for parts in encodings)
            compressed = accepts_gzip and len(body) > 1024 and (kind.startswith('application/json') or kind.startswith('text/'))
            if compressed:
                body = gzip.compress(body, compresslevel=1)
            self.send_response(code)
            self.send_header('Content-Type',kind)
            self.send_header('Cache-Control','no-store')
            self.send_header('Vary','Accept-Encoding')
            if compressed:
                self.send_header('Content-Encoding','gzip')
            self.send_header('Content-Length',str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        def do_POST(self):
            endpoint=urlparse(self.path).path
            if endpoint not in ('/preview-plan','/navigation/start','/navigation/stop','/map-switch'):
                return self.send(404,b'Not found','text/plain')
            try:
                if self.headers.get('Origin') not in (None, 'http://'+self.headers.get('Host',''), 'https://'+self.headers.get('Host','')):
                    raise ValueError('只接受当前页面发起的规划请求')
                length=int(self.headers.get('Content-Length','0'))
                if not 0<length<=4096:raise ValueError('无效的请求长度')
                data=json.loads(self.rfile.read(length))
                if not isinstance(data,dict):raise ValueError('请求必须是 JSON 对象')
                if endpoint!='/navigation/stop' and map_versions.busy(state.root):raise ValueError('地图正在切换，请稍候')
                if endpoint=='/map-switch':
                    if data.get('confirmed') is not True:raise ValueError('需要确认切换地图')
                    result=map_versions.request(state.root,data.get('version'),navigation,preview)
                elif endpoint=='/navigation/start':
                    if navigation is None:raise ValueError('导航执行未启用')
                    result=navigation.start(data)
                elif endpoint=='/navigation/stop':
                    if navigation is None:raise ValueError('导航执行未启用')
                    result=navigation.stop(data)
                else:
                    if preview is None:raise ValueError('规划预览未启用')
                    result=preview.plan(data)
                    if navigation is not None:result=navigation.register(result)
                    else:result.pop('_path',None)
                self.send(200,json.dumps(result,ensure_ascii=False,allow_nan=False).encode(),'application/json; charset=utf-8')
            except (ValueError,KeyError,TypeError) as e:
                print('[REQUEST_REJECTED] '+endpoint+' '+str(e),flush=True)
                self.send(400,json.dumps({'ok':False,'error':str(e),'code':getattr(e,'code','invalid_request'),'details':getattr(e,'details',None)},ensure_ascii=False).encode(),'application/json; charset=utf-8')
            except Exception:
                import traceback;traceback.print_exc()
                self.send(503,json.dumps({'ok':False,'error':'服务异常，请查看网页进程日志'},ensure_ascii=False).encode(),'application/json; charset=utf-8')
        def do_GET(self):
            try:
                url=urlparse(self.path);q=parse_qs(url.query)
                scene_name=q.get('scene',[default_scene])[0]
                if scene_name not in maps: scene_name=default_scene
                if url.path=='/map-versions':
                    data=map_versions.catalog(state.root)
                    result={'active':data['active'],'versions':[{'name':k,'label':v['label'],'scene':'old' if k==data['active'] else k,'navigation_enabled':v.get('navigation_enabled',True)} for k,v in data['versions'].items()],'switching':map_versions.busy(state.root)}
                    status=state.root/'work/map_switch_status.json'
                    if status.exists():result['status']=json.loads(status.read_text())
                    return self.send(200,json.dumps(result,ensure_ascii=False).encode(),'application/json; charset=utf-8')
                if url.path=='/navigation':
                    result=navigation.snapshot() if navigation else dict(ok=False,error='导航执行未启用')
                    return self.send(200,json.dumps(result,ensure_ascii=False,allow_nan=False).encode(),'application/json; charset=utf-8')
                if url.path=='/':return self.send(200,html,'text/html; charset=utf-8')
                if url.path=='/preview-costmap':
                    try:
                        if map_versions.busy(state.root):raise ValueError('地图正在切换')
                        if preview is None:raise ValueError('规划预览未启用')
                        result=preview.costmap_view(scene_name,q.get('floor',['lower'])[0])
                        return self.send(200,json.dumps(result,ensure_ascii=False,allow_nan=False).encode(),'application/json; charset=utf-8')
                    except ValueError as e:
                        return self.send(503,json.dumps({'ok':False,'error':str(e),'code':getattr(e,'code','costmap_unavailable')},ensure_ascii=False).encode(),'application/json; charset=utf-8')
                if url.path=='/state':
                    data=state.snapshot(time.monotonic(),ros_now())
                    spec=scene_info[scene_name]
                    if spec['map_id']!=state.map_id or map_versions.busy(state.root):
                        data.update(pose_current=False,navigation_ready=False,pose=None,trajectory=[],scan=None)
                    data['map_id']=spec['map_id']
                    data['selected_scene']=scene_name
                    data['active_scene']=next((n for n,v in scene_info.items() if v['map_id']==data.get('localization',{}).get('map_id')),None)
                    data['scenes']=scene_list(state.root)
                    data['entry']=spec.get('entry')
                    data['entry_identity_ok']=not spec.get('entry') or spec['map_id']==state.map_id
                    data['maps']={k:v[0] for k,v in maps[scene_name].items()}
                    versions=map_versions.catalog(state.root)
                    version=versions['versions'].get(versions['active'] if scene_name=='old' else scene_name)
                    data['floor_names']={k:Path(v['image']).stem for k,v in version['maps'].items()} if version else {}
                    site=yaml.safe_load((state.root/'config/site.yaml').read_text())
                    same=site.get('entry_map_id')==spec['map_id']
                    data['validation']={k:bool(same and site.get(k)) for k in ('maps_validated','scan_source_validated')}
                    return self.send(200,json.dumps(data,ensure_ascii=False,allow_nan=False).encode(),'application/json; charset=utf-8')
                if url.path=='/map.png':
                    floor=q.get('floor',['lower'])[0]
                    return self.send(200,maps[scene_name][floor][1],'image/png')
                if url.path=='/camera.jpg':
                    with state.lock:
                        camera['requested_at']=time.monotonic()
                        value=camera.get('image');at=camera.get('at',-math.inf);stamp=camera.get('stamp',-math.inf)
                    if value is not None and time.monotonic()-at<1.5 and -.1<=ros_now()-stamp<1.5:return self.send(200,value,'image/jpeg')
                    return self.send(404,b'No fresh image','text/plain')
                if url.path=='/reference.jpg':
                    i=int(q.get('bank',['-1'])[0])
                    if not 0<=i<len(references[scene_name]):raise ValueError('bad reference index')
                    return self.send(200,references[scene_name][i],'image/jpeg')
                self.send(404,b'Not found','text/plain')
            except (ValueError,KeyError,TypeError):self.send(400,b'Invalid request','text/plain')
    return Handler

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--host',default='127.0.0.1');p.add_argument('--port',type=int,default=8767);p.add_argument('--scene',default=os.environ.get('D1_SCENE','old'))
    args=p.parse_args();root=Path(__file__).resolve().parents[1]; registry=load_scenes(root)
    default_scene=args.scene if args.scene in registry['scenes'] else registry['default']
    if default_scene=='old':
        verify(root)
    bundles={name:scene_bundle(root,name) for name in registry['scenes']}
    maps={name:map_assets(root,bundle) for name,bundle in bundles.items()}
    state=ViewerState(root,bundles[default_scene]['map_id']);camera={}
    refs={}
    for name,bundle in bundles.items():
        refs[name]=[]
        with np.load(bundle['visual_map'],allow_pickle=False) as z:
            for i in range(len(z['stamps'])): refs[name].append(z[f'image_{i}'].tobytes())
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path as NavPath,Odometry
    from sensor_msgs.msg import LaserScan,Image
    from std_msgs.msg import String
    rclpy.init();n=rclpy.create_node('d1_readonly_web')
    def clock():return n.get_clock().now().nanoseconds*1e-9
    def stamp(h):return h.stamp.sec+h.stamp.nanosec*1e-9
    def textmsg(key):
        def cb(m):
            try:
                d=json.loads(m.data)
                if isinstance(d,dict):state.put(key,d)
            except ValueError:pass
        return cb
    def pose(m):
        if m.header.frame_id!='map':return
        v=dict(x=m.pose.position.x,y=m.pose.position.y,z=m.pose.position.z,yaw=yaw(m.pose.orientation),stamp=stamp(m.header))
        if not all(math.isfinite(a) for a in v.values()):return
        state.put('pose',v,stamp(m.header))
        with state.lock:
            loc=state.values.get('localization',({},0,None))[0];identity=(loc.get('session'),loc.get('epoch'))
            if identity!=(state.session,state.epoch):state.trail.clear();state.session,state.epoch=identity
            state.trail.append(v)
    def scan(m):
        if m.header.frame_id!='nav_base' or not -.1<=clock()-stamp(m.header)<.5:return
        pts=[]
        for i,r in enumerate(m.ranges):
            if math.isfinite(r) and m.range_min<=r<=m.range_max:
                a=m.angle_min+i*m.angle_increment;pts.append([r*math.cos(a),r*math.sin(a)])
        state.put('scan',pts,stamp(m.header))
    def path(m):
        if m.header.frame_id=='map':state.put('plan',[[v.pose.position.x,v.pose.position.y] for v in m.poses],stamp(m.header))
    def image(m):
        now=time.monotonic()
        with state.lock:
            requested_at=camera.get('requested_at',-math.inf)
        if now-requested_at>3:return
        if now-camera.get('at',-math.inf)<.5:return
        if (m.encoding not in ('rgb8','bgr8') or (m.width,m.height)!=(1920,1080) or
            m.step<m.width*3 or len(m.data)<m.step*m.height or not -.1<=clock()-stamp(m.header)<1):return
        a=np.ndarray((m.height,m.width,3),np.uint8,buffer=m.data,strides=(m.step,3,1))
        if m.encoding=='rgb8':a=cv2.cvtColor(a,cv2.COLOR_RGB2BGR)
        ok,jpeg=cv2.imencode('.jpg',cv2.resize(a,(480,270)),[cv2.IMWRITE_JPEG_QUALITY,70])
        if ok:
            with state.lock:camera.update(image=jpeg.tobytes(),at=now,stamp=stamp(m.header))
    def odom(m):
        if m.header.frame_id=='nav_odom' and m.child_frame_id=='nav_base':
            t=m.twist.twist;state.put('motion',dict(vx=t.linear.x,vy=t.linear.y,wz=t.angular.z),stamp(m.header))
    n.create_subscription(String,'/d1/localization/status',textmsg('localization'),1)
    n.create_subscription(String,'/d1/nav/scan_status',textmsg('scan_health'),1)
    n.create_subscription(String,'/d1/sdk/d1_motion_status',textmsg('sdk'),1)
    n.create_subscription(PoseStamped,'/d1/localization/pose6d',pose,10)
    n.create_subscription(LaserScan,'/d1/nav/scan',scan,qos_profile_sensor_data)
    n.create_subscription(Odometry,'/d1/localization/odom',odom,10)
    n.create_subscription(NavPath,'/plan',path,1)
    n.create_subscription(Image,'/front_camera/image',image,qos_profile_sensor_data)
    scene_info={name:{k:v for k,v in bundle.items() if k not in ('root','visual_map','maps')} for name,bundle in bundles.items()}
    from preview_planner import PreviewPlanner
    preview=PreviewPlanner(n,state,maps,default_scene)
    from web_navigation import NavigationManager
    navigation=NavigationManager(root,preview)
    server=ThreadingHTTPServer((args.host,args.port),handler(state,maps,camera,refs,clock,scene_info,default_scene,preview,navigation));server.daemon_threads=True
    thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    print(f'Navigation viewer: http://{args.host}:{args.port}; movement requires explicit web confirmation',flush=True)
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:navigation.close();server.shutdown();server.server_close();n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()

