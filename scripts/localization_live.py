#!/usr/bin/env python3
"""Online equivalent of the green trajectory, with no reference trajectory alignment."""
import argparse,collections,hashlib,json,math,os,queue,threading,time,uuid
from pathlib import Path
import numpy as np,cv2
from scipy.spatial.transform import Rotation
from geometry import navigation_pose,calibration,imu_from_base,pose_matrix,planar,interpolate
from fusion_policy import FusionPolicy
from scene_registry import scene_bundle
from site_identity import verify
import logging
from logging.handlers import RotatingFileHandler

def main():
    root=Path(__file__).resolve().parents[1]
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--visual-map',type=Path,required=True)
    p.add_argument('--scene',default=os.environ.get('D1_SCENE','old'))
    p.add_argument('--calibration',type=Path,default=root/'config/calibration.yaml')
    p.add_argument('--odom-frame',required=True,help='actual /Odometry header.frame_id')
    p.add_argument('--body-frame',required=True,help='actual /Odometry child_frame_id; must represent front IMU')
    p.add_argument('--max-visual-age',type=float,default=5.)
    p.add_argument('--matching-profile',choices=('strict','relaxed'),default='relaxed')
    args,ros_args=p.parse_known_args(); cv2.setNumThreads(2)
    alignment_checkpoint=os.environ.get('D1_ALIGNMENT_CHECKPOINT')
    scene=scene_bundle(root,args.scene)
    if args.scene=='old': verify(root)
    map_id=scene['map_id']
    if args.visual_map.resolve()!=scene['visual_map'].resolve():
        p.error('visual map does not belong to selected scene '+args.scene)
    (root/'work').mkdir(exist_ok=True)
    audit=logging.getLogger('visual_audit'); audit.setLevel(logging.INFO)
    handler=RotatingFileHandler(root/'work/visual_diagnostics.jsonl',maxBytes=8_000_000,backupCount=3)
    handler.setFormatter(logging.Formatter('%(message)s')); audit.addHandler(handler)
    if not 0<args.max_visual_age<=10: p.error('max visual age must be in (0,10]')
    if not args.visual_map.is_file(): p.error('missing visual_map.npz; video and trajectories cannot replace it')
    expected=hashlib.sha256(args.calibration.read_bytes()).hexdigest()
    with np.load(args.visual_map,allow_pickle=False) as z:
        if 'calibration_sha256' not in z or str(z['calibration_sha256'])!=expected:
            p.error('visual map calibration SHA256 mismatch')
    from visual_localizer import load_index,Localizer,active_matching_profile
    bank,_,_=load_index(args.visual_map); loc=Localizer(bank,args.calibration,args.matching_profile)
    K,D,_,_=calibration(args.calibration); I_B=imu_from_base(args.calibration)
    mx,my=cv2.initUndistortRectifyMap(K,D,None,K,(960,540),cv2.CV_32FC1)
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image
    from geometry_msgs.msg import PoseStamped,TransformStamped
    from std_msgs.msg import String
    from tf2_ros import TransformBroadcaster
    from cv_bridge import CvBridge
    from std_srvs.srv import Trigger
    from rcl_interfaces.srv import SetParametersAtomically
    from rcl_interfaces.msg import SetParametersResult
    from floor_routes import load_registry, floor_id, read_state, write_state
    registry=load_registry(root,scene)
    rclpy.init(args=ros_args); node=Node('d1_visual_fastlio'); cv=CvBridge(); tf=TransformBroadcaster(node)
    odpub=node.create_publisher(Odometry,'/d1/localization/odom',10)
    posepub=node.create_publisher(PoseStamped,'/d1/localization/pose6d',10)
    statpub=node.create_publisher(String,'/d1/localization/status',1)
    history=collections.deque(maxlen=2000); fusion=FusionPolicy(); session=str(uuid.uuid4())
    image_info={}; image_at=-math.inf
    latest=None; received=-math.inf; fault=''; sequence=0; image_queue=queue.Queue(maxsize=1); results=queue.Queue()
    stop=threading.Event(); pending=[]; last_query=-math.inf
    visual_info={}; sdk={}; sdk_at=-math.inf; recovery_until=0.; recovery_after=-math.inf
    graph_error=''; graph_at=-math.inf; last_state_key=None
    next_correction_log=0.
    def seconds(h): return h.stamp.sec+h.stamp.nanosec*1e-9
    def ros_seconds(): return node.get_clock().now().nanoseconds*1e-9
    def on_sdk(m):
        nonlocal sdk,sdk_at
        try:
            value=json.loads(m.data)
            if isinstance(value,dict): sdk=value; sdk_at=time.monotonic()
        except ValueError: pass
    def recovery_blocker():
        if fault: return fault
        if latest is None or time.monotonic()-received>.4: return 'fresh FAST-LIO required'
        window=[(t,O) for t,O in history if latest[0]-t<=2.5]
        if len(window)<10 or window[-1][0]-window[0][0]<2.: return 'wait for 2 seconds of stationary odometry'
        first=window[0][1]
        for _,O in window:
            d=np.linalg.inv(first)@O
            if np.linalg.norm(d[:3,3])>.03 or Rotation.from_matrix(d[:3,:3]).magnitude()>.05:
                return 'robot not stationary (FAST-LIO)'
        if graph_error: return graph_error
        if sdk_at>-math.inf:
            if time.monotonic()-sdk_at>.4: return 'SDK feedback stale'
            if sdk.get('state_fresh') is not True: return 'SDK state is not fresh'
            if sdk.get('command_zero') is not True or sdk.get('motion_pending') is not False or sdk.get('motion') not in ('Walk','BalanceStand'):
                return 'SDK must be stopped in Walk or BalanceStand'
            for k in ('reported_forward_speed','reported_lateral_speed','reported_yaw_rate'):
                v=sdk.get(k)
                if type(v) not in (int,float) or not math.isfinite(v) or abs(v)>.03: return 'SDK reports movement or invalid speed'
        return ''
    def audit_graph():
        nonlocal graph_error,graph_at,fault
        graph_at=time.monotonic(); graph_error=''
        names=set(node.get_node_names())
        if names.intersection({'d1_nav_supervisor','d1_multifloor_mission'}): graph_error='end navigation task before relocalization'
        for topic in ('/d1/sdk/supervised_cmd_vel',):
            if node.get_publishers_info_by_topic(topic): graph_error='active command publisher: '+topic
        if node.get_publishers_info_by_topic('/d1/sdk/d1_motion_status') and sdk_at==-math.inf:
            graph_error='waiting for SDK feedback'
        for topic in ('/Odometry','/d1/localization/status','/d1/localization/pose6d'):
            if len(node.get_publishers_info_by_topic(topic))>1: fault='multiple publishers: '+topic
    def request_recovery(req,res):
        nonlocal recovery_until,recovery_after
        audit_graph(); reason=recovery_blocker()
        if reason: res.success=False; res.message=reason; return res
        fusion.request(); recovery_until=time.monotonic()+30; recovery_after=ros_seconds()
        res.success=True; res.message=json.dumps(dict(message='stationary relocalization armed for 30s; keep robot still',session=session,epoch=fusion.epoch,map_id=map_id))
        return res
    def setpose(p,T):
        p.position.x,p.position.y,p.position.z=map(float,T[:3,3])
        q=Rotation.from_matrix(T[:3,:3]).as_quat()
        p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=map(float,q)
    def manual_initialize(req,res):
        try:
            if len(req.parameters)!=1 or req.parameters[0].name!='initial_pose' or req.parameters[0].value.type!=4:
                raise ValueError('initial_pose JSON 参数缺失')
            request=json.loads(req.parameters[0].value.string_value)
            if request.get('confirmed') is not True:raise ValueError('需要确认手动初值')
            audit_graph();reason=recovery_blocker()
            if reason:raise ValueError(reason)
            if request.get('session')!=session or request.get('epoch')!=fusion.epoch:
                raise ValueError('定位状态已变化，请刷新后重新标定')
            fid=floor_id(registry,request.get('floor_id'))
            from manual_initialization import alignment
            transform=alignment(request,latest[1],map_id)
            fusion.initialize_manual(transform,ros_seconds())
            pending.clear()
            write_state(root,registry,actual_floor=fid,active_nav_floor=None,target_floor=None,
                        phase='ON_FLOOR',connection_id=None,source='manual_initial_pose',
                        base_z_by_floor={**read_state(root,registry).get('base_z_by_floor',{}),fid:request['z']})
            audit.info(json.dumps(dict(event='MANUAL_INITIAL_POSE',request=request,epoch=fusion.epoch)))
            res.result=SetParametersResult(successful=True,reason=json.dumps(dict(epoch=fusion.epoch,floor_id=fid)))
        except (ValueError,KeyError,TypeError) as e:
            res.result=SetParametersResult(successful=False,reason=str(e))
        return res
    def trans(parent,child,T,stamp):
        msg=TransformStamped(); msg.header.frame_id=parent; msg.header.stamp=stamp; msg.child_frame_id=child
        msg.transform.translation.x,msg.transform.translation.y,msg.transform.translation.z=map(float,T[:3,3])
        q=Rotation.from_matrix(T[:3,:3]).as_quat()
        msg.transform.rotation.x,msg.transform.rotation.y,msg.transform.rotation.z,msg.transform.rotation.w=map(float,q)
        return msg
    def onodom(m):
        nonlocal latest,received,fault,alignment_checkpoint
        if fault: return
        t=seconds(m.header)
        if m.header.frame_id!=args.odom_frame or m.child_frame_id!=args.body_frame:
            fault='FAST-LIO frame contract changed'; return
        if abs(ros_seconds()-t)>.5: fault='FAST-LIO timestamp outside current clock'; return
        if history and t<=history[-1][0]: fault='odometry reset/out of order; restart localization'; return
        try: O=pose_matrix(m.pose.pose)@I_B
        except (ValueError,np.linalg.LinAlgError): fault='invalid odometry pose'; return
        if history and np.linalg.norm(O[:3,3]-history[-1][1][:3,3])>.5:
            fault='odometry discontinuity'; return
        if history and Rotation.from_matrix((np.linalg.inv(history[-1][1])@O)[:3,:3]).magnitude()>.8:
            fault='odometry rotation discontinuity'; return
        previous=latest; latest=(t,O,m.header.stamp); received=time.monotonic(); history.append((t,O))
        if alignment_checkpoint:
            checkpoint_path=alignment_checkpoint;alignment_checkpoint=None
            try:
                saved=json.loads(Path(checkpoint_path).read_text())
                before=np.array(saved['raw_base']);delta=np.linalg.inv(before)@O
                if (saved['map_id']!=map_id or not 0<=ros_seconds()-saved['stamp']<30 or
                    t<saved['stamp'] or np.linalg.norm(delta[:3,3])>.1 or
                    Rotation.from_matrix(delta[:3,:3]).magnitude()>.1):raise ValueError('checkpoint stale or odometry changed')
                fusion.initialize_manual(np.array(saved['alignment']),ros_seconds())
                fusion.alignment_source='restart_checkpoint'
                print('[ALIGNMENT_RESTORED] 已核对同一连续里程计，保留重启前地图对齐',flush=True)
            except (OSError,ValueError,KeyError) as e:
                print('[ALIGNMENT_NOT_RESTORED] '+str(e),flush=True)
        if fusion.T is None: return
        fusion.advance(O,t)
        M=fusion.T@O; P=planar(navigation_pose(O)); Q=planar(M)
        od=Odometry(); od.header.frame_id='nav_odom'; od.header.stamp=m.header.stamp; od.child_frame_id='nav_base'
        setpose(od.pose.pose,P)
        if previous:
            dt=t-previous[0]; d=np.linalg.inv(planar(navigation_pose(previous[1])))@P
            od.twist.twist.linear.x=float(d[0,3]/dt); od.twist.twist.linear.y=float(d[1,3]/dt)
            od.twist.twist.angular.z=float(math.atan2(d[1,0],d[0,0])/dt)
        # Covariance is not fabricated. Status carries validity; Nav2 uses odometry velocity.
        odpub.publish(od)
        tf.sendTransform([trans('map','nav_odom',Q@np.linalg.inv(P),m.header.stamp),trans('nav_odom','nav_base',P,m.header.stamp)])
        pose=PoseStamped(); pose.header.frame_id='map'; pose.header.stamp=m.header.stamp; setpose(pose.pose,M); posepub.publish(pose)
    def onimage(m):
        nonlocal last_query,image_info,image_at
        image_info=dict(width=m.width,height=m.height,encoding=m.encoding); image_at=time.monotonic()
        t=seconds(m.header)
        if fault or t-last_query<.2 or abs(ros_seconds()-t)>.5: return
        if m.width!=1920 or m.height!=1080 or m.encoding not in ('bgr8','rgb8'): return
        last_query=t
        if image_queue.full():
            try: image_queue.get_nowait()
            except queue.Empty: pass
        image_queue.put_nowait(m)
    def worker():
        while not stop.is_set():
            try: m=image_queue.get(timeout=.2)
            except queue.Empty: continue
            begin=time.monotonic()
            try:
                im=cv.imgmsg_to_cv2(m,desired_encoding='bgr8')
                im=cv2.remap(cv2.resize(im,(960,540)),mx,my,cv2.INTER_LINEAR)
                profile=active_matching_profile(fusion.T is not None,args.matching_profile)
                result=loc.locate(im,seconds(m.header),profile=profile)
                info=dict(loc.last_diagnostic); info.update(stamp=seconds(m.header),compute_sec=time.monotonic()-begin)
                results.put((seconds(m.header),result['pose'] if result else None,time.monotonic(),info))
            except Exception as e:
                results.put((seconds(m.header),None,time.monotonic(),dict(stamp=seconds(m.header),reason='visual worker: '+str(e),compute_sec=time.monotonic()-begin)))
    def tick():
        nonlocal sequence,fault,pending,visual_info,last_state_key,next_correction_log
        if time.monotonic()-graph_at>.5: audit_graph()
        fusion.expire(ros_seconds(),args.max_visual_age)
        if fusion.recovering:
            block=recovery_blocker()
            if block or time.monotonic()>recovery_until: fusion.invalidate(block or 'relocalization timed out')
        while not results.empty():
            item=results.get_nowait()
            pending.append(item)
        remaining=[]
        for t,V,wall,info in pending:
            info=dict(info,accepted=False,match_ok=V is not None,match_reason=info.get('reason','no match'),source_age=ros_seconds()-t,odom_bracket=False)
            reason=info.get('reason','no match')
            if fault: reason=fault
            elif time.monotonic()-wall>1 or not -.1<=ros_seconds()-t<=1.5: reason='visual result expired'
            elif V is not None:
                if latest is None or t>latest[0]: remaining.append((t,V,wall,info)); continue
                try:
                    O=interpolate(list(history),t); info['odom_bracket']=True
                    info['visual_xyz']=V[:3,3].tolist()
                    if fusion.recovering and t<recovery_after: reason='image predates relocalization request'
                    else:
                        info['accepted']=fusion.update(V,O,t); reason=fusion.reason
                except ValueError as e: reason='odometry bracket: '+str(e)
            info.update(reason=reason,delta_m=fusion.delta_m,delta_deg=fusion.delta_deg,epoch=fusion.epoch)
            visual_info=info; audit.info(json.dumps(dict(session=session,map_id=map_id,**info),allow_nan=False))
        pending=remaining
        sequence+=1
        tracking=(not fault and latest is not None and time.monotonic()-received<.4 and 0<=ros_seconds()-latest[0]<.5)
        visual_age=ros_seconds()-fusion.last_visual if fusion.last_visual is not None else None
        visual_fresh=visual_age is not None and 0<=visual_age<args.max_visual_age
        ready=bool(tracking and fusion.T is not None and not fusion.recovering)
        state='LOST' if not tracking else 'RELOCALIZING' if fusion.recovering else 'WAITING' if fusion.T is None else 'FUSED' if visual_fresh else 'ODOM_ONLY'
        reason=fault or ('FAST-LIO连续定位，等待有效视觉纠偏；导航可用' if state=='ODOM_ONLY' and ready else '视觉融合定位正常' if ready else fusion.reason)
        data=dict(session=session,sequence=sequence,epoch=fusion.epoch,map_id=map_id,tracking_valid=bool(tracking),tracking_fault=fault,ready=ready,
                  mode=state,alignment_source=fusion.alignment_source,recovery_required=fusion.required,relocalizing=fusion.recovering,visual=visual_info,
                  visual_fresh=visual_fresh,last_rejection_reason=fusion.last_rejection_reason,
                  correction=dict(fusion.correction,translation_rate_mps=fusion.translation_rate,rotation_rate_degps=math.degrees(fusion.rotation_rate),target_ttl_sec=fusion.target_ttl),
                  image_info=image_info,image_recent=time.monotonic()-image_at<1.,visual_age=visual_age,source_stamp=latest[0] if latest else None,reason=reason)
        key=(state,ready,fault)
        if fusion.correction['active'] and time.monotonic()>=next_correction_log:
            event=dict(event='VISUAL_CORRECTION',stamp=ros_seconds(),**data['correction'])
            audit.info(json.dumps(event));print('[VISUAL_CORRECTION] '+json.dumps(event),flush=True)
            next_correction_log=time.monotonic()+5.
        if key!=last_state_key:
            event=dict(event='LOCALIZATION_STATE',state=state,ready=ready,reason=reason,epoch=fusion.epoch,stamp=ros_seconds())
            audit.info(json.dumps(event,ensure_ascii=False));print('[LOCALIZATION] '+json.dumps(event,ensure_ascii=False),flush=True)
            last_state_key=key
        statpub.publish(String(data=json.dumps(data,allow_nan=False)))
    node.create_subscription(Odometry,'/Odometry',onodom,qos_profile_sensor_data)
    node.create_subscription(Image,'/front_camera/image',onimage,qos_profile_sensor_data)
    node.create_subscription(String,'/d1/sdk/d1_motion_status',on_sdk,1)
    node.create_service(Trigger,'/d1/localization/relocalize',request_recovery)
    node.create_service(SetParametersAtomically,'/d1/localization/manual_initialize',manual_initialize)
    node.create_timer(.1,tick); thread=threading.Thread(target=worker,daemon=True); thread.start()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        stop.set(); thread.join(timeout=2); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
if __name__=='__main__': main()

