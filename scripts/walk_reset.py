#!/usr/bin/env python3
"""Explicit operator recovery to Walk; never infer a floor or resume navigation."""
import argparse
import fcntl
import json
import math
import os
import signal
import time
from pathlib import Path


def reset_floor_state(state):
    known=bool(state.get('actual_floor') and state.get('actual_floor')==state.get('active_nav_floor'))
    return dict(phase='ON_FLOOR' if known else 'NEEDS_LOCALIZATION',target_floor=None,
                connection_id=None,from_floor=None,to_floor=None,
                reason='' if known else 'Walk已恢复，实际楼层或地图仍需确认',source='manual_walk_reset')


def main():
    p=argparse.ArgumentParser();p.add_argument('--scene',default='old');p.add_argument('--owner-pid',type=int)
    args=p.parse_args();root=Path(__file__).resolve().parents[1]
    lease=(root/'work/navigation_mission.lock').open('a')
    fcntl.flock(lease,fcntl.LOCK_EX|fcntl.LOCK_NB)
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import String,Bool
    from geometry_msgs.msg import Twist
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);n=rclpy.create_node('d1_manual_walk_reset')
    state={};at=-math.inf;stopped=False;pubs=[];permit=None
    def receive(m):
        nonlocal state,at
        try:state=json.loads(m.data);at=time.monotonic()
        except ValueError:pass
    def cancel(*_):
        nonlocal stopped
        stopped=True
    signal.signal(signal.SIGINT,cancel);signal.signal(signal.SIGTERM,cancel)
    n.create_subscription(String,'/d1/sdk/d1_motion_status',receive,1)
    deadline=time.monotonic()+25.
    def check():
        if stopped:raise InterruptedError('走路重置已取消')
        if time.monotonic()>deadline:raise RuntimeError('走路重置超时')
        if args.owner_pid:os.kill(args.owner_pid,0)
    def zero():
        if permit:permit.publish(Bool(data=False))
        for pub in pubs:pub.publish(Twist())
    def tick():
        check();zero();rclpy.spin_once(n,timeout_sec=.04)
    def sdk():
        if time.monotonic()-at>.4:raise RuntimeError('SDK状态过期')
        if not all(state.get(k) is True for k in ('sdk_connected','control_held','state_fresh')) or any(state.get(k) is not False for k in ('soft_stop','hardware_stop')):
            raise RuntimeError('SDK控制权或急停状态不允许切换步态')
        return state
    def stationary(s):
        return s.get('command_zero') is True and all(isinstance(s.get(k),(int,float)) and math.isfinite(s[k]) and abs(s[k])<.03 for k in ('reported_forward_speed','reported_lateral_speed','reported_yaw_rate'))
    try:
        topics=('/d1/sdk/supervised_cmd_vel','/d1/sdk/stair_cmd_vel')
        # Wait for ROS discovery, then require external command owners to vanish.
        discovery=time.monotonic()+1.
        while time.monotonic()<discovery or not state:tick()
        until=time.monotonic()+3.
        while any(n.count_publishers(t) for t in topics):
            if time.monotonic()>until:raise RuntimeError('仍有其他运动发布者，拒绝并行切换')
            tick()
        pubs=[n.create_publisher(Twist,t,1) for t in topics]
        permit=n.create_publisher(Bool,'/d1/sdk/stair_motion_permit',1)
        action=n.create_publisher(String,'/d1/sdk/d1_action',1)
        baseline=(sdk().get('bridge_session'),sdk().get('safety_epoch'))
        def safe():
            s=sdk()
            if (s.get('bridge_session'),s.get('safety_epoch'))!=baseline:raise RuntimeError('恢复期间SDK会话改变')
            if any(n.count_publishers(t)>1 for t in topics):raise RuntimeError('恢复期间出现其他运动控制者')
            return s
        print('[WALK_RESET] 已撤销导航与上楼许可，等待零速及原模式请求结束',flush=True)
        stable=None;sequences=set()
        while True:
            tick();s=safe()
            if stationary(s) and s.get('motion_pending') is False:
                if stable is None:stable=time.monotonic()
                sequences.add(s['state_sequence'])
                if len(sequences)>=2 and time.monotonic()-stable>=.3:break
            else:stable=None;sequences.clear()
        # Request even if already Walk: clears latched Stair/gait bridge state.
        before=s['pose_request_id'];action.publish(String(data='standup'))
        print('[WALK_RESET] 请求Walk，等待SDK确认；不发送前进指令',flush=True)
        sequences.clear();stable=None
        while True:
            tick();s=safe()
            confirmed=(s.get('motion')=='Walk' and s.get('motion_pending') is False and
                s.get('pose_request_id',-1)>before and s.get('pose_confirmed_id')==s.get('pose_request_id'))
            if confirmed and stationary(s):
                if stable is None:stable=time.monotonic()
                sequences.add(s['state_sequence'])
                if len(sequences)>=2 and time.monotonic()-stable>=.3:break
            else:stable=None;sequences.clear()
        from floor_routes import load_registry,read_state,write_state
        from scene_registry import scene_bundle
        registry=load_registry(root,scene_bundle(root,args.scene));old=read_state(root,registry)
        changes=reset_floor_state(old);write_state(root,registry,**changes)
        print('[WALK_RESET] Walk已确认，过渡任务已重置；'+('保持已确认楼层' if changes['phase']=='ON_FLOOR' else '楼层未确认，请手动标定当前位置'),flush=True)
        return 0
    except (Exception,KeyboardInterrupt) as e:
        print('FAULT: '+str(e),flush=True);return 130 if stopped else 1
    finally:
        until=time.monotonic()+.5
        while time.monotonic()<until:
            zero();rclpy.spin_once(n,timeout_sec=.04)
        n.destroy_node();rclpy.try_shutdown();lease.close()


if __name__=='__main__':raise SystemExit(main())
