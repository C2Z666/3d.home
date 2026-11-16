#!/usr/bin/env python3
"""Request stationary visual reinitialization; does not stop the robot."""
import argparse, json, time
def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--confirm-stopped', action='store_true', required=True)
    p.parse_args()
    import rclpy
    from std_srvs.srv import Trigger
    from std_msgs.msg import String
    rclpy.init(); n=rclpy.create_node('d1_request_relocalization')
    state={}; at=0.
    def cb(m):
        nonlocal state, at
        state=json.loads(m.data); at=time.monotonic()
    n.create_subscription(String,'/d1/localization/status',cb,1)
    c=n.create_client(Trigger,'/d1/localization/relocalize')
    try:
        if not c.wait_for_service(timeout_sec=5): raise RuntimeError('localization service unavailable')
        f=c.call_async(Trigger.Request()); rclpy.spin_until_future_complete(n,f,timeout_sec=5)
        if not f.done(): raise RuntimeError('request timeout')
        res=f.result(); print(res.message,flush=True)
        if not res.success: return 1
        expected=json.loads(res.message)
        identity=lambda s:tuple(s.get(k) for k in ('session','epoch','map_id'))
        end=time.monotonic()+35
        while time.monotonic()<end:
            rclpy.spin_once(n,timeout_sec=.1)
            if time.monotonic()-at<.5 and identity(state)==identity(expected) and state.get('ready') and not state.get('relocalizing'):
                print('READY: relocalization complete; inspect map position before a new navigation task'); return 0
            if identity(state)==identity(expected) and state.get('recovery_required') and not state.get('relocalizing'):
                raise RuntimeError(state.get('reason'))
        raise RuntimeError('no confirmed visual localization within timeout; inspect visual diagnostics')
    finally:
        n.destroy_node(); rclpy.shutdown()
if __name__=='__main__':
    raise SystemExit(main())
