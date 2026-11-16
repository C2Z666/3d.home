#!/usr/bin/env python3
"""Project factory scan into the planar Nav2 base frame. Verify scan-source semantics first."""
import math
import numpy as np
from scipy.spatial.transform import Rotation

def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.duration import Duration
    from rclpy.time import Time
    from sensor_msgs.msg import LaserScan
    from tf2_ros import Buffer,TransformListener
    rclpy.init(); node=Node('d1_scan_adapter'); buffer=Buffer(); listener=TransformListener(buffer,node)
    pub=node.create_publisher(LaserScan,'/d1/nav/scan',qos_profile_sensor_data)
    def receive(m):
        try:
            t=buffer.lookup_transform('base_link',m.header.frame_id,Time.from_msg(m.header.stamp),timeout=Duration(seconds=.02)).transform
            q=t.rotation; R=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix(); offset=np.array([t.translation.x,t.translation.y,t.translation.z])
            if abs(R[2,2])<.98: return  # Reject significantly tilted scan planes.
            out=LaserScan(); out.header=m.header; out.header.frame_id='nav_base'
            count=720; out.angle_min=-math.pi; out.angle_max=math.pi-2*math.pi/count; out.angle_increment=2*math.pi/count
            out.range_min=.05; out.range_max=m.range_max+float(np.linalg.norm(offset[:2])); out.scan_time=m.scan_time
            ranges=np.full(count,np.inf)
            for i,r in enumerate(m.ranges):
                if not math.isfinite(r) or not m.range_min<=r<=m.range_max: continue
                a=m.angle_min+i*m.angle_increment; xyz=R@np.array([r*math.cos(a),r*math.sin(a),0.])+offset
                distance=math.hypot(xyz[0],xyz[1]); angle=math.atan2(xyz[1],xyz[0])
                index=min(count-1,max(0,int((angle+math.pi)/out.angle_increment)))
                ranges[index]=min(ranges[index],distance)
            out.ranges=ranges.astype(float).tolist(); pub.publish(out)
        except Exception as e:
            node.get_logger().warning('scan transform unavailable: '+str(e),throttle_duration_sec=3.)
    node.create_subscription(LaserScan,'/laser_scan',receive,qos_profile_sensor_data)
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
if __name__=='__main__': main()
