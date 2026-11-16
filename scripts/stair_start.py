#!/usr/bin/env python3
import argparse,json,math,queue,sys,threading,time
from stair_policy import StairPolicy

def main():
    p=argparse.ArgumentParser(description='Supervised straight Stair segment')
    
    p.add_argument('--duration',type=float,required=True,help='maximum forward seconds, NOT landing detection')
    args=p.parse_args()

# 初始值仅用于进入准备流程；真正运动前由确认输入覆盖。
    policy=StairPolicy(0.05,args.duration,time.monotonic())
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Bool,String
    rclpy.init(); node=Node('d1_stair_start')
    status={}; received=-float('inf'); answers=queue.Queue()
    def receive(msg):
        nonlocal status,received
        try:
            data=json.loads(msg.data)
            if isinstance(data,dict): status=data; received=time.monotonic()
        except (ValueError,TypeError): pass
    node.create_subscription(String,'/d1/sdk/d1_motion_status',receive,1)
    permit=node.create_publisher(Bool,'/d1/sdk/stair_motion_permit',1)
    velocity=node.create_publisher(Twist,'/d1/sdk/stair_cmd_vel',1)
    action=node.create_publisher(String,'/d1/sdk/d1_action',1)
    zero=node.create_publisher(Twist,'/d1/sdk/supervised_cmd_vel',1)
    def stop():
        velocity.publish(Twist()); permit.publish(Bool(data=False)); zero.publish(Twist())
    def ask():
        try:
            answer=input(
               '已确认 Stair。输入 y 和上楼速度，例如 y 0.3（单位 m/s，上限 0.8）：'
            )
        except EOFError:
            answer=''
        answers.put(answer)
    print('等待 SDK 实际状态；请保持楼梯入口对正。确认前不会发非零速度。',flush=True)
    try:
        while rclpy.ok() and policy.state not in ('FAULT','DONE'):
            rclpy.spin_once(node,timeout_sec=.03); now=time.monotonic()
            command=policy.update(status,now,now-received)
            if command=='request_stair': stop(); action.publish(String(data='stair'))
            elif command=='prompt': stop(); threading.Thread(target=ask,daemon=True).start()
            elif command=='forward':
                permit.publish(Bool(data=True)); msg=Twist(); msg.linear.x=policy.speed; velocity.publish(msg)
            else: stop()
            if not answers.empty():
                answer=answers.get_nowait()

                # 只有处于等待确认状态时才处理输入。
                if policy.state=='WAIT_Y':
                    parts=answer.strip().split()

                    if len(parts)!=2 or parts[0].lower()!='y':
                        policy.fail('输入取消或格式错误，应为：y 0.3')
                        stop()
                    else:
                        try:
                            speed=float(parts[1])
                            cap=float(status.get('stair_speed_cap',0))

                            if not math.isfinite(cap) or cap<=0:
                                 raise ValueError('桥接速度上限无效')

                            limit=min(0.5,cap)

                            if not math.isfinite(speed) or not 0<speed<=limit:
                                raise ValueError(
                                    f'速度必须大于 0，且不超过 {limit:g} m/s'
                                )

                            policy.speed=speed
                            policy.answer('y',time.monotonic())
                            print(f'开始上楼，目标速度：{speed:g} m/s',flush=True)

                        except (ValueError,TypeError) as e:
                            policy.fail(str(e))
                            stop()
                print(policy.state+': '+policy.reason,flush=True)
    except KeyboardInterrupt: policy.fail('operator interrupted')
    finally:
        for _ in range(8):
            stop()
            if rclpy.ok(): rclpy.spin_once(node,timeout_sec=.03)
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
    return 0 if policy.state=='DONE' else 1
if __name__=='__main__': sys.exit(main())
