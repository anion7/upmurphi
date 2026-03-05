#!/usr/bin/env python3
"""
ROS2 Plan Dispatcher Node

Publishes UPMurphi plan actions as ROS2 messages for simulation.
Works with ROSPlan-style action dispatch or custom topics.

Dependencies: rclpy, std_msgs, geometry_msgs (install via ros2)

Usage:
  ros2 run upmurphi_ros_bridge ros2_plan_dispatcher \\
      --ros-args -p plan_file:=plan.pddl -p handler:=turtlebot
"""

import os
import sys
import json
import time as time_mod

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from upmurphi_plan_parser import parse_plan_file, PlanAction

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False


if HAS_ROS2:

    class UPMurphiDispatcherNode(Node):
        """ROS2 node that dispatches UPMurphi plans."""

        def __init__(self):
            super().__init__('upmurphi_plan_dispatcher')

            self.declare_parameter('plan_file', '')
            self.declare_parameter('time_scale', 0.01)
            self.declare_parameter('auto_dispatch', True)

            self.action_pub = self.create_publisher(
                String, '/upmurphi/action_dispatch', 10)
            self.status_pub = self.create_publisher(
                String, '/upmurphi/dispatch_status', 10)
            self.plan_pub = self.create_publisher(
                String, '/upmurphi/plan', 10)

            plan_file = self.get_parameter('plan_file').value
            if plan_file:
                self.get_logger().info(f'Loading plan from: {plan_file}')
                self.plans = parse_plan_file(plan_file)
                if self.plans:
                    self.publish_plan_summary()
                    auto = self.get_parameter('auto_dispatch').value
                    if auto:
                        self.create_timer(1.0, self.dispatch_plan_callback)
                else:
                    self.get_logger().error('No plans found in file')
            else:
                self.plans = []
                self.get_logger().warn('No plan_file specified. '
                                       'Waiting for plan on /upmurphi/plan_input...')

            self.dispatched = False

        def publish_plan_summary(self):
            if not self.plans:
                return
            plan = self.plans[0]
            summary = {
                'plan_number': plan.plan_number,
                'num_actions': plan.num_actions,
                'duration': plan.total_duration,
                'actions': [
                    {'t': a.timestamp, 'name': a.action_name,
                     'params': a.parameters, 'dur': a.duration}
                    for a in plan.actions
                ]
            }
            msg = String()
            msg.data = json.dumps(summary, ensure_ascii=False)
            self.plan_pub.publish(msg)
            self.get_logger().info(
                f'Plan #{plan.plan_number}: {plan.num_actions} actions, '
                f'duration={plan.total_duration:.3f}s')

        def dispatch_plan_callback(self):
            if self.dispatched or not self.plans:
                return
            self.dispatched = True
            self.dispatch_plan(self.plans[0])

        def dispatch_plan(self, plan):
            time_scale = self.get_parameter('time_scale').value
            self.get_logger().info(f'Dispatching plan #{plan.plan_number}...')

            status_msg = String()
            status_msg.data = json.dumps({
                'status': 'DISPATCHING', 'plan': plan.plan_number})
            self.status_pub.publish(status_msg)

            for i, action in enumerate(plan.actions):
                action_msg = String()
                action_msg.data = json.dumps({
                    'action_id': i,
                    'action_name': action.action_name,
                    'parameters': action.parameters,
                    'timestamp': action.timestamp,
                    'duration': action.duration,
                    'status': 'DISPATCHING',
                })
                self.action_pub.publish(action_msg)

                self.get_logger().info(
                    f'[{action.timestamp:.3f}] ({action.action_name} '
                    f'{" ".join(action.parameters)}) [{action.duration:.3f}]')

                sim_dur = action.duration * time_scale
                if sim_dur > 0:
                    time_mod.sleep(sim_dur)

                action_msg.data = json.dumps({
                    'action_id': i,
                    'action_name': action.action_name,
                    'status': 'SUCCEEDED',
                })
                self.action_pub.publish(action_msg)

            status_msg.data = json.dumps({
                'status': 'COMPLETED', 'plan': plan.plan_number})
            self.status_pub.publish(status_msg)
            self.get_logger().info('Plan dispatch complete!')

    def main(args=None):
        rclpy.init(args=args)
        node = UPMurphiDispatcherNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()

else:
    def main(args=None):
        print("ERROR: ROS2 (rclpy) is not available.")
        print("Install ROS2 or use the standalone dispatcher:")
        print("  python3 plan_dispatcher.py <plan_file>")
        sys.exit(1)


if __name__ == '__main__':
    main()
