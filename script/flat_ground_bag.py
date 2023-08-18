import numpy
import rclpy
from rclpy.node import Node
import signal
import threading
import pinocchio as pin
import time

from crocoddyl_ros import WholeBodyStateRosSubscriber
from crocoddyl_ros import WholeBodyStateRosPublisher

class WholeBodyStateRepublisherNode(Node):

    def __init__(self):
        super().__init__('whole_body_state_republisher')
        model = pin.buildModelFromUrdf("/home/james/ihmc-msgs-workspace/src/ihmc_msgs/resources/nadia-description/urdf/nadia_combined_knee_simple_collisions.urdf", pin.JointModelFreeFlyer()) # TODO MPC urdf
        self.sub = WholeBodyStateRosSubscriber(model)
        self.pub = WholeBodyStateRosPublisher(model)

    def run(self):
        if self.sub.has_new_msg():
            t, q, v, tau, p, pd, f, s = self.sub.get_state()
            p_mod, pd_mod, f_mod = self.numpy_to_pinocchio(p, pd, f)
            self.pub.publish(t, q, v, tau, p_mod, pd_mod, f_mod, s)

    def numpy_to_pinocchio(self, p, pd, f):
        # WholeBodyStateRosPublisher has a Pinocchio API, WholeBodyStateRosSubscriber has an Eigen API, so we need to
        # convert from Eigen to Pinocchio
        p_mod = {}
        pd_mod = {}
        f_mod = {}
        contact_names = p.keys()
        for name in contact_names:
            p_value = p[name]
            pd_value = pd[name]
            f_value = f[name]
            p_mod[name] = pin.SE3(p_value[1], p_value[0])  # first rotation, then translation
            pd_mod[name] = pin.Motion(pd_value)
            f_mod[name] = [pin.Force(f_value[0]), f_value[1], f_value[2]]
        return p_mod, pd_mod, f_mod


if __name__ == "__main__":
    rclpy.init()
    node = WholeBodyStateRepublisherNode()

    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    rate = node.create_rate(100)

    try:
        while rclpy.ok():
            node.run()
            rate.sleep()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    thread.join()
