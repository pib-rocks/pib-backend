import rclpy
import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class PibDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.__devices = {}

        self.__devices['tilt_sideways_motor'] = [self.__robot.getDevice('head_horizontal'),]
        self.__devices['tilt_forward_motor'] = [self.__robot.getDevice('head_vertical'),]
        self.__devices['shoulder_horizontal_right'] = [self.__robot.getDevice('shoulder_horizontal_right'),]
        self.__devices['elbow_right'] = [self.__robot.getDevice('ellbow_right'),]
        self.__devices['elbow_left'] = [self.__robot.getDevice('ellbow_left'),]

        self.__devices['lower_arm_right_rotation'] = [self.__robot.getDevice('forearm_right'),]
        self.__devices['lower_arm_left_rotation'] = [self.__robot.getDevice('forearm_left'),]
        self.__devices['thumb_left_stretch'] = [self.__robot.getDevice('thumb_left_distal'), 
                                                self.__robot.getDevice('thumb_left_proximal'),]
        self.__devices['thumb_right_stretch'] = [self.__robot.getDevice('thumb_right_distal'),
                                                self.__robot.getDevice('thumb_right_proximal'),]
        self.__devices['index_right_stretch'] = [self.__robot.getDevice('index_right_distal'),
                                                self.__robot.getDevice('index_right_proximal'),]
        self.__devices['middle_right_stretch'] = [self.__robot.getDevice('middle_right_distal'),
                                                self.__robot.getDevice('middle_right_proximal'),]
        self.__devices['ring_right_stretch'] = [self.__robot.getDevice('ring_right_distal'),
                                                self.__robot.getDevice('ring_right_proximal'),]
        self.__devices['pinky_right_stretch'] = [self.__robot.getDevice('pinky_right_distal'),
                                                self.__robot.getDevice('pinky_right_proximal'),]
        self.__devices['index_left_stretch'] = [self.__robot.getDevice('index_left_distal'),
                                                self.__robot.getDevice('index_left_proximal'),]
        self.__devices['middle_left_stretch'] = [self.__robot.getDevice('middle_left_distal'),
                                                self.__robot.getDevice('middle_left_proximal'),]
        self.__devices['ring_left_stretch'] = [self.__robot.getDevice('ring_left_distal'),
                                                self.__robot.getDevice('ring_left_proximal'),]
        self.__devices['pinky_left_stretch'] = [self.__robot.getDevice('pinky_left_distal'),
                                                self.__robot.getDevice('pinky_left_proximal'),]

        self.__devices['all_fingers_right'] = []
        self.__devices['all_fingers_right'].extend(self.__devices['thumb_right_stretch'])
        self.__devices['all_fingers_right'].extend(self.__devices['index_right_stretch'])
        self.__devices['all_fingers_right'].extend(self.__devices['middle_right_stretch'])
        self.__devices['all_fingers_right'].extend(self.__devices['ring_right_stretch'])
        self.__devices['all_fingers_right'].extend(self.__devices['pinky_right_stretch'])

        self.__devices['all_fingers_left'] = []
        self.__devices['all_fingers_left'].extend(self.__devices['thumb_left_stretch'])
        self.__devices['all_fingers_left'].extend(self.__devices['index_left_stretch'])
        self.__devices['all_fingers_left'].extend(self.__devices['middle_left_stretch'])
        self.__devices['all_fingers_left'].extend(self.__devices['ring_left_stretch'])
        self.__devices['all_fingers_left'].extend(self.__devices['pinky_left_stretch'])
        
        self.__target_trajectory = JointTrajectory()

        rclpy.init(args=None)
        self.__node = rclpy.create_node('pib_driver')
        self.__node.get_logger().info("nach create node")
        self.__node.create_subscription(JointTrajectory, '/joint_trajectory', self.__trajectory_callback, 1)
        self.__node.get_logger().info("nach create subscribe")


    def __trajectory_callback(self, trajectory):
        self.__target_trajectory = trajectory
        self.__node.get_logger().info("trajectory callback " + trajectory.joint_names[0] + '  ' + str(trajectory.points[0].positions[0]))

    def convert_cerebra(self, position):
        return (math.radians(position/100.0))

    def create_device(self, name):
        rotIndex = name.find('rota')
        if rotIndex != -1 :
            device = name[:(rotIndex-1)]
        else:
            device = name
        self.__devices[name] = [self.__robot.getDevice(device),]
        self.__node.get_logger().info('created device: ' + device + '   name: ' + name)


    # position interface, only the first point in a trajectory is considered
    def set_target_positions(self):
        for index in range(len(self.__target_trajectory.joint_names)):
            name = self.__target_trajectory.joint_names[index]
            if not (name in self.__devices):
                self.create_device(name)

            position = self.convert_cerebra(self.__target_trajectory.points[0].positions[index])

            if self.__devices[name] is not None:
                for dev in self.__devices[name]:
                    dev.setPosition(position)


    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        self.set_target_positions()
