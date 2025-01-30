import numpy as np
import rclpy
from rclpy.node import Node
from acoustic_msgs.msg import DetectionFlag, Dvl, PingInfo, SonarImageData, ProjectedSonarImage, RawSonarImage, SonarDetections, SonarRanges
from environmental_msgs.msg import TurbidityNTU
from geometry_msgs.msg import Vector3

class BagCreator(Node):
    def __init__(self):
        super().__init__('bag_creator')

    def make_turbidity_ntu(self):
        msg = TurbidityNTU()
        msg.header.frame_id = "sensor"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.ntu = 13.7
        return msg

    def make_detection_flag(self):
        msg = DetectionFlag()
        msg.flag = DetectionFlag.DETECT_BAD_SONAR
        return msg

    def make_dvl(self):
        msg = Dvl()
        msg.header.frame_id = "sensor"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.velocity_mode = Dvl.DVL_MODE_BOTTOM
        msg.dvl_type = Dvl.DVL_TYPE_PISTON

        msg.velocity = Vector3(x=1.0, y=2.0, z=3.0)
        msg.velocity_covar = [-1.0] * 9
        
        msg.altitude = 2.6
        msg.course_gnd = 1.1
        msg.speed_gnd = 2.2

        msg.num_good_beams = 4
        msg.sound_speed = 1510

        msg.beam_ranges_valid = True
        msg.beam_velocities_valid = True
        msg.beam_unit_vec = [Vector3(x=4, y=5, z=6) for _ in range(4)]
        msg.range = [3.8, 3.9, 4.1, 4.2]
        msg.range_covar = [-1.0] * 4
        msg.beam_quality = [0.5] * 4
        msg.beam_velocity = [2.5] * 4
        msg.beam_velocity_covar = [-1.0] * 4
        return msg

    def make_ping_info(self, num_beams):
        msg = PingInfo()
        msg.frequency = 2100000
        msg.sound_speed = 1510
        msg.tx_beamwidths = [0.0044] * num_beams
        msg.rx_beamwidths = [0.21] * num_beams
        return msg

    def make_sonar_image_data(self, num_beams, num_ranges):
        msg = SonarImageData()
        msg.is_bigendian = False
        msg.dtype = SonarImageData.DTYPE_UINT8
        msg.beam_count = num_beams
        msg.data = [np.random.randint(256) for _ in range(num_beams * num_ranges)]
        return msg

    def make_projected_sonar_image(self):
        msg = ProjectedSonarImage()
        msg.header.frame_id = "sonar"
        msg.header.stamp = self.get_clock().now().to_msg()
        nbeams = 10
        msg.ping_info = self.make_ping_info(nbeams)

        msg.beam_directions = [Vector3(x=4, y=5, z=6)] * nbeams
        nranges = 10
        msg.ranges = [2.5 * i for i in range(nranges)]
        msg.image = self.make_sonar_image_data(nbeams, nranges)
        return msg

    def make_raw_sonar_image(self):
        msg = RawSonarImage()
        msg.header.frame_id = "sonar"
        msg.header.stamp = self.get_clock().now().to_msg()
        nbeams = 10
        msg.ping_info = self.make_ping_info(nbeams)

        nranges = 10
        msg.sample_rate = 6000
        msg.samples_per_beam = nranges
        msg.sample0 = 3

        msg.tx_delays = [0.001 * i for i in range(nbeams)]
        msg.tx_angles = [0.05 * i - 0.25 for i in range(nbeams)]
        msg.rx_angles = [0.01 * i - 0.05 for i in range(nbeams)]
        msg.image = self.make_sonar_image_data(nbeams, nranges)
        return msg

    def make_sonar_detections(self):
        msg = SonarDetections()
        msg.header.frame_id = "sonar"
        msg.header.stamp = self.get_clock().now().to_msg()
        nbeams = 10
        msg.ping_info = self.make_ping_info(nbeams)
        msg.flags = [self.make_detection_flag() for _ in range(nbeams)]
        msg.two_way_travel_times = [np.random.rand() for _ in range(nbeams)]
        msg.tx_delays = [0.001 * i for i in range(nbeams)]
        msg.intensities = [np.random.rand() for _ in range(nbeams)]
        msg.tx_angles = [0.05 * i - 0.25 for i in range(nbeams)]
        msg.rx_angles = [0.01 * i - 0.05 for i in range(nbeams)]
        return msg

    def make_sonar_ranges(self):
        msg = SonarRanges()
        msg.header.frame_id = "sonar"
        msg.header.stamp = self.get_clock().now().to_msg()
        nbeams = 10
        msg.ping_info = self.make_ping_info(nbeams)
        msg.flags = [self.make_detection_flag() for _ in range(nbeams)]
        msg.transmit_delays = [0.001 * i for i in range(nbeams)]
        msg.intensities = [np.random.rand() for _ in range(nbeams)]
        msg.beam_unit_vec = [Vector3(x=1, y=2, z=3) for _ in range(nbeams)]
        msg.ranges = [10 * np.random.rand() for _ in range(nbeams)]
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = BagCreator()

    # Replace ROS 1 rosbag logic with appropriate ROS 2 alternatives if needed.
    node.get_logger().info("Bag creation node initialized.")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
