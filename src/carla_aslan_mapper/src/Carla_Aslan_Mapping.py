#!/usr/bin/env python
import numpy as np
import rospy
from carla_aslan_mapper.msg import SDControl
from carla_msgs.msg import CarlaEgoVehicleControl
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32
from geometry_msgs.msg import TwistStamped

class CarlaAslanMapping:

    def __init__(self):
        # Constants
        self.steer_min_aslan = -100
        self.steer_max_aslan = 100
        self.throttle_min_aslan = -100
        self.throttle_max_aslan = 30

        self.steer_min_carla = -1.0
        self.steer_max_carla = 1.0
        self.throttle_min_carla = -1.0
        self.throttle_max_carla = 1.0

        # Subscribers
        rospy.Subscriber("/carla/ego_vehicle/lidar", PointCloud2, self.lidar_callback)
        rospy.Subscriber("/carla/ego_vehicle/speedometer", Float32, self.speedometer_callback)
        rospy.Subscriber("/sd_control", SDControl, self.aslan_callback)

        # Publishers
        self.lidar_pub = rospy.Publisher('/points_raw', PointCloud2, queue_size=1)
        self.velocity_pub = rospy.Publisher('/carla_twist', TwistStamped, queue_size=1)
        self.carla_pub = rospy.Publisher('/carla/ego_vehicle/vehicle_control_cmd', CarlaEgoVehicleControl, queue_size=1)

    def _normalise(self, value, in_min, in_max, out_min, out_max) :
        if value >= 0:
            return value / in_max * out_max
        else:
            return value / in_min * out_min

    def normalise_steering_value(self, value):
        return self._normalise(
            value,
            self.steer_min_aslan,
            self.steer_max_aslan,
            self.steer_min_carla,
            self.steer_max_carla
        )

    def normalise_throttle_value(self, value):
        return self._normalise(
            value,
            self.throttle_min_aslan,
            self.throttle_max_aslan,
            self.throttle_min_carla,
            self.throttle_max_carla
        )

    def aslan_callback(self, aslan_data):
        carla_msg = CarlaEgoVehicleControl()

        # Steering directions in Aslan and Carla are opposite
        carla_msg.steer = -self.normalise_steering_value(aslan_data.steer)
        if aslan_data.torque >= 0:
            carla_msg.throttle = self.normalise_throttle_value(aslan_data.torque)
            carla_msg.brake = 0
        else:
            carla_msg.throttle = 0
            carla_msg.brake = self.normalise_throttle_value(-aslan_data.torque)

        self.carla_pub.publish(carla_msg)

    def speedometer_callback(self, speed):
        carla_twist = TwistStamped()
        carla_twist.header.frame_id = "base_link"
        carla_twist.header.stamp = rospy.Time.now()
        carla_twist.twist.linear.x = speed.data
        self.velocity_pub.publish(carla_twist)

    def lidar_callback(self, lidar_data):
        enriched_lidar_data = PointCloud2()
        enriched_lidar_data.header = lidar_data.header
        enriched_lidar_data.header.frame_id = "velodyne"
        enriched_lidar_data.height = lidar_data.height
        enriched_lidar_data.width = lidar_data.width
        enriched_lidar_data.is_bigendian = lidar_data.is_bigendian
        enriched_lidar_data.point_step = 22
        enriched_lidar_data.fields = lidar_data.fields
        enriched_lidar_data.fields.append(
            PointField(name='ring', offset=16, datatype=PointField.UINT16, count=1))
        enriched_lidar_data.fields.append(
            PointField(name='time', offset=18, datatype=PointField.FLOAT32, count=1))

        raw_data = np.frombuffer(lidar_data.data, dtype=np.float32).reshape((-1, 4))
        plane_dist = np.sqrt(np.square(raw_data[:,0]) + np.square(raw_data[:,1]))
        angle = np.arctan2(raw_data[:,2], plane_dist)

        min_angle = np.min(angle)
        max_angle = np.max(angle)
        angle_step = (max_angle - min_angle) / 31

        angle_idx = np.round((angle - min_angle) / angle_step).astype(np.uint16)

        enriched_data = np.rec.fromarrays(
            [raw_data[:,i] for i in range(raw_data.shape[1])]
            + [angle_idx, np.zeros(raw_data.shape[0], dtype=np.float32)]
        )
        enriched_lidar_data.data = enriched_data.tobytes()
        enriched_lidar_data.row_step = 22 * enriched_lidar_data.width
        enriched_lidar_data.is_dense = False

        self.lidar_pub.publish(enriched_lidar_data)


if __name__ == '__main__':
    rospy.init_node('Aslan_Carla_Mapper', anonymous=True)
    try:
        mapper = CarlaAslanMapping()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
