#!/usr/bin/env python

import rospy
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import LaserScan
from drive_msgs.msg import drive_param
from drive_msgs.msg import gazebo_state_telemetry

from rviz_geometry import show_circle_in_rviz, show_line_in_rviz, delete_marker

from circle import Circle, Point

import math

import numpy as np

import time

from dynamic_reconfigure.server import Server
from wallfollowing3.cfg import wallfollowing3Config

TOPIC_DRIVE_PARAMETERS = "/input/drive_param/autonomous"
TOPIC_GAZEBO_STATE_TELEMETRY = "/gazebo/state_telemetry"
TOPIC_CONTROLLED_DRIVE_PARAM = "/commands/controlled_drive_param"
TOPIC_LASER_SCAN = "/scan"

CAR_ACCELERATION = 4.5
CAR_DECCELERATION = 4.5

FRICTION = 0.5

MAX_STEERING_ANGLE = 30 * (math.pi / 180)
CAR_LENGTH = 0.325

CURVE_TYPE_RIGHT = 0
CURVE_TYPE_LEFT = 1

last_angle_moving_average = [[], 8]
last_speed = 0
current_speed = 0

total_scan_time = 0
total_scan_count = 0


class Parameters():
    def __init__(self, default_values):
        self.names = default_values.keys()
        for name in self.names:
            setattr(self, name, default_values[name])

    def __str__(self):
        return '\n'.join(name + ": " + str(getattr(self, name))
                         for name in self.names)


class PIDController():
    def __init__(self, p, i, d, anti_windup=0.2):
        self.p = p
        self.i = i
        self.d = d
        self.anti_windup = anti_windup

        self.integral = 0
        self.previous_error = 0

    def update_and_get_correction(self, error, delta_time):
        self.integral += error * delta_time
        if self.integral > self.anti_windup:
            self.integral = self.anti_windup
        elif self.integral < -self.anti_windup:
            self.integral = -self.anti_windup

        derivative = (error - self.previous_error) / delta_time
        self.previous_error = error
        return self.p * error + self.i * self.integral + self.d * derivative


def map(in_lower, in_upper, out_lower, out_upper, value):
    result = out_lower + (out_upper - out_lower) * \
        (value - in_lower) / (in_upper - in_lower)
    return min(out_upper, max(out_lower, result))


def convertRpmToSpeed(rpm):
    return rpm / 1299.224


def drive(angle, velocity):
    message = drive_param()
    message.angle = angle
    message.velocity = velocity
    drive_parameters_publisher.publish(message)


def get_scans_in_angular_range(laser_scan, angle_start, angle_end):
    ranges = np.array(laser_scan.ranges)
    filterd_ranges = ranges[int((laser_scan.angle_max+angle_start)/laser_scan.angle_increment):int((laser_scan.angle_max+angle_end)/laser_scan.angle_increment)]
    inf_mask = np.isinf(filterd_ranges)
    if inf_mask.any():
        filterd_ranges = filterd_ranges[~inf_mask]
    return filterd_ranges


def get_scan_as_cartesian(laser_scan):
    ranges = np.array(laser_scan.ranges)

    angles = np.linspace(
        laser_scan.angle_min,
        laser_scan.angle_max,
        ranges.shape[0])

    laser_range = laser_scan.angle_max - laser_scan.angle_min
    usable_range = math.radians(parameters.usable_laser_range)
    if usable_range < laser_range:
        skip_left = int((-laser_scan.angle_min - usable_range / 2) / laser_range * ranges.shape[0])  # nopep8
        skip_right = int((laser_scan.angle_max - usable_range / 2) / laser_range * ranges.shape[0])  # nopep8
        angles = angles[skip_left:-1 - skip_right]
        ranges = ranges[skip_left:-1 - skip_right]

    inf_mask = np.isinf(ranges)
    if inf_mask.any():
        ranges = ranges[~inf_mask]
        angles = angles[~inf_mask]

    points = np.zeros((ranges.shape[0], 2))
    points[:, 0] = -np.sin(angles) * ranges
    points[:, 1] = np.cos(angles) * ranges

    return points


def find_left_right_border(points, margin_relative=0.1):
    margin = int(points.shape[0] * margin_relative)

    relative = points[margin + 1:-margin, :] - points[margin:-margin - 1, :]
    distances = np.linalg.norm(relative, axis=1)

    return margin + np.argmax(distances) + 1


def calc_max_speed(distance, current_speed, target_speed, acceleration, decceleration):
    return math.sqrt((2 * distance * acceleration * decceleration + current_speed**2 * decceleration + target_speed**2 * acceleration) / (acceleration + decceleration))


def calc_braking_distance(distance, current_speed, target_speed, acceleration, decceleration):
    return (2 * distance * acceleration + current_speed**2 - target_speed**2) / (2 * acceleration + 2 * decceleration)


def calc_max_curve_speed(friction, radius):
    return math.sqrt(friction * 9.81 * radius)


def calc_steering_radius(steering_angle):
    if steering_angle == 0:
        return 10000
    return CAR_LENGTH / math.sin(steering_angle * MAX_STEERING_ANGLE)


def calc_max_steering_angle():
    if current_speed == 0:
        return 1
    intermediate_result = (FRICTION * 9.81 * CAR_LENGTH) / (current_speed**2)
    if intermediate_result >= 1:
        return 1
    angle_rad = math.asin(intermediate_result)
    return map(0, MAX_STEERING_ANGLE, 0, 1, angle_rad)


def calc_steering_angle(steering_radius):
    angle_rad = math.asin(CAR_LENGTH / steering_radius)
    return map(0, MAX_STEERING_ANGLE, 0, 1, angle_rad)


# def find_closest_point_to_car(points):
#     return min([])


def calc_angle_moving_average():
    if len(last_angle_moving_average[0]) == 0:
        return 0
    return sum(last_angle_moving_average[0]) / len(last_angle_moving_average[0])


def add_angle_moving_average(angle):
    global last_angle_moving_average
    if len(last_angle_moving_average[0]) >= last_angle_moving_average[1]:
        last_angle_moving_average[0] = last_angle_moving_average[0][1:]
    last_angle_moving_average[0].append(angle)


def calc_predicted_car_position(is_in_curve):
    if is_in_curve:
        prediction_distance = min(0.1 + last_speed * 0.3, 1.5)
    else:
        prediction_distance = min(0.1 + last_speed * 0.35, 2)
    # steering_radius = calc_steering_radius(calc_angle_moving_average())
    # circle_section_angle = prediction_distance / steering_radius
    # print steering_radius, circle_section_angle, Point(-steering_radius * math.cos(circle_section_angle) + steering_radius, abs(steering_radius * math.sin(circle_section_angle))), prediction_distance
    # return Point(-steering_radius * math.cos(circle_section_angle) + steering_radius, abs(steering_radius * math.sin(circle_section_angle))), prediction_distance
    return Point(0, prediction_distance), prediction_distance


def calc_target_car_position(predicted_car_position, curve_type, left_circle, right_circle, upper_circle, remaining_distance):
    left_point = left_circle.get_closest_point(predicted_car_position)
    right_point = right_circle.get_closest_point(predicted_car_position)
    central_point = Point((left_point.x + right_point.x) / 2, (left_point.y + right_point.y) / 2)
    track_width = abs(left_point.x - right_point.x)

    target_position = central_point

    if curve_type == CURVE_TYPE_LEFT:
        target_position = central_point
        if predicted_car_position.y > remaining_distance and remaining_distance is not None and upper_circle is not None:
            upper_point = upper_circle.get_closest_point(predicted_car_position)
            target_position = Point(
                (upper_point.x - track_width / 2),
                (left_point.y + right_point.y) / 2)
    elif curve_type == CURVE_TYPE_RIGHT:
        target_position = central_point
        if predicted_car_position.y > remaining_distance and remaining_distance is not None and upper_circle is not None:
            upper_point = upper_circle.get_closest_point(predicted_car_position)
            target_position = Point(
                (upper_point.x + track_width / 2),
                (left_point.y + right_point.y) / 2)

    show_line_in_rviz(2, [left_point, right_point],
                      color=ColorRGBA(1, 1, 1, 0.3), line_width=0.005)
    return target_position


def follow_walls(left_circle, right_circle, upper_circle, points, curve_type, remaining_distance, barrier, delta_time):
    global last_speed
    predicted_car_position, prediction_distance = calc_predicted_car_position(upper_circle is None)
    target_position = calc_target_car_position(predicted_car_position, curve_type, left_circle, right_circle, upper_circle, remaining_distance)

    distance_to_target = math.sqrt(target_position.x**2 + target_position.y**2)

    error = (target_position.x - predicted_car_position.x) / \
        distance_to_target
    if math.isnan(error) or math.isinf(error):
        error = 0

    steering_angle = pid.update_and_get_correction(
        error, delta_time)

    radius = min(left_circle.radius, right_circle.radius)

    speed = calc_max_curve_speed(FRICTION, radius)
    if remaining_distance is not None and upper_circle is not None:
        safety_margin = 0.25
        if remaining_distance < 5:
            safety_margin = 0.05 * remaining_distance
        target_speed = calc_max_curve_speed(FRICTION, upper_circle.radius)
        last_speed = target_speed
        braking_distance = calc_braking_distance(remaining_distance, current_speed, target_speed, CAR_ACCELERATION, CAR_DECCELERATION) + safety_margin
        if remaining_distance > braking_distance:
            speed = min(calc_max_speed(remaining_distance, current_speed, target_speed, CAR_ACCELERATION, CAR_DECCELERATION), speed)
        else:
            speed = target_speed

    steering_angle = steering_angle * map(parameters.high_speed_steering_limit_dead_zone, 1, 1, parameters.high_speed_steering_limit, speed/25)  # nopep8
    max_steering_angle = calc_max_steering_angle() # + (speed / 25) * 2
    # if steering_angle < 0:
    #     steering_angle = max(-max_steering_angle, steering_angle)
    # else:
    #     steering_angle = min(steering_angle, max_steering_angle)
    add_angle_moving_average(steering_angle)
    emergency_slowdown = min(1, distance_to_target**2 / prediction_distance**2)
    if emergency_slowdown > 0.6:
        emergency_slowdown = 1
    speed *= emergency_slowdown
    drive(steering_angle, speed)

    show_line_in_rviz(3, [Point(0, 0), predicted_car_position],
                      color=ColorRGBA(1, 1, 1, 0.3), line_width=0.005)
    show_line_in_rviz(4, [predicted_car_position,
                          target_position], color=ColorRGBA(1, 0.4, 0, 1))

    show_line_in_rviz(
        5, [Point(-2, barrier), Point(2, barrier)], color=ColorRGBA(1, 1, 0, 1))


def handle_scan(laser_scan, delta_time):
    if parameters is None:
        return

    points = get_scan_as_cartesian(laser_scan)

    if points.shape[0] == 0:
        rospy.logwarn("Skipping current laser scan message since it contains no finite values.")  # nopep8
        return

    split = find_left_right_border(points)

    right_wall = points[:split:4, :]
    left_wall = points[split::4, :]

    min_x_left_wall = min([point[0] for point in left_wall])
    right_wall = np.array([point for point in right_wall if point[0] > min_x_left_wall])

    max_x_right_wall = max([point[0] for point in right_wall])
    left_wall = np.array([point for point in left_wall if point[0] < max_x_right_wall])

    left_circle = Circle.fit(left_wall)
    right_circle = Circle.fit(right_wall)

    ranges_in_front = get_scans_in_angular_range(laser_scan, -0.01, 0.01)
    if len(ranges_in_front) != 0:
        min_range_in_front = min(ranges_in_front)
    else:
        min_range_in_front = 30

    upper_circle = None
    max_y = None
    curve_type = None
    if min_range_in_front < 30:
        radius_proportions = left_circle.radius/right_circle.radius
        if radius_proportions > 1.3 and right_circle.center.x < 0:
            max_y = np.max(left_wall[:, 1])
            upper_wall = np.array([point for point in right_wall if point[1] >= max_y - 1.5])
            right_wall = np.array([point for point in right_wall if point[1] < max_y])
            right_circle = Circle.fit(right_wall)
            curve_type = CURVE_TYPE_LEFT
        elif radius_proportions < 0.78 and right_circle.center.x > 0:
            max_y = np.max(right_wall[:, 1])
            upper_wall = np.array([point for point in left_wall if point[1] >= max_y - 1.5])
            left_wall = np.array([point for point in left_wall if point[1] < max_y])
            left_circle = Circle.fit(left_wall)
            curve_type = CURVE_TYPE_RIGHT

    if max_y is not None and len(upper_wall) > 0:
        upper_circle = Circle.fit(upper_wall)
        show_line_in_rviz(
            6, [Point(-2, max_y), Point(2, max_y)], color=ColorRGBA(0.2, 0.5, 0.8, 1))
        show_circle_in_rviz(upper_circle, upper_wall, 7)
    else:
        delete_marker(6)
        delete_marker(7)


    barrier_start = int(points.shape[0] * (0.5 - parameters.barrier_size_realtive))  # nopep8
    barrier_end = int(points.shape[0] * (0.5 + parameters.barrier_size_realtive))  # nopep8
    # Why max and not min?
    barrier = np.max(points[barrier_start: barrier_end, 1])

    follow_walls(left_circle, right_circle, upper_circle, points, curve_type, max_y, barrier, delta_time)

    show_circle_in_rviz(left_circle, left_wall, 0)
    show_circle_in_rviz(right_circle, right_wall, 1)


last_scan = None


def laser_callback(scan_message):
    global last_scan, total_scan_count, total_scan_time
    t_start = time.time()
    scan_time = scan_message.header.stamp.to_sec()
    if last_scan is not None and abs(scan_time - last_scan) > 0.0001 and scan_time > last_scan:  # nopep8
        delta_time = scan_time - last_scan
        handle_scan(scan_message, delta_time)

    last_scan = scan_time
    # t_delta = time.time() - t_start
    # total_scan_time += t_delta
    # total_scan_count += 1
    # if total_scan_count % 40 == 0:
    #     print "mean_scan_time:", total_scan_time / float(total_scan_count) * 1000.0, "ms", "scan_time:", delta_time, "s"


def speed_callback(speed_message):
    global current_speed
    current_speed = speed_message.wheel_speed


def controlled_drive_param_callback(drive_param):
    global current_speed
    print "speed:", current_speed
    current_speed = drive_param.velocity


def dynamic_configuration_callback(config, level):
    global parameters
    parameters = Parameters(config)
    pid.p = parameters.controller_p
    pid.i = parameters.controller_i
    pid.d = parameters.controller_d
    return config


rospy.init_node('wallfollowing3', anonymous=True)
parameters = None
pid = PIDController(1, 1, 1)

rospy.Subscriber(TOPIC_LASER_SCAN, LaserScan, laser_callback)
rospy.Subscriber(TOPIC_GAZEBO_STATE_TELEMETRY, gazebo_state_telemetry, speed_callback)
rospy.Subscriber(TOPIC_CONTROLLED_DRIVE_PARAM, drive_param, controlled_drive_param_callback)
drive_parameters_publisher = rospy.Publisher(
    TOPIC_DRIVE_PARAMETERS, drive_param, queue_size=1)

Server(wallfollowing3Config, dynamic_configuration_callback)

while not rospy.is_shutdown():
    rospy.spin()
