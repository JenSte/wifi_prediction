#!/usr/bin/env python

import math

import rospy
import tf
import tf.transformations

import geometry_msgs.msg
import move_base_msgs.msg
import nav_msgs.msg
import sensor_msgs.msg

import shapely.geometry
import shapely.geometry.polygon


class RangeChecker(object):
    '''Listens to wifi range updates and allows to check if a point has wifi coverage.'''

    def __init__(self, reference_frame_id):
        self._reference_frame_id = reference_frame_id
        self._subscribers = []
        self._polygons = {}

        self._tf_listener = tf.TransformListener()

    def add_scan_topic(self, topic_name):
        callback = lambda msg: self._scan_callback(topic_name, msg)
        self._subscribers.append(rospy.Subscriber(topic_name, sensor_msgs.msg.LaserScan, callback))

    def _scan_callback(self, topic_name, msg):
        translated_points = self._transform_laser_scanner_message(msg)

        if translated_points == None:
            self._polygons.pop(topic_name, None)
            return

        self._polygons[topic_name] = shapely.geometry.polygon.Polygon(translated_points)

    def _transform_laser_scanner_message(self, msg):
        try:
            (trans, rot) = self._tf_listener.lookupTransform(self._reference_frame_id, msg.header.frame_id, rospy.Time(0))

            translated_points = []

            for i, r in enumerate(msg.ranges):
                angle = msg.angle_min + i * msg.angle_increment
                if angle > 2 * math.pi:
                    # In Stage, a maximum angle of 362 degree had to be specified
                    # for it to cover the whole circle, but we want a proper polygon
                    # without self-intersections.
                    break

                x = r * math.cos(angle)
                y = r * math.sin(angle)

                x += trans[0]
                y += trans[1]

                translated_points.append((x, y))

            return translated_points

        except Exception as e:
            print('unable to transform: {}'.format(e))
        
        return None

    def check_point(self, point):
        p = shapely.geometry.Point(point)

        for topic_name, polygon in self._polygons.iteritems():
            if polygon.contains(p):
                return True

        return False


class WifiPrediction(object):
    '''Listens for navigation plan updates and moves a repeater so that we have wifi coverage.'''

    FRAME_ID = 'map'
    FEEDBACK_TOPIC_NAME = '/move_base/feedback'
    PLAN_TOPIC_NAME = '/move_base/NavfnROS/plan'

    def __init__(self, fixed_repeaters, free_repeaters):
        self._free_repeaters = free_repeaters
        self._next_drop_point = None
        self._last_drop_time = -2.0

        self._range_checker = RangeChecker(WifiPrediction.FRAME_ID)

        for r in fixed_repeaters:
            self._range_checker.add_scan_topic(self._scan_topic_name(r))

        self._reposition_publishers = {}
        for r in self._free_repeaters:
            self._reposition_publishers[r] = rospy.Publisher(self._reposition_topic_name(r), geometry_msgs.msg.Point, queue_size=2)

        self._feedback_subscriber = rospy.Subscriber(
            WifiPrediction.FEEDBACK_TOPIC_NAME,
            move_base_msgs.msg.MoveBaseActionFeedback,
            self._feedback_callback)
        self._plan_subscriber = rospy.Subscriber(
            WifiPrediction.PLAN_TOPIC_NAME,
            nav_msgs.msg.Path,
            self._plan_callback)

    def _scan_topic_name(self, repeater_name):
        return '/wifis/{}/range'.format(repeater_name)

    def _reposition_topic_name(self, repeater_name):
        return '/wifis/{}/reposition'.format(repeater_name)

    def _near(self, p1, p2, max_distance=0.3):
        distance = shapely.geometry.Point(p1).distance(shapely.geometry.Point(p2))
        return distance < max_distance

    def _feedback_callback(self, msg):
        if msg.feedback.base_position.header.frame_id != WifiPrediction.FRAME_ID:
            print('feedback not in "{}" frame'.format(WifiPrediction.FRAME_ID))
            return

        if self._next_drop_point == None:
            return

        # Check if we have reached the point where we want to drop the repeater.
        current_position = (msg.feedback.base_position.pose.position.x, msg.feedback.base_position.pose.position.y)
        if not self._near(self._next_drop_point, current_position):
            return
        
        # Take the next free repeater and drop it.
        if not self._free_repeaters:
            print('no more repeaters to drop')
            return

        repeater = self._free_repeaters.pop(0)
        self._next_drop_point = None

        # Calculate a position behind our robot where we will drop the repeater.
        o = msg.feedback.base_position.pose.orientation
	(roll, pitch, yaw) = tf.transformations.euler_from_quaternion([o.x, o.y, o.z, o.w])

        drop_distance = 0.5
        dx = drop_distance * math.cos(yaw)
        dy = drop_distance * math.sin(yaw)

        point_msg = geometry_msgs.msg.Point();
        point_msg.x = current_position[0] - dx
        point_msg.y = current_position[1] - dy
        point_msg.z = 0.0
        self._reposition_publishers[repeater].publish(point_msg)
        self._last_drop_time = rospy.get_time()

        self._range_checker.add_scan_topic(self._scan_topic_name(repeater))
 
    def _plan_callback(self, msg):
        if msg.header.frame_id != WifiPrediction.FRAME_ID:
            print('plan not in "{}" frame'.format(WifiPrediction.FRAME_ID))
            return

        if self._next_drop_point != None:
            return

        if rospy.get_time() - self._last_drop_time < 1.0:
            return
        
        path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        path = self._decimate_path(path)

        # Search for the first point that is not in the wifi range any more.
        idx = None
        for i, p in enumerate(path[:6]):
            if not self._range_checker.check_point(p):
                idx = i
                break

        if idx == None:
            # All points are in the range of the wifi, nothing to do
            return

        # Drop one step earlier, when we are still in the wifi range.
        if idx > 0:
            idx -= 1

        self._next_drop_point = path[idx]
#        print('next drop point: {}'.format(self._next_drop_point))
#        print(path)

    def _decimate_path(self, path):
        if not path:
            return path

        min_distance = 0.3

        # Start with the first point in the path, it is always taken.
        result = path[0:1]

        pairs = zip(path[:-1], path[1:])
        distances = [shapely.geometry.Point(ps[0]).distance(shapely.geometry.Point(ps[1])) for ps in pairs]

        sum_distances = 0.0 # Total distance to the previous taken point
        for point, distance in zip(path[1:], distances):
            sum_distances += distance
            if sum_distances > min_distance:
                result.append(point)
                sum_distances = 0.0

        return result


if __name__ == '__main__':
    try:
        rospy.init_node('wifi_prediction_node')

        if rospy.get_param('/use_sim_time', False):
            print('waiting for valid simulation time')
            zero_time = rospy.Time()
            while rospy.Time.now() != zero_time:
                pass

        rospy.sleep(0.5) 

        fixed = ['repeater0']
        free = ['repeater1', 'repeater2', 'repeater3', 'repeater4']
        wifi_prediction = WifiPrediction(fixed, free)

        rospy.spin()
    except rospy.ROSInterruptException: 
        pass
