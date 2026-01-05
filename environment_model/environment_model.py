# import required libraries
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, TransformException
import tf_transformations
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from sklearn.cluster import DBSCAN
import numpy as np

# import required standard and custom data types
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from vision_msgs.msg import Detection2DArray
from custom_msgs.msg import DetectedObject, DetectedObjectsPositionArray
from geometry_msgs.msg import TransformStamped

# measured from "base_link" to new "v2x_frame"
TRANSLATION_X = 0.65
TRANSLATION_Y = 0.0
TRANSLATION_Z = -0.07

# extracted values from ros2 topic of realsense camera
F_X= 607.3187866210938
F_Y = 607.1571044921875
C_X = 320.80078125
C_Y = 247.81671142578125

class EnvironmentModel(Node):

    def __init__(self, par='default'):
        super().__init__('environment_model')
        # create subscribers to /detectnet/detections, /odom and /camera/camera/aligned_depth_to_color/image_raw topics from Object Detection, Localization and Camera
        self.subscription = self.create_subscription(Detection2DArray, '/detectnet/detections', self.detections_callback, 10)
        self.subscription = self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        # self.subscription = self.create_subscription(Image, '/camera/camera/depth/image_rect_raw', self.depth_callback, 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # create publisher to publish detected object position
        self.publish_objects_position = self.create_publisher(DetectedObjectsPositionArray, 'detected_objects_pos', 10)
        
        # initialize the transform broadcaster
        self.br = TransformBroadcaster(self)
        
        # create timer of 0.02 sec to broadcast "v2x_frame"
        self.timer = self.create_timer(0.02, self.broadcast_frame)
        
        # initialize buffer and listen to the transforsm
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # intialize the variables 
        self.yaw = 0
        self.flg_calc_xyz = 0
        self.detection_time = 0
        self.bbox = []
        self.class_ids_confidence = []
        
        # Log startup info
        self.get_logger().info('✅ environment_model node started and ready!')

    def detections_callback(self, data):
        # store the detection time, centre coordinates and size of boundary box of the detected objects and their class ids and confidences
        if self.flg_calc_xyz == 0: # store only when the coordinates are not calculated in the depth_callback function
            self.detection_time = data.header.stamp.sec* 1e3 + data.header.stamp.nanosec * 1e-6
            for item in data.detections:
                self.bbox.append([item.bbox.center.position.x, item.bbox.center.position.y, item.bbox.size_x, item.bbox.size_y])
                self.class_ids_confidence.append([item.results[0].hypothesis.class_id, int(item.results[0].hypothesis.score*100)])
            if self.bbox == []:
                self.flg_calc_xyz = 0
            else:
                self.flg_calc_xyz = 1

    def depth_callback(self, data):
        
        # exit if there are no detections
        if self.bbox == []: 
            return
        
        # extract step and depth image from depth topic
        step = data.step
        depth_image = np.frombuffer(data.data, dtype=np.uint16).reshape((data.height, data.width))
        
        self.id = 0
        dop = DetectedObjectsPositionArray()
        
        
        for i in range(len(self.bbox)):
            # calculate the top left and right bottom coordinates of the boundary box for each object
            bbox_cx, bbox_cy, bbox_size_x, bbox_size_y= self.bbox[i][0], self.bbox[i][1], self.bbox[i][2], self.bbox[i][3]
            x_min = max(0, int(bbox_cx - bbox_size_x / 2))
            x_max = min(data.width, int(bbox_cx + bbox_size_x / 2))
            y_min = max(0, int(bbox_cy - bbox_size_y / 2))
            y_max = min(data.height, int(bbox_cy + bbox_size_y / 2))

            # extract the depth values in the bounding box window and find the valid depth values (0.1 - 10m)
            depth_window = depth_image[y_min:y_max, x_min:x_max] / 1000.0  #convert to meters
            self.valid_depths = depth_window[(depth_window > 0.1) & (depth_window < 10.0)] 
            
            # skip the detected object is there are no valid depth values
            if len(self.valid_depths) == 0:
                continue
            
            # cluster the depth values using DBSCAN algorithm (eps=0.1, min_samples=30)
            depth_vals = self.valid_depths.reshape(-1, 1)
            clustering = DBSCAN(eps=0.1, min_samples=30).fit(depth_vals)
            self.labels = clustering.labels_

            # choose the largest cluster and find the depth
            if len(set(self.labels)) > 1:
                largest_cluster = np.argmax(np.bincount(self.labels[self.labels >= 0]))
                cluster_depths = depth_vals[self.labels == largest_cluster]
                self.dominant_depth = np.median(cluster_depths)
            else:
                self.dominant_depth = np.median(self.valid_depths)
             
            # calculate the coordinates in camera depth optical frame using depth 
            self.X = self.dominant_depth * (self.bbox[i][0] - C_X) / F_X
            self.Y = self.dominant_depth * (self.bbox[i][1] - C_Y) / F_Y
            self.Z = self.dominant_depth

            #create point in camera depth optical frame
            point_in_source = PointStamped()
            point_in_source.header.stamp = self.get_clock().now().to_msg()
            point_in_source.header.frame_id = 'camera_depth_optical_frame'  # example source frame
            point_in_source.point.x = self.X
            point_in_source.point.y = self.Y
            point_in_source.point.z = self.Z

            try:
                # lookup the transform from 'camera_depth_optical_frame' to 'v2x_frame'
                transform = self.tf_buffer.lookup_transform(
                    'v2x_frame',   # target frame
                    'camera_depth_optical_frame',   # source frame
                    rclpy.time.Time())

                # transform the point to v2x_frame
                point_in_target = do_transform_point(point_in_source, transform)
                
                # store the position, class id and confidence of the detected object
                self.id = self.id + 1
                do = DetectedObject()
                do.id = self.id
                do.class_id,  do.confidence = self.class_ids_confidence[i][0], self.class_ids_confidence[i][1]
                do.x, do.y, do.z = int(point_in_target.point.x*100), int(point_in_target.point.y*100), int(point_in_target.point.z*100)
                dop.array.append(do)
                
                self.get_logger().info(
                    f"Distance: {self.dominant_depth}: "
                    f"Point in {point_in_source.header.frame_id}: "
                    f"({point_in_source.point.x:.2f}, {point_in_source.point.y:.2f}) -->  "
                    f"Point in {point_in_target.header.frame_id}: "
                    f"({point_in_target.point.x:.2f}, {point_in_target.point.y:.2f})")

            except TransformException as ex:
                self.get_logger().warn(f'Could not transform point: {ex}')
        
        # publish the positions, class id and confidence of detected objects
        dop.detection_time = self.detection_time
        self.publish_objects_position.publish(dop)
        
        # clear the array and variables
        dop.array.clear()
        self.bbox.clear()
        self.flg_calc_xyz = 0

    def broadcast_frame(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = '4/base_link'
        t.child_frame_id = 'v2x_frame'

        # translation: center of v2x_frame relative to base link
        t.transform.translation.x = TRANSLATION_X
        t.transform.translation.y = TRANSLATION_Y
        t.transform.translation.z = TRANSLATION_Z

        # rotation: yaw only (roll, pitch = 0)
        q = tf_transformations.quaternion_from_euler(0, 0, -self.yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # broadcast the new frame
        self.br.sendTransform(t)

    def odom_callback(self, data):
        q = data.pose.pose.orientation

        # convert quaternion to euler to get yaw angle
        _, _, self.yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])


def main(args=None):
    rclpy.init(args=args)
    node = EnvironmentModel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__': 
    main()
