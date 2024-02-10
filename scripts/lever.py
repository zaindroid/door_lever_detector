import rospy
import torch
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import open3d as o3d
from std_msgs.msg import Header
import tf2_ros
import tf2_geometry_msgs
import sys
print(sys.path)
import sys
sys.path.append('/home/maira/catkin_ws/src/lever_detection/scripts')

# import coordetector
from coordetector import t2d2t3d


class LeverDetector:
    def __init__(self):
        rospy.init_node('z_lever_detector', anonymous=True)

        self.bridge = CvBridge()
        self.disp_imager = np.ones((480,640,3),dtype=np.uint8)
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.cloud_sub = rospy.Subscriber("/camera/depth_registered/points", PointCloud2, self.point_cloud_callback)
        self.pose_pub = rospy.Publisher("lever_pose", PoseStamped, queue_size=10)
        self.image_pub=rospy.Publisher("lever_image", Image, queue_size=10)
        

        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path='src/lever_detection/model/door.pt')
        self.cv_image = None
        self.cloud_data = None
        self.td23D = t2d2t3d()
       
        self.tfbuffer_ = tf2_ros.Buffer()

    def image_callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def point_cloud_callback(self, data):
        try:
            self.cloud_data = data
            #print(self.cv_image)
        except ValueError as e:
            print(e)

        

    def process_detection(self):
        if self.cv_image is None or self.cloud_data is None:
            rospy.logwarn("Image or point cloud data not yet available.")
            return

        # Process image with model
        # results = self.model(self.cv_image)
        # detections = results.xyxy[0]  # Assuming you want all detections. Filter as needed.


        results = self.model(self.cv_image, size=416)
        self.publish_image(results.render()[0])
        print(results.pandas().xyxy[0])
        detections = results.pandas().xyxy[0]
        detected = False

        # Iterate through detections and publish poses
        # bboxs = []
        # for _, row in detections.iterrows():
        #     if int(row['class']) == 2:  # Assuming '2' is the class for levers
        #         # Calculate 3D pose. This is a placeholder for your actual 3D pose calculation logic.
        #         pose_3d = self.calculate_3d_pose(row)
        #         if pose_3d:
        #             self.publish_pose(pose_3d)

        bboxs = []
        for index, row in detections.iterrows():
            try:
                if row['class'] == 2:
                    detected= True
                    box = [[int(row['xmin']),int(row['ymin'])],[int(row['xmax']),int(row['ymax'])]]
                    bboxs.append(box)
            except:
                rospy.logerr("failed!!!!!!!!!!!!!!!!!!!!!!!!!")
        #print(f"Row {index}: xmin={xmin}, ymin={ymin}, xmax={xmax}, ymax={ymax}")
        if not detected:
            return 'failed'
        print("Detection DONE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

        final_pose = None

        
        for i in range(10):
            try:
                cloud = self.cloud_data
                whole, obj_clus = self.td23D.get_box_voxel(box, cloud)
                viz = False
                if viz:
                    mean_coords = obj_clus.get_center()
                    mea = self.show_point(mean_coords,col=[0,1,0])
                    aabb = obj_clus.get_oriented_bounding_box()
                    o3d.visualization.draw_geometries([whole, obj_clus, mea, aabb])
                obj_pose = self.td23D.get_3D_cords(obj_clus)
                print("-----------open3d-------------------")
                print(obj_pose)
                print("-------------------------------")

                real_object_head = self.transform_3D2head([obj_pose])[0]
                print("-----------real_object_head-------------------")
                print(real_object_head)
                print("-------------------------------")
                #real_object_pose = self.mover.transform_head2map([real_object_head])[0]
                real_object_pose = self.transform_head2base([real_object_head])[0]
                print("-----------real_object_pose-------------------")
                print(real_object_pose)
                print("-------------------------------")
                # final_pose = real_object_pose ## use this one for realtime working
                final_pose = real_object_head ## this is for test only
                break
            except Exception as e:
                print(e)
                continue
        
        if final_pose==None:
            return 'failed'

        print(real_object_head)
    def show_point(self, point, col=[1,1,1]):
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector([point])
            pcd.colors = o3d.utility.Vector3dVector([np.array(col)])
            return pcd

    def calculate_3d_pose(self, detection_row):
        # Placeholder for conversion from 2D detection to 3D pose
        # You might use depth information from self.cloud_data and camera intrinsics
        # to calculate the 3D position of the detected lever.
        # Return a PoseStamped message or similar ROS message type with 3D coordinates.
        return None

    def publish_image(self,image):
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        image_message = self.bridge.cv2_to_imgmsg(image, encoding="passthrough")
        self.image_pub.publish(image_message)

    def publish_pose(self, pose_3d):
        pose_msg = PoseStamped()
        pose_msg.header = Header()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "BASE"  # Assuming the pose is in the robot's base frame
        pose_msg.pose.position = Point(pose_3d[0], pose_3d[1], pose_3d[2])
        pose_msg.pose.orientation = Quaternion(0, 0, 0, 1)  # No orientation information for now
        self.pose_pub.publish(pose_msg)

    def run(self):
        rospy.loginfo('[detect_lever] Trying to detect nearest lever')
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.process_detection()
            rate.sleep()
    def callback1(self,data):
        #print('callback')
        try:
            self.cloud_data = data
            #print(self.cv_image)
        except ValueError as e:
            print(e)

    def transform_3D2head(self,obj_poses):
        transformed = []
        for obj_pose in obj_poses:
            x,y,z = obj_pose.pose.position.x, obj_pose.pose.position.y, obj_pose.pose.position.z
            obj_pose.header.frame_id = "camera_link"
            obj_pose.pose.position.x = y #headx=3dy
            obj_pose.pose.position.y = z #heady=-3dz
            obj_pose.pose.position.z = x #headz=3dx
            transformed.append(obj_pose)
        return transformed
    # def transform_head2map(self,obj_poses):
    #     transformed = []
    #     for obj_pose in obj_poses:
    #         transformation = self.tfbuffer_.lookup_transform('map', "head_rgbd_sensor_rgb_frame", rospy.Time())
    #         transformed_pose = tf2_geometry_msgs.do_transform_pose(obj_pose, transformation)
    #         transformed.append(transformed_pose)
    #     return transformed

    def transform_head2base(self,obj_poses):
        transformed = []
        for obj_pose in obj_poses:
            transformation = self.tfbuffer_.lookup_transform('BASE', "camera_link", rospy.Time())
            transformed_pose = tf2_geometry_msgs.do_transform_pose(obj_pose, transformation)
            transformed.append(transformed_pose)
        return transformed


if __name__ == '__main__':
    detector = LeverDetector()
    detector.run()
