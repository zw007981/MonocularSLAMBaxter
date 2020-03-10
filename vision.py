#!/usr/bin/env python
from std_msgs.msg import String
import rospy
import baxter_interface
import sys
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class file_saver():
    def __init__(self):
        self.image_pub = rospy.Publisher("cameras/left_hand_camera/image", Image, queue_size = 10)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("cameras/left_hand_camera/image", Image, self.save)
        self.saveFile = open('AWenZhou_record.txt', 'a')

    def save(self, data):
        global scene_number
        #duration = (rospy.Time.now()-last_time).to_sec()
        #Get end effector pose
        left = baxter_interface.Limb('left').endpoint_pose()
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        cv2.imshow("Image from Baxter's Camera", cv_image)
        #Save the image
        save_path = 'scene' + str(scene_number) + '.png'
        cv2.imwrite(save_path, cv_image)
        #Save the endpoint_pose
        Point = list(left['position'])
        Quaternion = list(left['orientation'])
        self.saveFile.write(str(Point[0]) + ' ' + str(Point[1]) + ' ' + str(Point[2]) + ' ')
        self.saveFile.write(str(Quaternion[0]) + ' ' + str(Quaternion[1]) + ' ' + str(Quaternion[2]) + ' ' + str(Quaternion[3]) + '\n')
        scene_number = scene_number + 1
        cv2.waitKey(88)

def main(args):
    rospy.init_node('file_saver', anonymous = True)
    global scene_number
    [scene_number, zw, timestep] = [0, 23, 10]
    last_time = rospy.Time.now()
    while True:
        current_time = rospy.Time.now()
        duration = (current_time - last_time).to_sec()
        if duration > timestep:
            ic = file_saver()
            last_time = current_time
        if scene_number > zw:
            break

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)


#rosrun baxter_tools camera_control.py -o left_hand_camera -r 1280x800
