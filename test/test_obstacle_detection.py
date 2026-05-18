
import roslib
import sys
import unittest
import rosunit
import rostest
from geometry_msgs.msg import TransformStamped

class TestObstacleDetection(unittest.TestCase):
    def test_dynamic_obstacles(self):


        #Sequence (obstacle_description_index)
        #Publish /vicon/obstacle_chair_1/obstacle_chair_1
        #Publish /vicon/obstacle_chair_2/obstacle_chair_2
        #Publish /vicon/obstacle_controller_1/obstacle_controller_1
        

        pub_chair_1 = rospy.Publisher("/vicon/obstacle_chair_1/obstacle_chair_1", TransformStamped, queue_size = 10)
        pub_chair_2 = rospy.Publisher("/vicon/obstacle_chair_2/obstacle_chair_2", TransformStamped, queue_size = 10)
        pub_controller_1 = rospy.Publisher("/vicon/obstacle_controller_1/obstacle_controller_1", TransformStamped, queue_size = 10)
        rate = rospy.Rate(10)

        topic_names = ["chair_1", "chair_2", "controller_1"]

        while not rospy.is_shutdown():
            #Make guidarservices instance
            guidar = GuidARServices()
            guidar._get_obstacles()
            messages = []

            for i in range(len(classes)): 
            
                transform_stamped = TransformStamped()
                transform_stamped.transform.translation.x = string_to_float(opic_names[i])
                transform_stamped.transform.translation.y = string_to_float(opic_names[i])
                transform_stamped.transform.translation.z = string_to_float(opic_names[i])


                # 3. Populate the Rotation (Identity quaternion)
                transform_stamped.transform.rotation.x = string_to_float(opic_names[i])
                transform_stamped.transform.rotation.y = string_to_float(opic_names[i])
                transform_stamped.transform.rotation.z = string_to_float(opic_names[i])
                transform_stamped.transform.rotation.w = string_to_float(opic_names[i])
                messages.append(transform_stamped)
            
            #Publish info
            
            pub_chair_1.publish(messages[0])
            pub_chair_2.publish()
            #Make sure guidar's class info checks out

            #Assert for chair 1

            self.assertEquals(guid)
            
            #Assert for chair 2

            #assert for controller 1
                     

        #Make sure we are retrieving in the guidAR class those obstacles 


if __name__ == "__main__":
    rosunit.unitrun("guidar","testObstacleDetection",TestObstacleDetection)
