#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_srvs.srv import Empty
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed,Bumper,HeadTouch
from sensor_msgs.msg import Image,JointState
from cv_bridge import CvBridge, CvBridgeError
import cv2

class Central:


    def __init__(self):
        # initialize class variables
        self.joint_names = []
        self.joint_angles = []
        self.joint_velocities = []
        self.jointPub = 0
        self.stiffness = False
        self.button1_fkt = False
        self.button2_fkt = False
        self.button3_fkt = False

        pass


    def key_cb(self,data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

    def joints_cb(self,data):
        #rospy.loginfo("joint states "+str(data.name)+str(data.position))
        # store current joint information in class variables
        self.joint_names = data.name 
        self.joint_angles = data.position
        self.joint_velocities = data.velocity

        pass

    def bumper_cb(self,data):
        rospy.loginfo("bumper: "+str(data.bumper)+" state: "+str(data.state))
        if data.bumper == 0:
            self.stiffness = True
        elif data.bumper == 1:
            self.stiffness = False

    def touch_cb(self,data):
        rospy.loginfo("touch button: "+str(data.button)+" state: "+str(data.state))
        if data.button == 1 and data.state == 1:
            self.button1_fkt = True #
            self.button2_fkt = False
            
        if data.button == 2 and data.state == 1:
            self.button1_fkt = False #
            self.button2_fkt = True
            
        if data.button == 3 and data.state == 1:
            self.button1_fkt = False #
            self.button2_fkt = False
            self.button3_fkt = !self.button3_fkt

    def image_cb(self,data):
        bridge_instance = CvBridge()
        try:
            cv_image = bridge_instance.imgmsg_to_cv2(data,"bgr8")
            # convert to hsv color space
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            # mask of red, found the values online
            mask = cv2.inRange(hsv, np.array([90-10, 70, 50]), np.array([90 + 10, 255, 255]))
            # find contours around the masked image
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            blob = max(contours, key=lambda el: cv2.contourArea(el))
            M = cv2.moments(blob)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            print('biggest blob center: ', center)
            # cv2.imshow('mask',mask)
        except CvBridgeError as e:
            rospy.logerr(e)
        
        cv2.imshow("image window",cv_image)
        cv2.waitKey(3) # a small wait time is needed for the image to be displayed correctly

    # sets the stiffness for all joints. can be refined to only toggle single joints, set values between [0,1] etc
    def set_stiffness(self,value): # TODO set each joint individually
        if value == True:
            service_name = '/body_stiffness/enable'
        elif value == False:
            service_name = '/body_stiffness/disable'
        try:
            stiffness_service = rospy.ServiceProxy(service_name,Empty)
            stiffness_service()
        except rospy.ServiceException, e:
            rospy.logerr(e)

    def set_joint_angles(self, joint_name, angle_value):

        joint_angles_to_set = JointAnglesWithSpeed()
        joint_angles_to_set.joint_names.append(joint_name) # each joint has a specific name, look into the joint_state topic or google
        joint_angles_to_set.joint_angles.append(angle_value) # the joint values have to be in the same order as the names!!
        joint_angles_to_set.relative = False # if true you can increment positions
        joint_angles_to_set.speed = 0.1 # keep this low if you can
        self.jointPub.publish(joint_angles_to_set)

 def right_arm_mirror(self):
        self.set_stiffness(True) # don't let the robot stay enabled for too long, the motors will overheat!! (don't go for lunch or something)
        rospy.sleep(0.05)

        self.set_joint_angles("RShoulderPitch", self.joint_angles[self.joint_names.index("LShoulderPitch")])
        self.set_joint_angles("RShoulderRoll", self.joint_angles[self.joint_names.index("RShoulderRoll")])
        self.set_joint_angles("RElbowYaw", self.joint_angles[self.joint_names.index("RElbowYaw")])
        self.set_joint_angles("RElbowRoll", self.joint_angles[self.joint_names.index("RElbowRoll")])
        self.set_joint_angles("RWristYaw", self.joint_angles[self.joint_names.index("RWristYaw")])
        self.set_joint_angles("RHand", self.joint_angles[self.joint_names.index("RHand")])
        rospy.sleep(0.05)
        #self.set_stiffness(False) # don't let the robot stay enabled for too long, the motors will overheat!! (don't go for lunch or something)

def left_arm_home(self):
        # test sequence to demonstrate setting joint angles
        self.set_stiffness(True) # don't let the robot stay enabled for too long, the motors will overheat!! (don't go for lunch or something)
        rospy.sleep(1.0)
        self.set_joint_angles("LShoulderPitch", 0.6856560707092285)
        self.set_joint_angles("LShoulderRoll", 0.2638061046600342)
        self.set_joint_angles("LElbowYaw", -0.5676219463348389)
        self.set_joint_angles("LElbowRoll", -0.8682019710540771)
        self.set_joint_angles("LWristYaw", -1.2165040969848633)
        self.set_joint_angles("LHand", 0.27799999713897705)
        rospy.sleep(3.0)
        self.set_stiffness(False) # always check that your robot is in a stable position before disabling the stiffness!!

def wave_left_arm(self):
        # test sequence to demonstrate setting joint angles
        self.set_stiffness(True) # don't let the robot stay enabled for too long, the motors will overheat!! (don't go for lunch or something)
        rospy.sleep(1.0)
        # left arm home angels
        self.set_joint_angles("LShoulderPitch", 0.6856560707092285)
        self.set_joint_angles("LShoulderRoll", 0.2638061046600342)
        self.set_joint_angles("LElbowYaw", -0.5676219463348389)
        self.set_joint_angles("LElbowRoll", -0.8682019710540771)
        self.set_joint_angles("LWristYaw", -1.2165040969848633)
        self.set_joint_angles("LHand", 0.27799999713897705)

        rospy.sleep(1.0)
        # left arm up pose
        self.set_joint_angles("LShoulderPitch", -1.1167941093444824)
        self.set_joint_angles("LShoulderRoll", 0.9295620918273926)
        self.set_joint_angles("LElbowYaw", -0.6121079921722412)
        self.set_joint_angles("LElbowRoll", -0.7009961605072021)
        self.set_joint_angles("LWristYaw", -1.5677900314331055)
        self.set_joint_angles("LHand", 0.2784000039100647)
        rospy.sleep(1.0)
        self.set_stiffness(False) # always check that your robot is in a stable position before disabling the stiffness!!


    def central_execute(self):
        rospy.init_node('central_node',anonymous=True) #initilizes node, sets name

        # create several topic subscribers
        rospy.Subscriber("key", String, self.key_cb)
        rospy.Subscriber("joint_states",JointState,self.joints_cb)
        rospy.Subscriber("bumper",Bumper,self.bumper_cb)
        rospy.Subscriber("tactile_touch",HeadTouch,self.touch_cb)
        rospy.Subscriber("/nao_robot/camera/top/camera/image_raw",Image,self.image_cb)
        self.jointPub = rospy.Publisher("joint_angles",JointAnglesWithSpeed,queue_size=10)


        # test sequence to demonstrate setting joint angles
        self.set_stiffness(True) # don't let the robot stay enabled for too long, the motors will overheat!! (don't go for lunch or something)
        rospy.sleep(1.0)
        self.set_joint_angles("LShoulderPitch", 0.5)
        rospy.sleep(3.0)
        self.set_joint_angles("LShoulderPitch", 0.0)
        rospy.sleep(3.0)
        self.set_stiffness(False) # always check that your robot is in a stable position before disabling the stiffness!!

        rate = rospy.Rate(10) # sets the sleep time to 10ms

        while not rospy.is_shutdown():
            self.set_stiffness(self.stiffness)
            if self.button1_fkt:
                self.left_arm_home()
                self.button1_fkt = False
            if self.button2_fkt:
                self.wave_left_arm()
                self.button2_fkt = False
            if self.button3_fkt:
                self.right_arm_mirror()

            rate.sleep()

    # rospy.spin() just blocks the code from exiting, if you need to do any periodic tasks use the above loop
    # each Subscriber is handled in its own thread
    rospy.spin()

if __name__=='__main__':
    # instantiate class and start loop function
    central_instance = Central()
    central_instance.central_execute()
