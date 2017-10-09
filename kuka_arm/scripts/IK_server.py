#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
		
        ### Your FK code here
        # Create symbols
		alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7') # twist angles
		a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') # link length
		d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') # link offset
		q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') # joint angle: theta_i
    	
		# Create Modified DH parameters
		DH = {alpha0:     0,  a0:      0,   d1:  0.75,
			  alpha1: -pi/2,  a1:   0.35,   d2:     0,   q2: q2-pi/2,
			  alpha2:     0,  a2:   1.25,   d3:     0,
			  alpha3: -pi/2,  a3: -0.054,   d4:  1.50,
			  alpha4: -pi/2,  a4:      0,   d5:     0,
			  alpha5: -pi/2,  a5:      0,   d6:     0,
			  alpha6:     0,  a6:      0,   d7: 0.303,   q7:       0}
		# Define Modified DH Transformation matrix
		def TF_Matrix(alpha, a, d, q):
			TF = Matrix([[	    cos(q),           -sin(q),           0,             a],
					[sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
					[sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
					[                0,                 0,           0,             1]])
			return TF
		# base_link to link1
		T0_1 = TF_Matrix(alpha0, a0, d1, q1).subs(DH)

		# link1 to link2
		T1_2 = TF_Matrix(alpha1, a1, d2, q2).subs(DH)

		# link2 to link3
		T2_3 = TF_Matrix(alpha2, a2, d3, q3).subs(DH)

		# link3 to link4
		T3_4 = TF_Matrix(alpha3, a3, d4, q4).subs(DH)

		# link4 to link5
		T4_5 = TF_Matrix(alpha4, a4, d5, q5).subs(DH)

		# link5 to link6
		T5_6 = TF_Matrix(alpha5, a5, d6, q6).subs(DH)

		# link6 to gripper_link
		T6_EE = TF_Matrix(alpha6, a6, d7, q7).subs(DH)

		T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE

		r,p,y = symbols('r p y') 
		# Extract rotation matrices from the transformation matrices
		R_x = Matrix([[1,      0,       0],
					  [0, cos(r), -sin(r)],
					  [0, sin(r),  cos(r)]]) # Roll

		R_y = Matrix([[ cos(p), 0,  sin(p)],
					  [     0,  1,       0],
					  [-sin(p), 0,  cos(p)]]) #Pitch

		R_z = Matrix([[cos(y), -sin(y), 0],
					  [sin(y),  cos(y), 0],
					  [     0,       0, 1]]) # Yaw

		R_EE = R_z * R_y * R_x
		R_error = R_z.subs(y, radians(180)) * R_y.subs(p, radians(-90))
		R_EE = R_EE * R_error
		# Total Homogeneous Transform Between base_link and gripper_link with Orientation Correction Applied

        ###

        # Initialize service response
		joint_trajectory_list = []
		for x in xrange(0, len(req.poses)):
            # IK code starts here
			joint_trajectory_point = JointTrajectoryPoint()

	    	# Extract end-effector position and orientation from request
	    	# px,py,pz = end-effector position
	    	# roll, pitch, yaw = end-effector orientation
			px = req.poses[x].position.x
			py = req.poses[x].position.y
			pz = req.poses[x].position.z

			(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
     		
            ### Your IK code here
			
			R_EE = R_EE.subs({'r':roll, 'p':pitch, 'y':yaw})

			EE = Matrix([[px],[py],[pz]])
			WC = EE - 0.303 * R_EE[:,2] # wrist center position
			
			# Calculate joint angles using Geometric IK method
			theta1 = atan2(WC[1],WC[0])

			# SSS triangle for theta2 and theta3
			l25 = sqrt(pow((sqrt(pow(WC[0],2) + pow(WC[1],2)) - a1.subs(DH)), 2) + pow((WC[2] - d1.subs(DH)), 2))
			l35 = sqrt(pow(a3.subs(DH),2)+pow(d4.subs(DH),2))

			angle_a = acos((pow(l25,2) + pow(a2.subs(DH),2) - pow(l35,2))/(2*l25*a2.subs(DH)))
			angle_b = acos((pow(l35,2) + pow(a2.subs(DH),2) - pow(l25,2))/(2*l35*a2.subs(DH)))
			angle_c = acos((pow(l35,2) + pow(l25,2) - pow(a2.subs(DH),2))/(2*l35*l25))
			angle_d = atan2(WC[1]- d1.subs(DH),WC[0]-a1.subs(DH))
			theta2 = pi/2 - angle_a - angle_d

			angle_e = pi/2 - theta2
			angle_f = atan2(a3.subs(DH),d4.subs(DH))

			theta3 = angle_e - angle_f - pi/2

			R0_3 = T0_1[0:3,0:3]*T1_2[0:3,0:3]*T2_3[0:3,0:3]
			R0_3 = R0_3.evalf(subs={q1:theta1, q2:theta2, q3:theta3})

			R3_6 = R0_3.inv("LU")*R_EE
			
			# Euler angels from rotation matrix
			theta4 = atan2(R3_6[2,2], -R3_6[0,2])
			theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]),R3_6[1,2])
			theta6 = atan2(-R3_6[1,1], R3_6[1,0])
            ###
		
            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
			joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
			joint_trajectory_list.append(joint_trajectory_point)

		rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
		return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
