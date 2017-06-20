'''In this exercise you need to implement forward kinematics for NAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint in function
       ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    the local_trans has to consider different joint axes and link parameters for differnt joints
'''

# add PYTHONPATH
import os
import sys
import math
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

from numpy.matlib import matrix, identity

from angle_interpolation import AngleInterpolationAgent


class ForwardKinematicsAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain
        self.chains = {'Head': ['HeadYaw', 'HeadPitch'],
			'LArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw'],
			'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'],
			'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll'],
			'RArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw']
                       }

    def think(self, perception):
        self.forward_kinematics(perception.joint)
	
        return super(ForwardKinematicsAgent, self).think(perception)

    def local_trans(self, joint_name, joint_angle):
        '''calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        '''
        T = matrix(4)


       	s = math.sin(joint_angle)
	c = math.cos(joint_angle)


	Param =          {'HeadYaw': [0,0,0], 
                                'HeadPitch': [0,0,0],
                                'LShoulderPitch': [0,0,0], 
                                'LShoulderRoll': [0,0,0],
                                'LElbowYaw': [105,15,0], 
                                'LElbowRoll': [0,0,0], 
                                'LWristYaw': [55.95,0,0],
                                'RShoulderPitch': [0,0,0], 
                                'RShoulderRoll': [0,0,0],
                                'RElbowYaw': [105,-15,0], 
                                'RElbowRoll': [0,0,0], 
                                'RWristYaw': [55.95,0,0],
                                'LHipYawPitch': [0,0,0], 
                                'LHipRoll': [0,0,0], 
                                'LHipPitch': [0,0,0], 
                                'LKneePitch': [0,0,-100],  
                                'LAnkleRoll': [0,0,0],
                                'LAnklePitch': [0,0,-102.9],
                                'RHipYawPitch': [0,0,0], 
                                'RHipRoll': [0,0,0], 
                                'RHipPitch': [0,0,0], 
                                'RKneePitch': [0,0,-100], 
                                'RAnkleRoll': [0,0,0], 
                                'RAnklePitch': [0,0,-102.9]}

 # dictionary of translation, problem with names?? Joint_name is the name of the next joint or the name of the joint before transformation ? 
			
	
	Trans = Param[joint_name]

	Ry = ([[c,0,s,Trans[0]], #Rotation pitch around y axis
		[0,1,0,Trans[1]],
		[-s,0,c,Trans[2]],
		[0,0,0,1]])

	Rz = ([[c,-s,0,Trans[0]], #Rotation yaw around z axis
		[s,c,0,Trans[1]],
		[0,0,1,Trans[2]],
		[0,0,0,1]])

	Rx = ([[1,0,0,Trans[0]], #Rotation roll around x axis
		[0,c,-s,Trans[1]],
		[0,s,c,Trans[2]],
		[0,0,0,1]])
	
 

	if 'Pitch' in joint_name:
		T = Ry
	elif 'Yaw' in joint_name:
		T = Rz
	elif 'Roll' in joint_name:
		T = Rx	

        return T

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
	i = 1
        for chain_joints in self.chains.values():
            T = identity(4)
            for joint in chain_joints:
		if joint == 'LWristYaw' or joint == 'RWristYaw': 
			angle = 0
		else:
                	angle = joints[joint]
                
		Tl = self.local_trans(joint, angle)
                T = T*Tl
		

		

                self.transforms[joint] = T
		print chain_joints
		print T

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()
