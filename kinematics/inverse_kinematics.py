'''In this exercise you need to implemente inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinemtatics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinemtatics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinemtatics implementation.
'''


from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity


class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = {}
        for chain in self.chains[effector_name]:
            joint_angles[chain] = self.joints[chain]

        error = 0
        error_limit = 1e-4
        lambda_ = 0.001
        theta=0
        target = self.from_trans(transform)
        chain_of_effector = self.chains[effector_name]

        while True:
            self.forward_kinematics(joint_angles)
            T = []
            for chain in self.chains:
                T.append(self.transforms[chain])

            Te = np.matrix([self.from_trans(T[-1])])
            e = target - Te
            T = np.matrix([self.from_trans(i) for i in T[0:-1]])
            J = Te - T
            J = J.T
            J[-1, :] = 1  # angular velocity
            JJT = np.dot(J, J.T)
            d_theta = lambda_ * J.T * JJT.I * e.T
            theta += np.asarray(d_theta.T)[0]

            i = 0
            for chain in self.chains:
                joint_angles[chain] += np.asarray(d_theta.T)[0][i]
                i +=1
            error = np.linalg.norm(d_theta)
            if error < error_limit:
                return joint_angles
	 
        for chain in self.chains[effector_name]:
            joint_angles[chain] = self.joints[chain]

        error = 0
        error_limit = 1e-4
        lambda_ = 0.001
        theta=0
        target = self.from_trans(transform)
        chain_of_effector = self.chains[effector_name]

        while True:
            self.forward_kinematics(joint_angles)
            T = []
            for chain in self.chains:
                T.append(self.transforms[chain])

            Te = np.matrix([self.from_trans(T[-1])])
            e = target - Te
            T = np.matrix([self.from_trans(i) for i in T[0:-1]])
            J = Te - T
            J = J.T
            J[-1, :] = 1  # angular velocity
            JJT = np.dot(J, J.T)
            d_theta = lambda_ * J.T * JJT.I * e.T
            theta += np.asarray(d_theta.T)[0]

            i = 0
            for chain in self.chains:
                joint_angles[chain] += np.asarray(d_theta.T)[0][i]
                i +=1
            error = np.linalg.norm(d_theta)
            if error < error_limit:
                return joint_angles
		
	
    	
   def from_trans(self, T):
        theta_x, theta_y, theta_z = 0,0,0
        #find the angle from the transform matrix
        if T[0, 0] == 1:
            teta_x = np.arctan2(T[2, 1], T[1, 1])
        
        elif T[1, 1] == 1:
            teta_y = np.arctan2(T[0, 2], T[0, 0])
        
        elif T[2, 2] == 1:
            teta_z = np.arctan2(T[1, 0], T[0, 0])

	return np.array([T[3, 0], T[3, 1], T[3, 2], teta_x, teta_y, teta_z])

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        self.keyframes = ([], [], [])  # the result joint angles have to fill in

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = 0.26
    agent.set_transforms('LLeg', T)
    agent.run()
