import numpy as np
import pybullet as p
import itertools

class Robot():
    """ 
    The class is the interface to a single robot
    """
    # switch=0
    p_des=np.zeros((6,2))
    def __init__(self, init_pos, robot_id, dt):
        self.id = robot_id
        self.dt = dt
        self.pybullet_id = p.loadSDF("../models/robot.sdf")[0]
        self.joint_ids = list(range(p.getNumJoints(self.pybullet_id)))
        self.initial_position = init_pos
        self.reset()

        # No friction between bbody and surface.
        p.changeDynamics(self.pybullet_id, -1, lateralFriction=5., rollingFriction=0.)

        # Friction between joint links and surface.
        for i in range(p.getNumJoints(self.pybullet_id)):
            p.changeDynamics(self.pybullet_id, i, lateralFriction=5., rollingFriction=0.)
            
        self.messages_received = []
        self.messages_to_send = []
        self.neighbors = []
        self.timer=0


    def reset(self):
        """
        Moves the robot back to its initial position 
        """
        p.resetBasePositionAndOrientation(self.pybullet_id, self.initial_position, (0., 0., 0., 1.))
            
    def set_wheel_velocity(self, vel):
        """ 
        Sets the wheel velocity,expects an array containing two numbers (left and right wheel vel) 
        """
        assert len(vel) == 2, "Expect velocity to be array of size two"
        p.setJointMotorControlArray(self.pybullet_id, self.joint_ids, p.VELOCITY_CONTROL,
            targetVelocities=vel)

    def get_pos_and_orientation(self):
        """
        Returns the position and orientation (as Yaw angle) of the robot.
        """
        pos, rot = p.getBasePositionAndOrientation(self.pybullet_id)
        euler = p.getEulerFromQuaternion(rot)
        return np.array(pos), euler[2]
    
    def get_messages(self):
        """
        returns a list of received messages, each element of the list is a tuple (a,b)
        where a= id of the sending robot and b= message (can be any object, list, etc chosen by user)
        Note that the message will only be received if the robot is a neighbor (i.e. is close enough)
        """
        return self.messages_received
        
    def send_message(self, robot_id, message):
        """
        sends a message to robot with id number robot_id, the message can be any object, list, etc
        """
        self.messages_to_send.append([robot_id, message])
        
    def get_neighbors(self):
        """
        returns a list of neighbors (i.e. robots within 2m distance) to which messages can be sent
        """
        return self.neighbors
    
    def compute_controller(self):
        
        self.timer+=1

        neig = self.get_neighbors()
        messages = self.get_messages()
        pos, rot = self.get_pos_and_orientation()
        
        #send message of positions to all neighbors indicating our position
        for n in neig:
            self.send_message(n, pos)
        
        dx = 0.
        dy = 0.

        print(self.timer)    
        
        if ((self.timer>0)&(self.timer<1000)):
            Robot.p_des = [[1,-1],[1,0],[1,1],[2,1],[2,-1],[2,0]]

        if ((self.timer>1000)&(self.timer<4000)):
            Robot.p_des=[[1.5,0.5],[2.5,0.5],[2.5,1.5],[2.5,3.5],[2.5,-0.5],[2.5,2.5]]

        if ((self.timer>4000)&(self.timer<5500)):
            Robot.p_des=[[2.5,2],[2.5,2.5],[2.5,3.5],[2.5,5.5],[2.5,1.5],[2.5,4.5]]

        if ((self.timer>5500)&(self.timer<7000)):
            Robot.p_des=[[2.5,3.5],[2.5,4.5],[2.5,5.5],[2.5,7.5],[2.5,2.5],[2.5,6.5]]

        if ((self.timer>6500)&(self.timer<10000)):
            Robot.p_des = [[0.5,5.5],[1.5,5.5],[2.5,5.5],[3.5,5.5],[4.5,5.5],[5.5,5.5]]

        if ((self.timer>10000)&(self.timer<11000)):
            Robot.p_des = [[-3.5,5.5],[-2.5,5.5],[-1.5,5.5],[-0.5,5.5],[0.5,5.5],[1.5,5.5]]

        if ((self.timer>11000)&(self.timer<12500)):
            Robot.p_des = [[-7.5,5.5],[-6.5,5.5],[-5.5,5.5],[-4.5,5.5],[-3.5,5.5],[-2.5,5.5]]

        if ((self.timer>12500)&(self.timer<14500)):
            Robot.p_des = [[-4,4.5],[-4.5,5],[-4,6.5],[-5,5.5],[-4,5.5],[-4.5,6]]

        if ((self.timer>14500)&(self.timer<15500)):
            Robot.p_des = [[-4,9.5],[-4.5,10],[-4,11.5],[-5,10.5],[-4,10.5],[-4.5,11]]

        if messages:
            for m in messages:
                dx += 1.5*(m[1][0] - pos[0] -Robot.p_des[m[0]][0] + Robot.p_des[self.id][0])
                dy += 1.5*(m[1][1] - pos[1] -Robot.p_des[m[0]][1] + Robot.p_des[self.id][1])

                dx += 4.5*np.minimum((Robot.p_des[self.id][0]-pos[0]),1)
                dy += 4.5*np.minimum((Robot.p_des[self.id][1]-pos[1]),1)
        
            #compute velocity change for the wheels
            vel_norm = np.linalg.norm([dx, dy]) #norm of desired velocity
            if vel_norm < 0.01:
                vel_norm = 0.01
            des_theta = np.arctan2(dy/vel_norm, dx/vel_norm)
            right_wheel = np.sin(des_theta-rot)*vel_norm + np.cos(des_theta-rot)*vel_norm
            left_wheel = -np.sin(des_theta-rot)*vel_norm + np.cos(des_theta-rot)*vel_norm
            self.set_wheel_velocity([left_wheel, right_wheel])
        

    
       
