#import os, subprocess, time, signal
import gym
from gym import error, spaces
from gym import utils
from gym.utils import seeding

import vrep
import time
import numpy as np

class VrepSoccerEnv(gym.Env, utils.EzPickle):
    metadata = {'render.modes': ['human']}

    def __init__(self):
        ip = '127.0.0.1'
        port = 19997
        time_step = 0.01
        actuator_names = ['LeftMotor', 'RightMotor']
        object_names = ['Bola']
        robot_names = ['DifferentialDriveRobot']
        scene_path = './Cenario.ttt'
         # Connect to the V-REP continuous server
        vrep.simxFinish(-1)
        self.clientID = vrep.simxStart(ip, port, True, True, -500000, 5)
        if self.clientID != -1: # if we connected successfully
            print ('Connected to remote API server')
        else:
            raise Exception()
        self._configure_environment(scene_path)
        # -------Setup the simulation
        vrep.simxSynchronous(self.clientID, True) #if we need to be syncronous
        # move simulation ahead one time step
        vrep.simxSynchronousTrigger(self.clientID)

        vrep.simxSetFloatingParameter(self.clientID,
                vrep.sim_floatparam_simulation_time_step,
                time_step, # specify a simulation time step
                vrep.simx_opmode_oneshot)

        self.get_handles(actuator_names, robot_names, object_names)

        self.time_step = time_step
        #todo check if observation_space and action_space are correct
        shape = len(robot_names)*6 + len(object_names)*6
        self.observation_space = spaces.Box(low=0, high=1700, shape=[shape])
        shape = len(actuator_names)
        self.action_space = spaces.Box(low=-10, high=10, shape=[shape])

    def __del__(self):
        vrep.simxPauseSimulation(self.clientID, vrep.simx_opmode_blocking)
        time.sleep(.05)
        vrep.simxFinish(self.clientID)

    def _configure_environment(self, scene_path):
        """
        Provides a chance for subclasses to override this method and supply
        a different server configuration. By default, we initialize one
        offense agent against no defenders.
        """
        vrep.simxLoadScene(self.clientID, scene_path, 1,vrep.simx_opmode_blocking)

    def step(self, action):
        self._take_action(action)
        state_info = self.getSimulationState()
        return state_info,self.get_reward(),False, None

    def _take_action(self, action):
        for idx,(key,motor_handle) in enumerate(self.act_handles.items()):
            vrep.simxSetJointTargetVelocity(self.clientID, motor_handle,
                        action[key], # target velocity
                        vrep.simx_opmode_blocking)
        vrep.simxSynchronousTrigger(self.clientID)

    def get_reward(self):
        state_info = self.state
        reward = 0
        robot_pos = np.array(state_info['DifferentialDriveRobot'][0])
        target_pos = np.array(state_info['Bola'][0])
        distance = np.linalg.norm(robot_pos - target_pos)
        reward = 1/ distance if distance != 0 else 1000
        return reward

    def reset(self):
        vrep.simxStopSimulation(self.clientID, vrep.simx_opmode_blocking)
        time.sleep(.05)
        vrep.simxStartSimulation(self.clientID, vrep.simx_opmode_blocking)
        time.sleep(.05)
        return self.getSimulationState()

    def render(self):
        pass

    def stop_robot(self, actuator_names):
        motor_handles = [self.act_handles[act_name] for act_name in actuator_names
                            if act_name in self.act_handles]
        for ii,motor_handle in enumerate(motor_handles):
            # if force has changed signs,
            # we need to change the target velocity sign
            vrep.simxSetJointTargetVelocity(self.clientID,
                        motor_handle,
                        0, # target velocity
                        vrep.simx_opmode_blocking)
        # move simulation ahead one time step
        vrep.simxSynchronousTrigger(self.clientID)

    def get_position(self, obj_name):
        if obj_name in self.ddr_handles:
            obj_handle = self.ddr_handles[obj_name]
        elif obj_name in self.obj_handles:
            obj_handle = self.obj_handles[obj_name]
        elif obj_name in self.act_handles:
            obj_handle = self.act_handles[obj_name]
        else:
            return -1

        _, obj_xyz = vrep.simxGetObjectPosition(self.clientID, obj_handle,
                -1, # retrieve absolute, not relative, position
                vrep.simx_opmode_blocking)
        if _ !=0 : raise Exception()
        else: return obj_xyz

    def get_orientation(self, obj_name):
        if obj_name in self.ddr_handles:
            obj_handle = self.ddr_handles[obj_name]
        elif obj_name in self.obj_handles:
            obj_handle = self.obj_handles[obj_name]
        elif obj_name in self.act_handles:
            obj_handle = self.act_handles[obj_name]
        else:
            return -1

        _, obj_ang = vrep.simxGetObjectOrientation(self.clientID, obj_handle,
                -1, # retrieve absolute, not relative, position
                vrep.simx_opmode_blocking)
        if _ !=0 : raise Exception()
        else: return obj_ang

    def setJointVelocity(self, motor_names, target_velocity):
        for idx,motor_name in enumerate(motor_names):
            if motor_name in self.act_handles:
                vrep.simxSetJointTargetVelocity(self.clientID, self.act_handles[motor_name],
                            target_velocity[idx], # target velocity
                            vrep.simx_opmode_blocking)
            else:
                return -1
        vrep.simxSynchronousTrigger(self.clientID)
        return 0

    def getSimulationState(self):
        self.state = {}
        for name, handle in self.ddr_handles.items():
            self.state[name] = [self.get_position(name),self.get_orientation(name)]
        for name, handle in self.obj_handles.items():
            self.state[name] = [self.get_position(name),self.get_orientation(name)]

        return self.state

    def get_handles(self, actuator_names, robot_names, object_names):
        # get the handles for each motor and set up streaming
        self.act_handles = {}
        for name in actuator_names:
            _, obj_handle = vrep.simxGetObjectHandle(self.clientID,
                    name, vrep.simx_opmode_blocking)
            if _ !=0 : raise Exception()
            self.act_handles.update({name:obj_handle})

        # get handle for target and set up streaming
        self.obj_handles = {}
        for name in object_names:
            _, obj_handle = vrep.simxGetObjectHandle(self.clientID,
                    name, vrep.simx_opmode_blocking)
            if _ !=0 : raise Exception()
            self.obj_handles.update({name:obj_handle})

        # get robot handle
        self.ddr_handles = {}
        for name in robot_names:
            _, obj_handle = vrep.simxGetObjectHandle(self.clientID,
                    name, vrep.simx_opmode_blocking)
            if _ !=0 : raise Exception()
            self.ddr_handles.update({name:obj_handle})

    def startSimulation(self):
        vrep.simxStartSimulation(self.clientID, vrep.simx_opmode_blocking)


