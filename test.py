import gym
import vrep_env

env = gym.make('vrep_soccer-v0')

motor_names = ['LeftMotor', 'RightMotor']
object_names = ['Bola']
robot_names = ['DifferentialDriveRobot']

num_simulations = 0
while(num_simulations < 2):
	env.reset()
	dt = 0
	while (dt < 1):
		state_info,reward, done, info = env.step(dict(zip(motor_names, [10,10])))
		dt += env.time_step
		print(reward)
	num_simulations += 1
