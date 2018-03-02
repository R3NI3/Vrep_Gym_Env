import gym
import vrep_env

env = gym.make('vrep_soccer-v0')

motor_names = ['LeftMotor', 'RightMotor']
object_names = ['Bola']
robot_names = ['DifferentialDriveRobot']

def flatten(list):
    return [item for sublist in list for item in sublist]

print(env.observation_space.shape)
print(env.action_space)
print (env.getSimulationState())
num_simulations = 0
while(num_simulations < 2):
	env.reset()
	dt = 0
	while (dt < 1):
		state_info,reward, done, info = env.step([0.1,0])
		dt += env.time_step
		print (env.getSimulationState())
		print(reward)
	num_simulations += 1
