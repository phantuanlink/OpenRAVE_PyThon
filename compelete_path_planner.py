import numpy as np
import openravepy as orpy
import rosgraph
import tf.transformations as tr

# Use the OO interface to avoid QT conflicts
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from matplotlib.figure import Figure



# load environment
env = orpy.Environment()
env.Load('osr_openrave/worlds/cubes_task.env.xml')
env.SetDefaultViewer()

#get robot and manipulator
robot = env.GetRobot('robot')
manipulator = robot.SetActiveManipulator('gripper')
robot.SetActiveDOFs(manipulator.GetArmIndices())
np.set_printoptions(precision=6, suppress=True) #set up printing to number Significant number

print("SOLUTION STARTS FROM HERE")
print(" ")

#move the joints to the box
qgrasp = [-0.463648, 0.68088 , 1.477533, 0, 0.98318 , -2.034444]
robot.SetActiveDOFValues(qgrasp)
planner = orpy.RaveCreatePlanner(env, 'birrt') # Using bidirectional RRT
params = orpy.Planner.PlannerParameters()
params.SetRobotActiveJoints(robot)
params.SetGoalConfig(qgrasp)
params.SetPostProcessing('ParabolicSmoother', '<_nmaxiterations>40</_nmaxiterations>')
planner.InitPlan(robot, params)

# Plan a trajectory
traj = orpy.RaveCreateTrajectory(env, '')
planner.PlanPath(traj)

# Execute the trajectory
controller = robot.GetController()
controller.SetPath(traj)

times = np.arange(0, traj.GetDuration(), 0.01)
qvect = np.zeros((len(times), robot.GetActiveDOF()))
spec = traj.GetConfigurationSpecification()
for i in range(len(times)):
  trajdata = traj.Sample(times[i])
qvect[i,:] = spec.ExtractJointValues(trajdata, robot, manipulator.GetArmIndices(), 0)


fig = Figure()
canvas = FigureCanvas(fig)
ax = fig.add_subplot(1,1,1)
ax.plot(times, qvect)
ax.grid(True)
ax.set_xlabel('Time [s]')
ax.set_ylabel('Joint Values [rad]')
canvas.print_figure('joint_values.png')


for i in range(1):
  raw_input('Press Enter fto exit')
