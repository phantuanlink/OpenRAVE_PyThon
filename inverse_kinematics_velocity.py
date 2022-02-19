import numpy as np
import openravepy as orpy
import rosgraph
import tf.transformations as tr

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

# Box to illustrate elbow up/down
with env:
  box = orpy.RaveCreateKinBody(env, '')
  box.SetName('box')
  box.InitFromBoxes(np.array([[0.5, 0.3, 1, 0.01, 0.04, 0.22]]), True)
  env.AddKinBody(box)

box_centroid = box.ComputeAABB().pos()
print box_centroid
ikmodel = orpy.databases.inversekinematics.InverseKinematicsModel(robot, iktype=orpy.IkParameterization.Type.Transform6D)
if not ikmodel.load():
  ikmodel.autogenerate()
Tgrasp = tr.quaternion_matrix([ 0.5,  0.5,  0.5, -0.5])
Tgrasp[:3,3] = box_centroid
solutions = manipulator.FindIKSolutions(Tgrasp, orpy.IkFilterOptions.CheckEnvCollisions)
print solutions

qgrasp = solutions[0] #one of the solution from inverse kinmatics
#move the joints to the box
robot.SetActiveDOFValues(qgrasp)


# Close the gripper and grab the box
taskmanip = orpy.interfaces.TaskManipulation(robot)
taskmanip.CloseFingers()
robot.WaitForController(0)
robot.Grab(box) #make the box stick to the end-effector of the robot

#define the destination velocity space
twist = np.array([0, 0, 0.01, 0, 0, 0])

#perform the IK in velodity space using built in numpy.linalg.solve
link_idx = [l.GetName() for l in robot.GetLinks()].index('robotiq_85_base_link')
link_origin = robot.GetLink('robotiq_85_base_link').GetTransform()[:3,3]
J = np.zeros((6,6))
q = robot.GetActiveDOFValues()

for i in range(15):
  J[:3,:] = robot.ComputeJacobianTranslation(link_idx, link_origin)[:,:6]
  J[3:,:] = robot.ComputeJacobianAxisAngle(link_idx)[:,:6]
  qdot = np.linalg.solve(J, twist)
  q[:6] += qdot
  robot.SetActiveDOFValues(q)
  raw_input('Press Enter for next differential IK step')



print(" ")
print("SOLUTION ENDS FROM HERE")