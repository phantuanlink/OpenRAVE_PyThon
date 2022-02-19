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

#get box position
box_centroid = box.ComputeAABB().pos()
print box_centroid

print(" ")
print("Solve Inverse Kinematics")
print(" ")

# initialize model, passing in IK model and robot
ikmodel = orpy.databases.inversekinematics.InverseKinematicsModel(robot, iktype=orpy.IkParameterization.Type.Transform6D)
if not ikmodel.load():
  ikmodel.autogenerate()

Tgrasp = tr.quaternion_matrix([ 0.5,  0.5,  0.5, -0.5])
print Tgrasp
print(" ")
Tgrasp[:3,3] = box_centroid
print Tgrasp
print(" ")
solutions = manipulator.FindIKSolutions(Tgrasp, 0)
print solutions

print(" ")
print("SOLUTION ENDS FROM HERE")