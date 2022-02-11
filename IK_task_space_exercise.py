import numpy as np
import openravepy as orpy
import tf.transformations as tr


env = orpy.Environment()
env.Load('osr_openrave/worlds/cubes_task.env.xml')
env.SetDefaultViewer()
robot = env.GetRobot('robot')
manipulator = robot.SetActiveManipulator('gripper')
robot.SetActiveDOFs(manipulator.GetArmIndices())
np.set_printoptions(precision=6, suppress=True)
# Box to illustrate elbow up/down
with env:
  box = orpy.RaveCreateKinBody(env, '')
  box.SetName('box')
  box.InitFromBoxes(np.array([[0.5, 0.3, 1, 0.01, 0.04, 0.22]]), True)
  env.AddKinBody(box)

box_centroid = box.ComputeAABB().pos()
print ("INVERT KINEMATIC EXERCISE")
print box_centroid

print ("perform INVERT KINEMATIC to pick up the box")
ikmodel = orpy.databases.inversekinematics.InverseKinematicsModel(robot, iktype=orpy.IkParameterization.Type.Transform6D)
if not ikmodel.load():
  ikmodel.autogenerate()
Tgrasp = tr.quaternion_matrix([ 0.5,  0.5,  0.5, -0.5])
Tgrasp[:3,3] = box_centroid
solutions = manipulator.FindIKSolutions(Tgrasp, 0)
print solutions

robot.SetActiveDOFValues(solutions[4])

#solve for collision free solution
solutions = manipulator.FindIKSolutions(Tgrasp, orpy.IkFilterOptions.CheckEnvCollisions)
print solutions
