import openravepy as orpy
import numpy as np


env = orpy.Environment()
env.Load('osr_openrave/robots/denso_robotiq_85_gripper.robot.xml')
env.SetDefaultViewer()
robot = env.GetRobot('denso_robotiq_85_gripper')
manipulator = robot.SetActiveManipulator('gripper')
robot.SetActiveDOFs(manipulator.GetArmIndices())

robot.SetActiveDOFValues([0.1, 0.7, 1.5, -0.5, -0.8, -1.2])
print manipulator.GetEndEffectorTransform()

robot.SetActiveDOFValues([0.8, -0.4, 1.5, -0.5, -0.8, -1.2])
print manipulator.GetEndEffectorTransform()

robot.GetLinks()

link_idx = [l.GetName() for l in robot.GetLinks()].index('robotiq_85_base_link')
link_origin = robot.GetLink('robotiq_85_base_link').GetTransform()[:3,3]
# Improve the visualization settings
import numpy as np
np.set_printoptions(precision=6, suppress=True)
# Print the result
print robot.ComputeJacobianTranslation(link_idx, link_origin)

print robot.ComputeJacobianAxisAngle(link_idx)

print("Exercise FK")

#define configuration space
q1 = np.array([-0.1, 1.8, 1.0, 0.5, 0.2, 1.3])
qdot1 = np.array([1.2, -0.7, -1.5, -0.5, 0.8, -1.5]) #joint velocities
delta_t = 0.1

#############
# Questions:
# 1. Compute the transformation matrix of the  robotiq_85_base_link  at configurations  q1  and  q1 + delta_t*qdot1 .
# Compute the linear and angular Jacobians of the  robotiq_85_base_link  at time  t1 .
# Using these Jacobians, compute another approximation of the transformation matrix of the  robotiq_85_base_link  at time  q1 + delta_t*qdot1 .

print("Quesion 1")
robot.SetActiveDOFValues(q1)
print manipulator.GetEndEffectorTransform()
print ("------------------")
robot.SetActiveDOFValues(q1 + delta_t*qdot1)
print manipulator.GetEndEffectorTransform()

print("Question 2")
link_idx = [l.GetName() for l in robot.GetLinks()].index('robotiq_85_base_link')
link_origin = robot.GetLink('robotiq_85_base_link').GetTransform()[:3,3]
# Improve the visualization settings
np.set_printoptions(precision=6, suppress=True)
# Print the result
print robot.ComputeJacobianTranslation(link_idx, link_origin)

print robot.ComputeJacobianAxisAngle(link_idx)
