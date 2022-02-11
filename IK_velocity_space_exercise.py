import numpy as np
import openravepy as orpy


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


qgrasp = [0.170333,  0.884552,  1.161062,  1.649269,  1.41946,  -0.48077 ]
robot.SetActiveDOFValues(qgrasp)

# Close the gripper and grab the box
taskmanip = orpy.interfaces.TaskManipulation(robot)
taskmanip.CloseFingers()
robot.WaitForController(0)
robot.Grab(box)

#define velocity vector (6D)
twist = np.array([0, 0, 0.01, 0, 0, 0])

#get link index of end effector
link_idx = [l.GetName() for l in robot.GetLinks()].index('robotiq_85_base_link')
#get end-effector location
link_origin = robot.GetLink('robotiq_85_base_link').GetTransform()[:3,3]

#initialize empty jacobian matrix
J = np.zeros((6,6))
q = robot.GetActiveDOFValues()

for i in range(30):
  J[:3,:] = robot.ComputeJacobianTranslation(link_idx, link_origin)[:,:6]
  J[3:,:] = robot.ComputeJacobianAxisAngle(link_idx)[:,:6]
  qdot = np.linalg.solve(J, twist)
  q[:6] += qdot
  robot.SetActiveDOFValues(q)
  raw_input('Press Enter for next differential IK step')