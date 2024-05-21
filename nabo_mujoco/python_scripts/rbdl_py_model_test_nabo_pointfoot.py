import numpy as np
import rbdl

model = rbdl.Model()

LenThigh = 0.2
LenShank = 0.2
HipY = 0.1
HipZ = -0.1
maxL = LenShank + LenThigh


feetId = np.array([0, 0])

torso = rbdl.Body.fromMassComInertia(
                         10.0, 
                         np.array([0., 0., 0.]),
                         np.diag([0.08333, 0.02967, 0.09633])
                         )
hipRol = rbdl.Body.fromMassComInertia(
                        0.5,
                        np.array([0., 0., 0.]),
                        np.diag([0.0001, 0.000067, 0.000067])
                        )
thigh = rbdl.Body.fromMassComInertia(
                        1.,
                        np.array([0., 0., -LenThigh/2]),
                        np.diag([0.0001, 0.000067, 0.000067]) 
                        )
shank = rbdl.Body.fromMassComInertia(
                        1.,
                        np.array([0., 0., -LenShank/2]),
                        np.diag([0.00170036, 0.00170036, 0.00010929])
                        )
ankle = rbdl.Body.fromMassComInertia(
                        0.01,
                        np.array([0., 0., 0.]),
                        np.diag([1e-8, 1e-8, 1e-8])
                        )

jx = rbdl.Joint.fromJointType("JointTypeRevoluteX")
jy = rbdl.Joint.fromJointType("JointTypeRevoluteY")
jfixed = rbdl.Joint.fromJointType("JointTypeFixed")


floatBase = rbdl.Joint.fromJointAxes(
                        np.array([ [0., 0., 0., 1., 0., 0.],
                                   [0., 0., 0., 0., 1., 0.], 
                                   [0., 0., 0., 0., 0., 1.],
                                   [0., 0., 1., 0., 0., 0.],
                                   [0., 1., 0., 0. ,0., 0.],
                                   [1., 0., 0., 0., 0., 0.] ])
                        )


xtrans = rbdl.SpatialTransform()

torsoId = model.AddBody(0, xtrans, floatBase, torso) # 驱干

#  右腿
xtrans.r = np.array([0, -HipY, HipZ]) 

model.AppendBody(xtrans, jx, hipRol) # hip

model.AppendBody(rbdl.SpatialTransform(), jy, thigh) # thigh

xtrans.r = np.array([0, 0, -LenThigh])

model.AppendBody(xtrans, jy, shank) # shank

xtrans.r = np.array([0, 0, -LenShank])

feetId[0] = model.AppendBody(xtrans, jfixed, ankle) # ankle

#  左腿
xtrans.r = np.array([0, HipY, HipZ]) 

model.AddBody(0, xtrans, jx, hipRol) # hip

model.AppendBody(rbdl.SpatialTransform(), jy, thigh) # thigh

xtrans.r = np.array([0, 0, -LenThigh])

model.AppendBody(xtrans, jy, shank) # shank

xtrans.r = np.array([0, 0, -LenShank])

feetId[1] = model.AppendBody(xtrans, jfixed, ankle) # ankle

#  print(feetId)



#####################################
ikset = rbdl.InverseKinematicsConstraintSet()

ikset.AddFullConstraint(
            feetId[0], 
            np.array([0., 0., 0.]), 
            np.array([0., -HipY, -maxL]), 
            np.eye(3)
            )
ikset.AddFullConstraint(
            feetId[1], 
            np.array([0., 0., 0.]), 
            np.array([0., HipY, -maxL]), 
            np.eye(3)
            )
ikset.AddFullConstraint(
            torsoId, 
            np.array([0., 0., 0.]), 
            np.array([0., 0., 0.]), 
            np.eye(3)
            )


q_test = np.array([0, 0, 0, 0, 0, 0,     0., -0.5, 1. , 0, -0.5, 1.])
qd_test = np.zeros(12)
qdd_test =np.zeros(12)

body_point_position = np.array([0., 0., 0.])

rbdl.UpdateKinematics(model, q_test, qd_test, qdd_test)


####################################
ik_res = rbdl.CalcBodyToBaseCoordinates(model, q_test, feetId[0], body_point_position, 0) 

print("CalcBodyToBaseCoordinates: \n", ik_res, "\n", "*"*50)


#######################################
orientation_res = rbdl.CalcBodyWorldOrientation(model, q_test, feetId[0], 0)

print("CalcBodyWorldOrientation: \n", orientation_res, "\n",  "*"*50)



###################################  前三项角速度，后三项式速度
Jcb6D = np.zeros([6, 12])
rbdl.CalcPointJacobian(model, q_test, feetId[0], body_point_position, Jcb6D, 0)

print("CalcPointJacobian6D: \n", Jcb6D, "\n", "*"*50)



###################################  只有三项速度
Jcb3D = np.zeros([3, 12])
rbdl.CalcPointJacobian(model, q_test, feetId[0], body_point_position, Jcb3D, 0)

print("CalcPointJacobian3D: \n", Jcb3D, "\n", "*"*50)



################################ ID 
torque_solution = np.zeros(12) 
rbdl.InverseDynamics(model, q_test, qd_test, qdd_test, torque_solution)
print("ID joint torque solution: \n", torque_solution, "\n", "*"*50)