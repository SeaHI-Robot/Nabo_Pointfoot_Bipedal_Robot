import mujoco

model = mujoco.MjModel.from_xml_path("./test.xml")
data = mujoco.MjData(model)
names = model.names

mujoco.mj_step(model, data)


print("Name: ", names)
print("-"*30)

print('body inertia: \n', model.body_inertia) # 这个是相对于质心的惯量,在geom设置的旋转无效的,具体惯量的xyz方向排布需要人工辨别

print('center of mass: \n', model.body_ipos)
