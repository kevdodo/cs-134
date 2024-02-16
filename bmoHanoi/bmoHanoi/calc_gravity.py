import sklearn.linear_model as Linearmodel
import numpy as np

actual_position = np.load(r'/home/robot/robotws/src/cs-134/bmoHanoi/bmoHanoi/act_pos.npy')
desired_position = np.load(r'/home/robot/robotws/src/cs-134/bmoHanoi/bmoHanoi/pos_cmds.npy')
effort = np.load(r'/home/robot/robotws/src/cs-134/bmoHanoi/bmoHanoi/act_eff.npy')

SHOULDER_INDEX = 1
ELBOW_INDEX = 2
print(effort.shape)
print(actual_position.shape)
print(desired_position.shape)


actual_position_shoulder = actual_position[:, SHOULDER_INDEX]
actual_position_elbow = actual_position[:, ELBOW_INDEX]

desired_position_shoulder = desired_position[:, SHOULDER_INDEX, 0]
desired_position_elbow = desired_position[:, ELBOW_INDEX, 0]

x_shoulder = np.sin(actual_position_shoulder - actual_position_elbow)

y = desired_position_shoulder - actual_position_shoulder

effort_model = Linearmodel.LinearRegression().fit(x_shoulder.reshape(-1, 1), y.reshape(-1, 1))
print(effort_model.coef_)



x = np.sin(actual_position_shoulder - actual_position_elbow)
y = effort[:, ELBOW_INDEX]


print(x.shape)
print(y.shape)
effort_model = Linearmodel.LinearRegression().fit(x.reshape(-1, 1), y.reshape(-1, 1))
print(effort_model.coef_)


