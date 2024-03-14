import numpy as np
from sklearn.linear_model import LinearRegression

import matplotlib.pyplot as plt

points = np.load(r'/home/robot/robotws/src/cs-134/bruh2.npy', allow_pickle=True)#np.array([np.array([.122, -.548, .107 ]), np.array([.277, -.534, .103]), np.array([.431, -.516, .102]), np.array([.134, -.108, .068]), np.array([-.146, -.092, 0.067]), np.array([-.157, -.199, .073]), np.array([-.186, .293, 0.084]), np.array([-.219, -.405, .090]), np.array([-.264, -.598, .107]), np.array([-.263, -.595, .108]), np.array([-.054, -.567, .104]), np.array([0.267, -.200, .074,]), np.array([.172, -.261, .084]), np.array([.058, -.349, .092]), np.array([-.170, -.220, .080]), np.array([-.076, -.255, .080]), np.array([.065, -.310, .088]), np.array([.376, -.477, .099]), np.array([.375, -.293, .083]), np.array([.342, -.099, .066])])

# print(points.reshape((27,1))[1:])

points = points.reshape((len(points),1))[1:]

X = []
y = []

for i in range(len(points) - 1):
    X.append([points[i][0][0], points[i][0][1]])
    y.append([points[i][0][2] - .105])

X = np.array(X).reshape((len(points) - 1,2))
y = np.array(y).reshape((len(points) - 1,1))

# print(X.shape)

reg = LinearRegression().fit(X, y)
print(reg.score(X, y))
print(reg.coef_)
print(reg.intercept_)
    
# for p in points[0]

# bruh = np.empty((26, 3))
# bruh[:] = np.nan

# for idx, point in enumerate(points[1:]):
#     print(type(points[idx]))
#     if points[idx] is not None :
#         bruh[idx, :] = points[idx].reshape((3))
# print(len(np.flatnonzero(bruh)))
# X = bruh[:, :2] 
# y = bruh[:, 2]



# print(reg.predict([[.431, -0.516]]))

plt.scatter(X[:, 0], y)
plt.show()