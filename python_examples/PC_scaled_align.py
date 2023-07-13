#
# add path to local library mrob on bashr_rc: "export PYTHONPATH=${PYTHONPATH}:${HOME}/mrob/mrob/lib"
import pybind as mrob
import numpy as np
import open3d

# generate random data
N = 10
X =  np.random.rand(N,3)
theta = np.random.randn(3)
t = np.array([0,0,0])
T = mrob.geometry.SE3(mrob.geometry.SO3(theta),t)
Y = T.transform_array(X)
scale = 1000
t = np.random.randn(3)
Y = scale*Y + t + np.random.rand(N,3)*0.01

S = T.T()
S[:3,:3] *= scale
S[:3,3] = t

print('X = \n', X,'\n S = \n', S,'\n Y =\n', Y)




def pcd_1(X, color, T = np.identity(4)):
    pcd = open3d.geometry.PointCloud()
    pcd.points = open3d.utility.Vector3dVector(X)
    pcd.transform(T)
    pcd.paint_uniform_color(color)
    return pcd

def vis_her(X, Y, T = np.identity(4)):
    blue = np.array([0,0,1], dtype='float64')
    red = np.array([1,0,0], dtype='float64')
    open3d.visualization.draw_geometries([pcd_1(X,red,T), pcd_1(Y,blue)])

def vis_arr(X):
	pcd = open3d.PointCloud()
	pcd.points =open3d.utility.Vector3dVector(X)
	pcd.paint_uniform_color(np.random.rand(3,).astype(np.float64))
	open3d.visualization.draw_geometries([pcd])



# solve the problem
vis_her(X,Y)
S_sol = mrob.registration.scaled_arun(X,Y)
print('Arun solution =\n', S_sol)
print('Initial transformation =\n',S)
vis_her(X,Y,S_sol)

