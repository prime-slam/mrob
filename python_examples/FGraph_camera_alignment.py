import mrob
import numpy as np


H, W = 640, 320
K = np.array([
    [500, 0, 320],
    [0, 500, 160],
    [0, 0, 1]
])

camera_k = np.array([ K[0, 0],
                      K[1, 1],
                      K[0, 2],
                      K[1, 2] ])

def unproject(p, d, K):
    fx = K[0, 0]
    fy = K[1, 1]
    cx = K[0, 2]
    cy = K[1, 2]
    v, u = p[0], p[1]
    z = d
    x = (v - cx) / fx * z
    y = (u - cy) / fy * z
    return np.array([x, y, z])

def project(P, K):
    p = K @ np.array(P).reshape((3, 1)) / P[2]
    return p[0, 0], p[1, 0]


def transform_point(p, T):
    new_p_hom = T @ np.append(p, 1).reshape(4, 1)
    # print(p)
    # print(new_p_hom[:3].reshape(3))
    return new_p_hom[:3].reshape(3)

def is_point_in_image(p,H,W):
    result = True
    if p[0] > W or p[0] < 0:
        result = False
    if p[1] > H or p[1] < 0:
        result = False
    return result    


# Create random points
N = 10
p1 = np.random.rand(N,3)
p1[:,0] *= W
p1[:,1] *= H
p1[:,2] *= 10.0
#print(p1)

# Lift points to 3D
X = np.zeros((N,3))
for i in range(N):
    X[i,:] = unproject(p1[i,:2],p1[i,2],K)
#print(X)

# Transform point no the new reference frame
T = mrob.geometry.SE3(np.random.randn(6)*0.1)
T.print()
X_target = T.transform_array(X)
#print(X_target)

# project points into new observations
p2 = np.zeros((N,2))
for i in range(N):
    p2[i,0], p2[i,1] = project(X_target[i,:], K)


# ----------------------------------------------------
# Point3d
# Solve the problem
graph = mrob.FGraph()
n1 = graph.add_node_pose_3d(mrob.geometry.SE3())
W = np.identity(2)
for i in range(N):
    # create a landmark point
    l = graph.add_node_landmark_3d(X[i,:], mrob.NODE_ANCHOR)
    graph.add_factor_camera_proj_3d_point(
            obs = p2[i,:],
            nodePoseId = n1,
            nodeLandmarkId = l,
            camera_k = camera_k,
            obsInvCov = W)
graph.print()

# Solve FGraph
print('3d points: current error = ', graph.chi2())
graph.solve(mrob.LM)
T_estim = mrob.geometry.SE3(graph.get_estimated_state()[0]).inv()
print('distance rotation = ', T.distance_rotation(T_estim))
print('distance translation = ', T.distance_trans(T_estim))


# ----------------------------------------------------
# Lines 3d
# Solve the problem
graph = mrob.FGraph()
n1 = graph.add_node_pose_3d(mrob.geometry.SE3())
W = np.identity(2)
for i in range(0,N,2):
    # create a landmark points x2
    l1 = graph.add_node_landmark_3d(X[i,:], mrob.NODE_ANCHOR)
    l2 = graph.add_node_landmark_3d(X[i+1,:], mrob.NODE_ANCHOR)
    graph.add_factor_camera_proj_3d_line(
            obsPoint1 = p2[i,:],
            obsPoint2 = p2[i+1,:],
            nodePoseId = n1,
            nodePoint1 = l1,
            nodePoint2 = l2,
            camera_k = camera_k,
            obsInvCov = W)
graph.print()

# Solve FGraph
print('3d Lines: current error = ', graph.chi2())
graph.solve(mrob.LM)
T_estim_line = mrob.geometry.SE3(graph.get_estimated_state()[0]).inv()
print('distance rotation = ', T.distance_rotation(T_estim_line))
print('distance translation = ', T.distance_trans(T_estim_line))


