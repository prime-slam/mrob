#
# add path to local library mrob on bash_aliases:
# "export PYTHONPATH=${PYTHONPATH}:${HOME}/mrob/mrob/build/mrobpy"
import mrob
import numpy as np


def point_plane_residual(point, plane_normal, plane_offset):
    return float(np.dot(plane_normal, point) + plane_offset)


graph = mrob.FGraph()

# One anchored pose is enough to define the EF plane from local points.
pose_id = graph.add_node_pose_3d(mrob.geometry.SE3(), mrob.NODE_ANCHOR)
plane_ef_id = graph.add_eigen_factor_plane()

# Synthetic plane points lying on z = 0 in the world frame.
plane_points = np.array(
    [
        [-1.0, -1.0, 0.0],
        [-1.0, 1.0, 0.0],
        [1.0, -1.0, 0.0],
        [1.0, 1.0, 0.0],
        [0.0, 0.0, 0.0],
        [0.4, -0.3, 0.0],
    ],
    dtype=np.float64,
)
graph.eigen_factor_plane_add_points_array(
    planeEigenId=plane_ef_id,
    nodePoseId=pose_id,
    pointsArray=plane_points,
    W=1.0,
)

# One landmark point starts away from the plane.
point_initial = np.array([0.25, -0.35, 0.8], dtype=np.float64)
landmark_id = graph.add_node_landmark_3d(point_initial, mrob.NODE_STANDARD)
graph.add_factor_1landmark_point2plane_4d(
    nodeLandmarkId=landmark_id,
    planeEigenId=plane_ef_id,
    obsInf=np.array([1.0], dtype=np.float64),
)

print("pose id =", pose_id, ", EF plane id =", plane_ef_id, ", landmark id =", landmark_id)
print("initial landmark =", point_initial)
print("initial |point-to-plane distance| =", abs(point_plane_residual(point_initial, np.array([0.0, 0.0, 1.0]), 0.0)))
print("initial chi2 =", graph.chi2(True))

graph.solve(mrob.LM, 20, verbose=True)

estimated_state = graph.get_estimated_state()
point_refined = np.asarray(estimated_state[landmark_id], dtype=np.float64).reshape(3)

print("refined landmark =", point_refined)
print("refined |point-to-plane distance| =", abs(point_plane_residual(point_refined, np.array([0.0, 0.0, 1.0]), 0.0)))
print("final chi2 =", graph.chi2())
graph.print(True)
