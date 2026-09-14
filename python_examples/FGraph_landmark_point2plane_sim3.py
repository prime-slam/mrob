#
# Sim3 landmark-to-local-plane factor.
# Residual r = n' * T^{-1} X + d, with T in Sim3 (camera-to-world).
import mrob
import numpy as np


def point_plane_residual(point, plane):
    plane = np.asarray(plane, dtype=np.float64).reshape(4)
    n = plane[:3]
    n = n / np.linalg.norm(n)
    return float(np.dot(n, point) + plane[3])


graph = mrob.FGraph()

pose_id = graph.add_node_sim3(mrob.geometry.Sim3(np.eye(4)), mrob.NODE_ANCHOR)
point_initial = np.array([0.25, -0.35, 2.0], dtype=np.float64)
landmark_id = graph.add_node_landmark_3d(point_initial, mrob.NODE_STANDARD)

# Camera-frame plane z = 1.
obs_plane_local = np.array([0.0, 0.0, 1.0, -1.0], dtype=np.float64)
graph.add_factor_1pose_1landmark_point2plane_sim3(
    obsPlaneLocal=obs_plane_local,
    nodePoseId=pose_id,
    nodeLandmarkId=landmark_id,
    obsInf=np.array([1.0], dtype=np.float64),
)

print("pose id =", pose_id, ", landmark id =", landmark_id)
print("initial landmark =", point_initial)
print("initial |point-to-plane| =", abs(point_plane_residual(point_initial, obs_plane_local)))
print("initial chi2 =", graph.chi2(True))

graph.solve(mrob.LM, 20, verbose=True)

estimated_state = graph.get_estimated_state()
point_refined = np.asarray(estimated_state[landmark_id], dtype=np.float64).reshape(3)
print("refined landmark =", point_refined)
print("refined |point-to-plane| =", abs(point_plane_residual(point_refined, obs_plane_local)))
print("final chi2 =", graph.chi2())
graph.print(True)
