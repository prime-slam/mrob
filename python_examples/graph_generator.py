import mrob
import numpy as np
import matplotlib.pyplot as plt

def generate_random_graph(nodes: int = 5, factors : int = 10) -> mrob.FGraphDiff:

    assert factors >= nodes*2
    graph = mrob.FGraphDiff()

    x = np.random.randn(3)*1e-1
    n = graph.add_node_pose_2d(x, mrob.NODE_ANCHOR)

    addional_factor_counter = 0

    max_additional_factors = factors - nodes*2

    indexes = [n]
    for i in range(1, nodes):
        x = np.array([i,0,0]) + np.random.randn(3)*1e-1
        n = graph.add_node_pose_2d(x)

        invCov = np.identity(3)
        graph.add_factor_1pose_2d_diff(np.array([i,0,0] + np.random.randn(3)*1e-1),n,1e6*invCov)
        graph.add_factor_2poses_2d_diff(np.array([1,0,0]),indexes[-1],n,invCov)
        if addional_factor_counter < max_additional_factors:
            if np.random.random() > 0.5:
                graph.add_factor_1pose_2d_diff(np.array([i,0,0] + np.random.randn(3)*1e-1), n, 1e6*invCov)
                addional_factor_counter += 1

        indexes.append(n)

    node_index = 0
    while addional_factor_counter < max_additional_factors:
        if np.random.random() > 0.5:
            graph.add_factor_1pose_2d_diff(np.array([node_index,0,0] + np.random.randn(3)*1e-1), node_index, 1e6*invCov)
            addional_factor_counter += 1

        else:
            node_index += 1

            node_index  = node_index % nodes

    print('Current chi2 = ', graph.chi2() ) # re-evaluates the error, on print it is only the error on evalation before update
    
    return graph

if __name__ == "__main__":
    graph = generate_random_graph(10,20)

    print(graph)

    graph.solve()
