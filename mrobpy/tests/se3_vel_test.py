from mrob import SE3vel, SO3
import numpy as np

# TODO, add proper test, this is just initial try

# -------------------------------------------------------------------------
# 1 Testing contructors and print
# -------------------------------------------------------------------------
T = SE3vel()
print(T)


theta = np.random.randn(3)
R = SO3(theta)
print(R)
T = SE3vel(R, np.random.randn(3), np.random.randn(3))
T.print()


xi = np.random.randn(9)
print('xi initializator:\n', xi)
T = SE3vel(xi)
T.print()


# -------------------------------------------------------------------------
# 2 Testing logarithm exponent
# -------------------------------------------------------------------------
xi_reprojected = T.Ln()
print('xi reprojected:\n', xi_reprojected)
print('matrix proof:\n', SE3vel(xi_reprojected))



# -------------------------------------------------------------------------
# 2 Testing extracting data from T
# -------------------------------------------------------------------------
print('full matrix: \n', T.T())
print('rotation: \n', T.R())
print('translation: \n', T.t())
print('velocity: \n', T.v())
print('full matrix compact: \n', T.T_compact())





# -------------------------------------------------------------------------
# 3 Testing Adjoint
# -------------------------------------------------------------------------
print(T.adj())






# -------------------------------------------------------------------------
# 4 Testing Operations
# -------------------------------------------------------------------------






