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
T1 = SE3vel(R, 0*np.random.randn(3), np.random.randn(3))
T1.print()


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
# 4 Testing Operations
# -------------------------------------------------------------------------
print('testing inverse', T1*T1.inv())



# -------------------------------------------------------------------------
# 3 Testing Adjoint
# -------------------------------------------------------------------------
#print(T.adj())
# create a T exp(xi) = Exp( adj_T xi ) T
xi = np.random.randn(9)
print(T * SE3vel(xi))
print(SE3vel(T.adj() @ xi) * T)


