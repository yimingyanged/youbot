# Joint group nale
string group_name
# IK type: IKPositionOrientation6D, IKPosition3D
string ik_type
# If true, null-space resolution will be used to achieve secondary joint angle pose (the q_comfortable pose)
bool use_nullspace
# If true, "weights" will be used
bool use_weighting
# Joint weighting of the least squares solution
# Values should be from interval (0,1] (smaller = less influence). The vector must have the same size and the number of joints in the group.
string weights
# Comfortable pose used for null-space resolution
string q_comfortable
# Regularisation term (smaller = less regularisation)
float64 lambda 
# Convergence threshold (smaller= higher accuracy)
float64 epsilon