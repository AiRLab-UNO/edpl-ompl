signal x1, y1, ox

# d_safe : robot must keep at least this distance to any obstacle
# d_target: position tolerance for declaring a target reached
# T      : time horizon for the temporal operators
param d_safe = 0.4, d_target = 0.4, T = 60

# Robot start pose
param rx = -2.5, ry = -2.5, rtheta = 0.7854

# Targets the robot must visit (in order)
param gx1 =  1.5, gy1 =  1.5

# Spec-declared obstacles. STLRobustness populates the ox channel with the
# distance to the nearest of these for each sampled state.
param ox1_x = -1.0, ox1_y = -1.0
param ox2_x =  1.0, ox2_y =  0.5
param ox3_x =  0.0, ox3_y = -2.0

# An obstacle is "critical" when the robot is within d_safe of it.
obs_critical := ox[t] < d_safe

# Reach predicate for the lone target.
target_r1 := abs(x1[t] - gx1) < d_target and abs(y1[t] - gy1) < d_target
phi_reach1 := ev_[0, T] target_r1

phi_reach := phi_reach1
phi_safe  := alw_[0, T] not obs_critical
phi       := phi_reach and phi_safe
