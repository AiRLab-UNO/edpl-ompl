STL phi.eval_smooth_rob(smooth_tau_, smooth_type_) will provide global heuristic/stablization instead of FIRM

❯ debug and update stl-firmcp-demo with this setup file examples/configs/firmcp/SetupFIRMCP-Beacon.yaml. Our goal
  is to achieve similar result as FIRMCP with STL specification and corresponding robustness value. read the yaml
  file and src/Planner/STLFIRMCP.cpp implementation. Fix the run time bug as well.