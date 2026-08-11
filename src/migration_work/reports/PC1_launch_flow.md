# PC1 launch flow

Evidence: `source-derived` plus `runtime-verified-local-five-cycles`

```text
run_autoware
└─ run_autoware_safe.sh
   ├─ exact lock + local process scan + can0..can3 DOWN checks
   └─ ros2 launch autoware_launch autoware.launch.xml
      ├─ global_parameter_loader                         ALWAYS
      ├─ tier4_planning_component                       ON by default
      │  ├─ mission planning
      │  ├─ scenario selection
      │  ├─ behavior path/velocity planning
      │  ├─ motion planning/velocity smoothing
      │  ├─ parking module                                OFF in stationary profile
      │  └─ planning validator
      ├─ tier4_control_component                        OFF / deferred
      │  ├─ trajectory follower
      │  ├─ vehicle command gate
      │  ├─ control validator
      │  └─ operation-mode transition manager
      ├─ tier4_autoware_api_component                   ON by default
      │  ├─ default AD API + localhost web server
      │  └─ RViz initial-pose/routing adaptors           OFF in stationary profile
      └─ RViz                                            OFF by default
```

The active top file does not include vehicle, system, map, sensing, localization, or perception
components; their formerly misleading defaults are now false. `planning_module_preset=default`
selects planning plugins rather than PC ownership. It is unchanged and can require predicted
objects, obstacle pointcloud, occupancy grid, and traffic-signal inputs.

The evaluated stationary tree produced 47 nodes in each of five isolated PC1 cycles with zero
duplicate FQNs and no forbidden control/CAN/parking/RViz-adaptor nodes. The web server listened
on localhost:8888. Required upstream publishers were absent, so this validates composition and
lifecycle rather than planning-data readiness.

The alias still passes `vehicle_model:=sample_vehicle`. Its geometry is not verified for the
current vehicle, so route/trajectory/control validation remains blocked. The current final
planning trajectory is `/planning/scenario_planning/trajectory`, not `/planning/trajectory`.
