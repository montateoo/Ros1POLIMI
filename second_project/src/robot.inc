# ---------------------- Laser sensor definitions ----------------------

define laser_front ranger
(
  sensor
  (
    pose     [ 0.25 0.00 0.15 0.0 ] # front-mounted, facing frontward
    size     [ 0.06 0.06 0.10 ]
    range    [ 0.00 20.0 ]
    fov      270.0
    samples  540                       # 0.5° resolution
  )
  color "black"
)

define laser_rear ranger
(
  sensor
  (
    pose     [ -0.25 0.00 0.15 180 ] # Rear-mounted, facing backward
    size     [ 0.06 0.06 0.10 ]
    range    [ 0.00 20.0 ]
    fov      270.0
    samples  540
  )
  color "black"
)

# ---------------------- Omnidirectional Robot -------------------------

define omni_bot position
(
  pose          [ 0.0 0.0 0.0 0.0 ]

  # Robot geometry and appearance
  size          [ 0.54 0.40 0.40 ]       # Footprint and height
  origin        [ 0.0 0.0 0.0 0.0 ]      # Centered origin
  color         "grey"
  gui_nose      1                        # Display heading in GUI

  # Motion model
  drive         "omni"                   # Holonomic movement

  # Odometry (simulated noise for realism)
  localization  "odom"
  odom_error    [ 0.2 0.2 0.0 0.3 ]

  # Laser sensors
  laser_front()
  laser_rear()
)
