# ---------------------- Laser sensor definitions ----------------------

define laser360 ranger
(
  sensor
  (
    pose     [ 0.00 0.00 0.00 0.0 ]     # Front-mounted, facing forward
    size     [ 0.06 0.06 0.10 ]
    range    [ 0.00 20.0 ]
    fov      360.0
    samples  640                       # 0.5° resolution
  )
  color "black"
  size [ 0.06 0.15 0.03 ]
)
# ---------------------- Omnidirectional Robot -------------------------

define omniBot position
(
  # Initial pose (x, y, z, yaw) — overridden in .world if needed
  pose          [ 0.0 0.0 0.0 0.0 ]

  localization  "odom"

  odom_error    [ 0.2 0.2 0.0 0.3 ]

  # Robot geometry and appearance
  size          [ 0.54 0.40 0.40 ]       # Footprint and height
  origin        [ 0.0 0.0 0.0 0.0 ]      # Centered origin
  gui_nose      1                        # Display heading in GUI
  drive         "omni"
  color         "red"  

  # Laser sensor
  laser360()
)
