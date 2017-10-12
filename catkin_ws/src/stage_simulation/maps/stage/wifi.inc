define wifiscanner_sensor sensor (
  size [ 0.025 0.025 0.025 ]
  range [ 0 10.0 ]

  # needed for stage to close the circle
  fov 362

  color_rgba [ 1 0 0 0.5 ]
  samples 180
)

define wifiscanner wifiranger (
  size [ 0.2 0.2 0.2 ]
  wifiscanner_sensor (
    pose [ 0 0 0 0 ]
  )
)
