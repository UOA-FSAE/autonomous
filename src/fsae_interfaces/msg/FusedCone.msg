# A single fused cone: colour comes from the camera, position is refined by the LiDAR.

# source values
uint8 SOURCE_CAMERA_LIDAR_FUSED = 0   # position refined against the LiDAR sphere
uint8 SOURCE_CAMERA_ONLY        = 1   # LiDAR could not refine (e.g. beyond LiDAR range)
uint8 SOURCE_LIDAR_ONLY         = 2   # orphan LiDAR cluster, no camera seed (stub for now)

# colour values (matches Cone.msg)
uint8 COLOUR_BLUE   = 0
uint8 COLOUR_ORANGE = 1
uint8 COLOUR_YELLOW = 2
uint8 COLOUR_OTHER  = 3

geometry_msgs/Point position    # cone position in the base_link frame
uint8 colour                    # see COLOUR_* constants above
float32 confidence              # 0.0-1.0, from the LiDAR refinement
uint8 source                    # see SOURCE_* constants above

# diagnostics (0 when source is CAMERA_ONLY)
uint16 point_count              # LiDAR points in the verified cluster
uint8 ring_count                # distinct LiDAR rings the cluster spans
float32 mean_intensity          # mean LiDAR intensity of the cluster
