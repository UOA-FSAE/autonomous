# Fused cones for one LiDAR frame, grouped by colour like Detections.msg.
std_msgs/Header header          # stamp = LiDAR cloud stamp, frame_id = base_link
FusedCone[] yellow
FusedCone[] blue
FusedCone[] small_orange
FusedCone[] big_orange
FusedCone[] unknown             # populated only by the orphan-LiDAR pass (stub for now)
