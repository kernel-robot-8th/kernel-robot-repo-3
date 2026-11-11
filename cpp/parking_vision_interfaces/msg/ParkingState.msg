# Display-only live parking state, the ROS form of GET /api/parking-spots/{spot_id}/live.
# NOT_IN_VIEW -> MOVING -> IN_POSITION / OFF_POSITION. Never a condition for the arm step: that is
# CheckParking, which judges only frames captured after the request.
# Published on /pinky_parking/state with reliable, keep-last 1, transient_local QoS.

std_msgs/Header header              # stamp and frame_id of the last judged frame; publish time before any frame
string spot_id
int32 marker_id
string state                        # NOT_IN_VIEW, MOVING, IN_POSITION, OFF_POSITION, AMBIGUOUS, VIEW_UNVERIFIED,
                                    # CAMERA_UNAVAILABLE, SPOT_INVALID, TRACKING_ERROR (software fault)
string reason
builtin_interfaces/Time since       # when this state began
builtin_interfaces/Time updated_at
string feed_camera_id               # image topic the node subscribes to
bool has_frame
uint64 frame_sequence               # node-local count of received images

# Offset against the registered spot in its marker axes (JSON offset; null -> has_offset false).
# dx + toward the marker's right edge, dy + toward its top edge, dyaw + counterclockwise seen from the camera.
bool has_offset
float64 dx_mm
float64 dy_mm
float64 distance_mm
float64 dyaw_deg
string marker_forward_axis          # +x, -x, +y, -y; empty when the spot does not record it
bool has_pinky_frame                # forward_mm / left_mm present (only with marker_forward_axis)
float64 forward_mm
float64 left_mm

bool has_ratios                     # position_ratio / yaw_ratio / deviation_level present
float64 position_ratio              # distance_mm / position tolerance
float64 yaw_ratio                   # |dyaw_deg| / yaw tolerance
string deviation_level              # WITHIN, NEAR, FAR

bool has_spot                       # false only for SPOT_INVALID; the fields below need the spot
float64 tolerance_position_mm
float64 tolerance_yaw_deg
bool view_check                     # JSON "ON" / "OFF"
bool has_view_shift
float64 view_shift_px

# image_overlay: raw feed pixels for drawing only.
bool has_view_shift_xy
float64 view_shift_x_px
float64 view_shift_y_px
uint32 image_width
uint32 image_height
float64[8] registered_corners_px    # x, y of 4 corners in ArUco order
bool has_marker_corners             # only from judged frames; cleared when the view changes or Pinky is lost
float64[8] marker_corners_px
float64[] anchor_centers_px         # x, y per fixed marker

bool display_only                   # always true
bool usable_for_robot_control       # always false
