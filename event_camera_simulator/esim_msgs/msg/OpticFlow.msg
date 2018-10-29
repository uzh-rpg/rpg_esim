# This message contains an optic flow map
# (0, 0) is at top-left corner of image
# The optic flow x and y components are expressed in px/s and encoded with floating point accuracy.
# The memory layout of the data is row-major.

Header header
uint32 height
uint32 width

float32[] flow_x
float32[] flow_y
