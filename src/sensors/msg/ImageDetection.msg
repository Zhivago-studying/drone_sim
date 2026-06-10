std_msgs/Header header
uint8 count          # number of detected objects
float32[] x          # normalized bbox center x [0-1]
float32[] y          # normalized bbox center y [0-1]
float32[] width      # normalized bbox width [0-1]
float32[] height     # normalized bbox height [0-1]
float32[] confidence # detection confidence score (0-1)
