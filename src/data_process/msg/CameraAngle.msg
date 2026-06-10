std_msgs/Header header
uint8 count            # number of detected objects
float32[] alpha        # horizontal bearing angle (radians), atan2(y, x)
float32[] theta        # elevation angle (radians), asin(z / mag)
