# Raw Velodyne LIDAR input.

uint32 PACKET_SIZE = 1206
uint32 PACKETS_PER_REVOLUTION = 260

Header  header          # standard ROS message header
uint8[] data            # data array

