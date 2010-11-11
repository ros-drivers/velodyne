# Velodyne LIDAR scan packets.

uint32 PACKETS_PER_REVOLUTION = 260

Header           header         # standard ROS message header
VelodynePacket[] packets        # vector of raw packets
