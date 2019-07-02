# Raw Rfans LIDAR packet.
#Header        header         # standard ROS message header
time stamp              # packet timestamp
uint32 udpCount          # packet number
uint32 udpSize
uint8[] data      # packet contents 600 Block
