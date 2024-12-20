# Raw Velodyne position packet.

Header     header       # Stamp marks the start time of data collection.
uint8[512] data         # Velodyne position packet data blocks + timestamp
