std_msgs/Header header
uint8 rf_id               # 1 = RF1 or single RF, 2 = RF2
uint16 time_offset_ms     # firmware timestamp offset (ms since start)
uint16[] samples          # raw ADC samples (4000 per packet)
