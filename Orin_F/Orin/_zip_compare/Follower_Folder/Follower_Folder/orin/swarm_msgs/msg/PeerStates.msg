# Array of all non-stale peer states
builtin_interfaces/Time header_stamp
PeerState[] peers

# Peer Jetson liveness (from HEARTBEAT over RFD900x radio)
bool peer_jetson_alive        # true if HEARTBEAT received within timeout
float64 peer_last_heartbeat   # Unix timestamp of last peer HEARTBEAT

# Own battery state (from local /mavros/battery — SYS_STATUS ID1)
float32 own_battery_voltage     # Volts
float32 own_battery_current     # Amps
float32 own_battery_remaining   # 0.0–1.0 fraction (-1.0 = unknown)

# RFD900x radio link quality (from RADIO_STATUS ID109 injected by modem)
uint8 radio_rssi          # Local received signal strength (0-255)
uint8 radio_remrssi       # Remote received signal strength (0-255)
uint8 radio_txbuf         # Transmit buffer free space % (0-100)
uint8 radio_noise         # Local background noise (0-255)
uint8 radio_remnoise      # Remote background noise (0-255)
uint16 radio_rxerrors     # Receive error count
bool radio_link_alive     # true if RADIO_STATUS received within timeout
float64 radio_last_status # Unix timestamp of last RADIO_STATUS
