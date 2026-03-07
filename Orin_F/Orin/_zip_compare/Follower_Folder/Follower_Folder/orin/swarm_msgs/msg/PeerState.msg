# Single peer drone state — matches UDP wire format
uint32 drone_id
float64 latitude       # degrees
float64 longitude      # degrees
float64 altitude       # meters (relative to home)
float64 vn             # North velocity m/s
float64 ve             # East velocity m/s
float64 vd             # Down velocity m/s
float64 heading        # degrees
float64 stamp          # Unix timestamp from sender
uint8 guidance_mode    # 0=NONE, 1=TRACKING, 2=CATCHUP, 3=EVASION, 4=FAILSAFE

# FC state (from peer HEARTBEAT over RFD900x radio)
uint8 fc_mode_code     # ArduPilot mode: 0=STABILIZE 4=GUIDED 5=LOITER 6=RTL etc.
bool fc_armed          # Peer FC armed?
bool fc_connected      # Peer FC connected to MAVROS2?

# Battery state (from peer /mavros/battery relayed over RFD900x as NAMED_VALUE_FLOAT)
float32 battery_voltage     # Volts
float32 battery_current     # Amps (negative = discharging on some FCs)
float32 battery_remaining   # 0.0–1.0 fraction (-1.0 = unknown)
