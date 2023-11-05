# Message from pathfinder to central node to GOTO new waypoint
# Values are in METRES.
uint64 timestamp # time since system start (microseconds)

# Position of drone (metres) relative to reference point
float64 x
float64 y

# Reference point (degrees) in global (GPS / WGS84)
float64 ref_lat
float64 ref_lon
