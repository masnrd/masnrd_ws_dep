import rclpy
from math import degrees, sqrt, atan2, asin, acos, cos, sin, radians
from rclpy.node import Node
from dataclasses import dataclass
from masnrd_msgs.msg import Detected, GotoWaypoint, ReachedWaypoint

EARTH_RADIUS = 6378.137 * 1000 # Earth's radius in metres

@dataclass
class MapPosition:
    lat: float
    lon: float

    def isClose(self, other: 'MapPosition', tolerance=0.0001):
        if abs(self.lat - other.lat) > tolerance:
            return False
        if abs(self.lon - other.lon) > tolerance:
            return False
        return True

    def toPositionXY(self, refPt: 'MapPosition') -> 'PositionXY':
        """ Converts from WGS84 coordinates (lat, lon) to a position vector relative to a reference point (x, y, refPt). """
        # 1. find the bearing: http://www.movable-type.co.uk/scripts/latlong.html
        lat1 = radians(refPt.lat)
        lat2 = radians(self.lat)
        dLat = radians(self.lat - refPt.lat)
        dLon = radians(self.lon - refPt.lon)
        y_temp = sin(dLon) * cos(lat2)
        x_temp = (cos(lat1) * sin(lat2)) - (sin(lat1)*cos(lat2)*cos(dLon))
        bearing = atan2(y_temp, x_temp) # in radians

        # 2. find the distance (haversine)
        h_lat = (1 - cos(dLat))/2
        h_lon = (1 - cos(dLon))/2
        d = asin(sqrt(h_lat + (cos(lat1)*cos(lat2)*h_lon))) * (2 * EARTH_RADIUS)

        # 3. calc vector
        new_x = d*cos(bearing)
        new_y = d*sin(bearing)
        return PositionXY(new_x, new_y, refPt)

@dataclass
class PositionXY:
    x: float
    y: float
    refPt: MapPosition
    
    def toMapPosition(self) -> MapPosition:
        """ Converts from a position vector relative to a reference point (x, y, refPt) to WGS84 coordinates (lat, lon) """
        # Code adapted from https://stackoverflow.com/questions/7222382/get-lat-long-given-current-point-distance-and-bearing
        lat = radians(self.refPt.lat)
        lon = radians(self.refPt.lon)
        d = sqrt(self.x**2 + self.y**2)
        bearing = radians(atan2(self.y, self.x))
        R = EARTH_RADIUS

        new_lat = asin(sin(lat) * cos(d/R) + cos(lat) * sin(d/R) * cos(bearing))
        new_lon = lon + atan2(
            sin(bearing) * sin(d/R) * cos(lat),
            cos(d/R) - sin(lat) * sin(new_lat)
        )

        return MapPosition(degrees(new_lat), degrees(new_lon))

START_POSITION = MapPosition(1.40728, 104.02880)

hardcoded_points = [
    MapPosition(1.40736, 104.02866),
    MapPosition(1.40714, 104.02861),
    MapPosition(1.40701, 104.02895),
]

def GetNewTarget(pos: MapPosition) -> MapPosition:
    """
    Returns a target position from the pathfinding algorithm,
    having reached `pos`.
    - Called by the PathfinderNode upon the drone reaching its target.
    """
    
    for i in range(len(hardcoded_points)):
        if hardcoded_points[i].isClose(pos):
            new_i = (i + 1) % len(hardcoded_points)
            print(f"Reached: {hardcoded_points[i]}, going to: {hardcoded_points[new_i]}")
            return hardcoded_points[new_i]

    print(f"Unknown end position {pos}, heading back to start position {START_POSITION}")
    return START_POSITION # otherwise, return to start position in sector

def FoundSignals(pos: MapPosition, signal_count: int):
    """
    Updates the pathfinding algorithm about new signals
    """
    pass

class PathfinderNode(Node):
    """
    ROS2 Pathfinder Node. Publishes and subscribes to the necessary topics
    """
    #TODO: need to maintain previous and next waypoint because the gps is inacc
    def __init__(self):
        super().__init__("pathfinder")
        self.origin = MapPosition(0.0, 0.0)
        self.reachedwp_sub = self.create_subscription(
            ReachedWaypoint,
            "/pathfinder/in/reached_waypoint",
            self.reached_waypoint_callback,
            10,
        )
        self.detected_sub = self.create_subscription(
            Detected,
            "/pathfinder/in/detected",
            self.detected_callback,
            10,
        )
        self.gotowp_pub = self.create_publisher(
            GotoWaypoint,
            "/pathfinder/out/goto_waypoint",
            10
        )

        # Give first target
        initialPoint = MapPosition(1.40724, 104.02896)
        self.origin = initialPoint
        new_target = START_POSITION.toPositionXY(initialPoint)

        # Handover new target (as x, y relative to refPt) to central interface
        gotowp_msg = GotoWaypoint()
        gotowp_msg.timestamp = self.get_clock().now().nanoseconds // 1000
        gotowp_msg.x = new_target.x
        gotowp_msg.y = new_target.y
        gotowp_msg.ref_lat = new_target.refPt.lat
        gotowp_msg.ref_lon = new_target.refPt.lon
        self.gotowp_pub.publish(gotowp_msg)
        self.get_logger().info(f"Going to: ({gotowp_msg.x}, {gotowp_msg.y})")
    
    def reached_waypoint_callback(self, msg: ReachedWaypoint):
        refPt = MapPosition(msg.ref_lat, msg.ref_lon)
        #self.get_logger().info(f"Reached ({msg.x}, {msg.y}) (taking ref as {refPt})")

        # Get new target
        reached_pt = PositionXY(msg.x, msg.y, refPt)
        print("reached_pt =", reached_pt)
        print("reached_pt as MapPos: ", reached_pt.toMapPosition())
        new_target = GetNewTarget(reached_pt.toMapPosition()).toPositionXY(refPt)
        print("new_target =", new_target)
        print("new_target as MapPos: ", new_target.toMapPosition())
        # Handover new target (as x, y relative to refPt) to central interface
        gotowp_msg = GotoWaypoint()
        gotowp_msg.timestamp = self.get_clock().now().nanoseconds // 1000
        gotowp_msg.x = new_target.x
        gotowp_msg.y = new_target.y
        gotowp_msg.ref_lat = new_target.refPt.lat
        gotowp_msg.ref_lon = new_target.refPt.lon
        self.gotowp_pub.publish(gotowp_msg)
        #self.get_logger().info(f"Going to: ({gotowp_msg.x}, {gotowp_msg.y}) (taking ref as {new_target.refPt})")

    def detected_callback(self, msg: Detected):
        pass


def main(args=None):
    rclpy.init(args=args)
    pathfinder_node = PathfinderNode()

    rclpy.spin(pathfinder_node)

    # Teardown
    pathfinder_node.destroy_node()

if __name__ == "__main__":
    main()