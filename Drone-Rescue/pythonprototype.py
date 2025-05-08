from numpy import cos, sin, arctan2, sqrt
import math

# Radius of the earth
r = 6371.2

# Start point
latitude = 51.5007
longitude = 0.1246

# End point
latitude2 = 40.6892
longitude2 = 74.0445

 # Convert angles to radians
latituder = latitude * (math.pi / 180)
longituder = longitude * (math.pi / 180)
latitude2r = latitude2 * (math.pi / 180)
longitude2r = longitude2 * (math.pi / 180)

def haversine_distance(lon1,lat1,lon2,lat2,rad):

    # Get delta latitude and delta longitude
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    
    # Apply haversine formula
    a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
    c = 2 * arctan2(sqrt(a), sqrt(1 - a))
    distance = rad * c * 1000
    
    return distance


print("Distance (m) =",haversine_distance(longituder,latituder,longitude2r,latitude2r,r))