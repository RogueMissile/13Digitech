from numpy import cos, sin, arctan2, sqrt
from math import pi


# Radius of the earth 
r = 6371.2

# Start point
latitude = 39.0925
longitude = 174.1120

# End point
latitude2 = 39.0930
longitude2 = 174.1120

# Convert angles to radians
latituder = latitude * (pi / 180)
longituder = longitude * (pi / 180)
latitude2r = latitude2 * (pi / 180)
longitude2r = longitude2 * (pi / 180)

def total_distance(lon1,lat1,lon2,lat2,r):

    # Get delta latitude and delta longitude
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    
    # Apply haversine formula
    a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
    c = 2 * arctan2(sqrt(a), sqrt(1 - a))
    distance = r * c * 1000  # Convert to meters
    
    return distance

def ns_ew_distance(lon1,lat1,lon2,lat2,r):

    # For North-South distance is just difference in latitude
    ns = r * (lat2 - lat1) * 1000  # Convert to meters

    # For East-West distance depends on the difference in longitude but must be scaled by 
    # the cosine of latitude as the distance between longitude lines decreases as you move 
    # toward the poles
    ew = r * (lon2 - lon1) * cos((lat1 + lat2) / 2) * 1000 # Convert to meters

    return ns,ew

print("Total Distance (m) =",total_distance(longituder,latituder,longitude2r,latitude2r,r))
print("NS Component =",ns_ew_distance(longituder,latituder,longitude2r,latitude2r,r)[0])
print("EW Component =",ns_ew_distance(longituder,latituder,longitude2r,latitude2r,r)[1])