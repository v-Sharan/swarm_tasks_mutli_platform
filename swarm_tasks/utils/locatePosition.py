# Import Necessary Packages
import math
from scipy import interpolate


def destination_location(homeLattitude, homeLongitude, distance, bearing):
    R = 6371e3 #Radius of earth in metres
    rlat1 = homeLattitude * (math.pi/180) 
    rlon1 = homeLongitude * (math.pi/180)
    d = distance
    bearing = bearing * (math.pi/180) #Converting bearing to radians
    rlat2 = math.asin((math.sin(rlat1) * math.cos(d/R)) + (math.cos(rlat1) * math.sin(d/R) * math.cos(bearing)))
    rlon2 = rlon1 + math.atan2((math.sin(bearing) * math.sin(d/R) * math.cos(rlat1)) , (math.cos(d/R) - (math.sin(rlat1) * math.sin(rlat2))))
    rlat2 = rlat2 * (180/math.pi) #Converting to degrees
    rlon2 = rlon2 * (180/math.pi) #converting to degrees
    location = [rlat2, rlon2]
    return location

def distance_bearing(homeLattitude, homeLongitude, destinationLattitude, destinationLongitude):
    R = 6371e3 #Radius of earth in metres
    rlat1 = homeLattitude * (math.pi/180)
    rlat2 = destinationLattitude * (math.pi/180) 
    rlon1 = homeLongitude * (math.pi/180) 
    rlon2 = destinationLongitude * (math.pi/180) 
    dlat = (destinationLattitude - homeLattitude) * (math.pi/180)
    dlon = (destinationLongitude - homeLongitude) * (math.pi/180)
    #haversine formula to find distance
    a = (math.sin(dlat/2) * math.sin(dlat/2)) + (math.cos(rlat1) * math.cos(rlat2) * (math.sin(dlon/2) * math.sin(dlon/2)))
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    distance = R * c #distance in metres
    #formula for bearing
    y = math.sin(rlon2 - rlon1) * math.cos(rlat2)
    x = math.cos(rlat1) * math.sin(rlat2) - math.sin(rlat1) * math.cos(rlat2) * math.cos(rlon2 - rlon1)
    bearing = math.atan2(y, x) #bearing in radians
    bearingDegrees = bearing * (180/math.pi)
    out = [distance, bearingDegrees]
    return out

def geoToCart(origin, endDistance, geoLocation):
    # The initial point of rectangle in (x,y) is (0,0) so considering the current
    # location as origin and retreiving the latitude and longitude from the GPS
    # origin = (12.948048, 80.139742) Format

    # Calculating the hypot end point for interpolating the latitudes and longitudes 
    rEndDistance = math.sqrt(2*(endDistance**2))

    # The bearing for the hypot angle is 45 degrees considering coverage area as square
    bearing = 45

    # Determining the Latitude and Longitude of Middle point of the sqaure area
    # and hypot end point of square area for interpolating latitude and longitude
    lEnd, rEnd = destination_location(origin[0], origin[1], rEndDistance, 180+bearing), destination_location(origin[0], origin[1], rEndDistance, bearing)

    # Array of (x,y)
    x_cart, y_cart  = [-endDistance, 0, endDistance], [-endDistance, 0, endDistance]

    # Array of (latitude, longitude)
    x_lon, y_lat = [lEnd[1], origin[1], rEnd[1]], [lEnd[0], origin[0], rEnd[0]]

    # Latitude interpolation function 
    f_lat = interpolate.interp1d(y_lat, y_cart)

    # Longitude interpolation function
    f_lon = interpolate.interp1d(x_lon, x_cart)
       
    # Converting (latitude, longitude) to (x,y) using interpolation function
    y, x = f_lat(geoLocation[0]), f_lon(geoLocation[1])
    return (float(x),float(y))

def cartToGeo(origin, endDistance, cartLocation):
    # The initial point of rectangle in (x,y) is (0,0) so considering the current
    # location as origin and retreiving the latitude and longitude from the GPS
    # origin = (12.948048, 80.139742) Format

    # Calculating the hypot end point for interpolating the latitudes and longitudes 
    rEndDistance = math.sqrt(2*(endDistance**2))

    # The bearing for the hypot angle is 45 degrees considering coverage area as square
    bearing = 45

    # Determining the Latitude and Longitude of Middle point of the sqaure area
    # and hypot end point of square area for interpolating latitude and longitude
    lEnd, rEnd = destination_location(origin[0], origin[1], rEndDistance, 180+bearing), destination_location(origin[0], origin[1], rEndDistance, bearing)

    # Array of (x,y)
    x_cart, y_cart  = [-endDistance, 0, endDistance], [-endDistance, 0, endDistance]

    # Array of (latitude, longitude)
    x_lon, y_lat = [lEnd[1], origin[1], rEnd[1]], [lEnd[0], origin[0], rEnd[0]]

    # Latitude interpolation function 
    f_lat = interpolate.interp1d(y_cart, y_lat)

    # Longitude interpolation function
    f_lon = interpolate.interp1d(x_cart, x_lon)
       
    # Converting (latitude, longitude) to (x,y) using interpolation function
    lat, lon = f_lat(cartLocation[1]), f_lon(cartLocation[0])
    return (lat, lon)
