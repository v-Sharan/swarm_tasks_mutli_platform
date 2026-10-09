from concurrent.futures import ThreadPoolExecutor, as_completed

# dronekit (2.9.2, last released 2019) still does `collections.MutableMapping`,
# which was removed from `collections` in Python 3.10 (moved to
# `collections.abc` back in 3.3). Restore the old aliases before importing it
# so `from dronekit import ...` below doesn't crash with AttributeError on
# modern Python.
import collections
import collections.abc

for _name in ("MutableMapping", "Mapping", "Sequence", "Iterable"):
    if not hasattr(collections, _name):
        setattr(collections, _name, getattr(collections.abc, _name))

from dronekit import connect, APIException
from pymavlink import mavutil
from utils import compass_to_math_rad

# Hybrid swarm support: a connected drone's real airframe (quadcopter,
# fixed-wing plane, VTOL, rover, ...) is reported in every MAVLink
# HEARTBEAT as a MAV_TYPE_* value, which dronekit already captures into
# vehicle._vehicle_type as soon as connect() gets its first heartbeat --
# no extra MAVLink traffic needed. We collapse that down to the
# simulation's two kinematic models ('fixedwing': bank-limited turns,
# can't hover/pivot; 'ground': can pivot/hover in place -- covers
# multirotors, helicopters and rovers) so each bot in a mixed swarm is
# simulated with the kinematics matching its own real vehicle, instead of
# one vehicle_type hardcoded for every bot.
_FIXEDWING_MAV_TYPES = {
    mavutil.mavlink.MAV_TYPE_FIXED_WING,
    mavutil.mavlink.MAV_TYPE_VTOL_DUOROTOR,
    mavutil.mavlink.MAV_TYPE_VTOL_QUADROTOR,
    mavutil.mavlink.MAV_TYPE_VTOL_TILTROTOR,
    mavutil.mavlink.MAV_TYPE_VTOL_RESERVED2,
    mavutil.mavlink.MAV_TYPE_VTOL_RESERVED3,
    mavutil.mavlink.MAV_TYPE_VTOL_RESERVED4,
    mavutil.mavlink.MAV_TYPE_VTOL_RESERVED5,
}


def mav_type_to_vehicle_type(mav_type):
    """MAV_TYPE_* (from a drone's HEARTBEAT) -> this project's sim
    vehicle_type ('fixedwing' or 'ground'). Unrecognised/unset types
    default to 'ground', matching the simulation's original behaviour.
    """
    return "fixedwing" if mav_type in _FIXEDWING_MAV_TYPES else "ground"


class Vehicles:
    def __init__(self, conn_str, heartbeat_timeout=3, max_workers=None):
        self.conn_str = conn_str
        self.heartbeat_timeout = heartbeat_timeout
        self.max_workers = max_workers or len(conn_str)
        self.drones = [None] * len(conn_str)
        # Auto-detected per-drone sim vehicle_type, index-aligned with
        # self.drones -- see mav_type_to_vehicle_type() above.
        self.vehicle_types = [None] * len(conn_str)
        self.connect_all()

    def get_positions(self, geoToCart, origin, endDistance):
        return [
            (
                *[
                    coord
                    for coord in geoToCart(
                        origin,
                        endDistance,
                        [
                            drone.location.global_relative_frame.lat,
                            drone.location.global_relative_frame.lon,
                        ],
                    )
                ],
                compass_to_math_rad(drone.heading),
            )
            for drone in self.drones
        ]

    def connect_all(self):
        with ThreadPoolExecutor(max_workers=self.max_workers) as executor:
            futures = {
                executor.submit(self._connect_one, i, s): i
                for i, s in enumerate(self.conn_str)
            }
            for future in as_completed(futures):
                future.result()  # re-raises any unhandled exception, if you want strictness

        # Drop failed slots (None) from both lists together, so
        # vehicle_types stays index-aligned with drones. Without this,
        # get_positions() and Loiter_param() below iterate every slot
        # unconditionally and crash with AttributeError the moment any
        # one of these ports fails to connect -- which takes down
        # Swarm's search()/goal()/AutosplitMission() entirely, not just
        # the one missing drone.
        #
        # NOTE: this renumbers bot ids relative to connection order, not
        # the original port list -- e.g. if the drone on port 14553
        # (the 3rd configured port) fails, the drone on 14554 becomes
        # bot 3, not bot 4. There's no stable port->id mapping anywhere
        # else in this codebase (remove_bot/add_bot/goal all address
        # bots by plain position in self.drones), so this matches how
        # the rest of the system already treats len(self.drones) as
        # the swarm size -- main.py passes it straight through as
        # num_bots.
        paired = [
            (drone, vtype)
            for drone, vtype in zip(self.drones, self.vehicle_types)
            if drone is not None
        ]
        self.drones = [drone for drone, _ in paired]
        self.vehicle_types = [vtype for _, vtype in paired]

        connected = len(self.drones)
        print(f"Number of Drones Connected: {connected}/{len(self.conn_str)}")

    def _connect_one(self, index, conn_string):
        try:
            vehicle = connect(conn_string, heartbeat_timeout=self.heartbeat_timeout)

            self.drones[index] = vehicle
            vtype = mav_type_to_vehicle_type(vehicle._vehicle_type)
            self.vehicle_types[index] = vtype
            print(f"Drone {index} connected ({conn_string}) -- detected vehicle_type={vtype}")
        except APIException:
            print(f"Drone {index} failed: no heartbeat ({conn_string})")
        except Exception as e:
            print(f"Drone {index} failed: {e} ({conn_string})")

    def Loiter_param(self):
        """Largest WP_LOITER_RAD among the connected drones that actually
        have it.

        WP_LOITER_RAD is an ArduPlane-only parameter -- ArduCopter has no
        such parameter at all, so indexing vehicle.parameters["WP_LOITER_RAD"]
        on a copter raises KeyError (dronekit's Parameters.__getitem__
        looks it up in the vehicle's own reported param set). In a hybrid
        swarm that's a real combination now, not a hypothetical one, so
        this uses .get() (falls back to the dict-like default, None,
        instead of raising) and filters those Nones out before taking
        the max -- an all-copter swarm correctly returns None, same as
        before when nothing had the parameter at all.
        """
        radii = [
            r
            for r in (drone.parameters.get("WP_LOITER_RAD") for drone in self.drones)
            if r is not None
        ]
        return max(radii, default=None)
