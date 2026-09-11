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
from utils import compass_to_math_rad


class Vehicles:
    def __init__(self, conn_str, heartbeat_timeout=3, max_workers=None):
        self.conn_str = conn_str
        self.heartbeat_timeout = heartbeat_timeout
        self.max_workers = max_workers or len(conn_str)
        self.drones = [None] * len(conn_str)
        self.connect_all()
        print("Drones..........", self.drones)

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

        self.drones = [drone for drone in self.drones if drone is not None]

        connected = len(self.drones)
        print(f"Number of Drones Connected: {connected}/{len(self.conn_str)}")

    def _connect_one(self, index, conn_string):
        try:
            vehicle = connect(conn_string, heartbeat_timeout=self.heartbeat_timeout)

            self.drones[index] = vehicle
            print(f"Drone {index} connected ({conn_string})")
        except APIException:
            print(f"Drone {index} failed: no heartbeat ({conn_string})")
        except Exception as e:
            print(f"Drone {index} failed: {e} ({conn_string})")

    def Loiter_param(self):
        return max(
            (
                drone.parameters.get("WP_LOITER_RAD")
                for drone in self.drones
                if drone.parameters is not None
                and drone.parameters.get("WP_LOITER_RAD") is not None
            ),
            default=None,
        )
