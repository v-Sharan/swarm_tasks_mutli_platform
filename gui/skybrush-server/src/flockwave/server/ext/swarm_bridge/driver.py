"""Driver that translates generic Flockwave UAV commands into the exact
UDP wire format main.py's command loop expects (see main.py:43-137 and
send_command_example.py for the authoritative formats this must match
byte-for-byte).
"""

from __future__ import annotations

import json
import socket
from typing import TYPE_CHECKING

from flockwave.gps.vectors import GPSCoordinate

from flockwave.server.model.uav import UAVBase, UAVDriver

if TYPE_CHECKING:
    pass


class SwarmBridgeDriver(UAVDriver["SwarmBridgeUAV"]):
    """Sends commands to main.py over a plain UDP socket, one datagram per
    command -- fire-and-forget, exactly like send_command_example.py's
    SwarmClient. main.py never replies, so there is nothing to await.
    """

    command_host: str = "127.0.0.1"
    command_port: int = 12008
    default_guided_circle_direction: int = 1
    default_guided_circle_radius: int = 50

    def _send_udp(self, message: str) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.sendto(message.encode("utf-8"), (self.command_host, self.command_port))
        finally:
            sock.close()

    # -- stock Flockwave signal vocabulary -----------------------------
    #
    # Only "fly to target" has a reasonable single-point equivalent in
    # main.py's protocol (a one-waypoint "goal" for one bot). The rest of
    # main.py's commands have no single-UAV stock equivalent and are
    # exposed exclusively as custom commands below.

    def _send_fly_to_target_signal_single(self, uav: "SwarmBridgeUAV", target: GPSCoordinate):
        """Convenience wrapper so the stock map-click "fly to target" UX
        works for a single bot: wraps the one point into a one-waypoint
        "goal" command for just that bot, using the extension's configured
        default direction/radius (the stock dialog has no field for them).
        """
        self._send_udp(
            "goal_{}_{}_{}_{}".format(
                json.dumps([[target.lat, target.lon]]),
                self.default_guided_circle_direction,
                self.default_guided_circle_radius,
                json.dumps([uav.id]),
            )
        )
        return True

    # -- custom commands, one per main.py wire command ------------------
    #
    # send_command()/the dispatch machinery in UAVDriver calls these as
    # handle_multi_command_<name>(uavs, *args, **kwds) (all targeted UAVs
    # in one call) or handle_command_<name>(uav, *args, **kwds) (once per
    # UAV) -- see model/uav.py's send_command().

    def handle_multi_command_goal(
        self, uavs, goal_latlon, direction=None, radius=None, ids=None
    ):
        """goal_<json goal_latlon>_<direction>_<radius>_<json ids>

        goal_latlon: list of [lat, lon] waypoint pairs (a path, not a
        single point). ids defaults to the targeted UAVs' own ids.
        """
        effective_ids = ids if ids is not None else [uav.id for uav in uavs]
        self._send_udp(
            "goal_{}_{}_{}_{}".format(
                json.dumps(goal_latlon),
                direction if direction is not None else self.default_guided_circle_direction,
                radius if radius is not None else self.default_guided_circle_radius,
                json.dumps(effective_ids),
            )
        )
        return dict.fromkeys(uavs, True)

    def handle_multi_command_stop(self, uavs, *args, **kwds):
        """stop -- a GLOBAL stop of whatever main.py is currently running
        (search/goal/split), not a per-UAV command. uavs is ignored.
        """
        self._send_udp("stop")
        return dict.fromkeys(uavs, True)

    def handle_multi_command_search(
        self, uavs, center_lat, center_lon, grid_space, coverage_area, num_uavs=None
    ):
        """search,<center_lat>,<center_lon>,<num_uavs>,<grid_space>,<coverage_area>

        Always targets every active bot on the main.py side regardless of
        what's passed here (main.py ignores num_uavs and recomputes it);
        uavs is ignored.
        """
        self._send_udp(
            f"search,{center_lat},{center_lon},{num_uavs or 0},{grid_space},{coverage_area}"
        )
        return dict.fromkeys(uavs, True)

    def handle_multi_command_split(
        self, uavs, center_lat_lon_array, grid_space, coverage_area, selected_uav_ids=None
    ):
        """split_<json center_lat_lon_array>_<json selected_uav_ids>_<json grid_space>_<json coverage_area>"""
        effective_ids = (
            selected_uav_ids if selected_uav_ids is not None else [int(uav.id) for uav in uavs]
        )
        self._send_udp(
            "split_{}_{}_{}_{}".format(
                json.dumps(center_lat_lon_array),
                json.dumps(effective_ids),
                json.dumps(grid_space),
                json.dumps(coverage_area),
            )
        )
        return dict.fromkeys(uavs, True)

    def handle_multi_command_specificsplit(
        self, uavs, center_lat_lon_array, uav_array, grid_space, coverage_area
    ):
        """specificsplit_<json center_lat_lon_array>_<json uav_array>_<json grid_space>_<json coverage_area>

        uav_array is a list of per-region id lists (e.g. [[1,2],[3,4]]) --
        it must come from the caller, it cannot be derived from the flat
        `uavs` list.
        """
        self._send_udp(
            "specificsplit_{}_{}_{}_{}".format(
                json.dumps(center_lat_lon_array),
                json.dumps(uav_array),
                json.dumps(grid_space),
                json.dumps(coverage_area),
            )
        )
        return dict.fromkeys(uavs, True)

    def handle_multi_command_different(self, uavs, start, diff):
        """different,<start>,<diff> -- sets every bot's commanded height;
        uavs is ignored.
        """
        self._send_udp(f"different,{start},{diff}")
        return dict.fromkeys(uavs, True)

    def handle_command_remove(self, uav, *args, **kwds):
        """remove_<json [id]> -- self-heal: freeze this one bot. Dispatched
        once per targeted UAV (singular handle_command_, not
        handle_multi_command_).
        """
        self._send_udp(f"remove_{json.dumps([uav.id])}")
        return True

    def handle_command_add(self, uav, *args, **kwds):
        """add_<json [id]> -- bring this one bot back into service."""
        self._send_udp(f"add_{json.dumps([uav.id])}")
        return True


class SwarmBridgeUAV(UAVBase[SwarmBridgeDriver]):
    """No overrides needed -- UAVBase's default status/device-tree handling
    is exactly what this bridge needs; all the real work is encoding
    outbound commands (SwarmBridgeDriver, above) and decoding inbound
    telemetry (update_status() calls from SwarmBridgeExtension.run()).
    """
