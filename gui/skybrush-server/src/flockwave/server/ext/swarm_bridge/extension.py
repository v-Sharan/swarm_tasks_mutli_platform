"""Extension entry point: pre-registers this project's bots as UAVs at
startup (ids are known up front from config, unlike MAVLink's heartbeat-
driven discovery), then polls main.py's /api/telemetry endpoint and turns
each poll into a pushed UAV-INF update.
"""

from __future__ import annotations

import httpx
import trio

from flockwave.gps.vectors import GPSCoordinate
from flockwave.server.ext.base import UAVExtension
from flockwave.server.model.battery import BatteryInfo
from flockwave.server.registries.errors import RegistryFull

from .driver import SwarmBridgeDriver, SwarmBridgeUAV


class SwarmBridgeExtension(UAVExtension[SwarmBridgeDriver]):
    _uavs: list[SwarmBridgeUAV] | None = None

    def _create_driver(self) -> SwarmBridgeDriver:
        return SwarmBridgeDriver()

    def configure_driver(self, driver: SwarmBridgeDriver, configuration) -> None:
        driver.command_host = str(configuration.get("command_host", "127.0.0.1"))
        driver.command_port = int(configuration.get("command_port", 12008))
        driver.default_guided_circle_direction = int(
            configuration.get("default_guided_circle_direction", 1)
        )
        driver.default_guided_circle_radius = int(
            configuration.get("default_guided_circle_radius", 50)
        )

    def _register_uav(self, uav: SwarmBridgeUAV) -> None:
        assert self.app is not None
        if self._uavs is None:
            raise RuntimeError("cannot register a UAV before the extension is started")
        try:
            self.app.object_registry.add(uav)
        except RegistryFull:
            self.app.handle_registry_full_error(self, "swarm bridge UAV")
        self._uavs.append(uav)

    def _unregister_all_uavs(self) -> None:
        assert self.app is not None
        if self._uavs:
            for uav in self._uavs:
                self.app.object_registry.remove(uav)
        self._uavs = None

    @staticmethod
    def _mode_string(bot: dict) -> str:
        """Encodes vehicle type + current job into the stock `mode` status
        field -- see the plan's "verify-then-upgrade" note on why this
        isn't a dedicated UAVStatusInfo field yet.
        """
        vehicle_type = bot.get("vehicle_type") or "unknown"
        if bot.get("removed"):
            activity = "removed"
        elif bot.get("job"):
            activity = bot["job"]
        elif bot.get("loitering"):
            activity = "loiter"
        elif bot.get("done"):
            activity = "done"
        else:
            activity = "idle"
        return f"{vehicle_type}:{activity}"

    async def run(self, app, configuration) -> None:
        num_bots = int(configuration.get("num_bots", 10))
        id_format = str(configuration.get("id_format", "{0}"))
        telemetry_url = str(
            configuration.get("telemetry_url", "http://127.0.0.1:8765/api/telemetry")
        )
        poll_interval = float(configuration.get("poll_interval", 0.5))

        self._uavs = []
        try:
            uavs_by_id: dict[str, SwarmBridgeUAV] = {}
            for i in range(1, num_bots + 1):
                uav_id = id_format.format(i)
                uav = SwarmBridgeUAV(id=uav_id, driver=self.driver)
                self._register_uav(uav)
                uavs_by_id[uav_id] = uav

            assert self.log is not None
            self.log.info(f"swarm_bridge: registered {num_bots} UAV(s), polling {telemetry_url}")

            was_connected = False
            async with httpx.AsyncClient(timeout=poll_interval * 4) as client:
                while True:
                    try:
                        response = await client.get(telemetry_url)
                        response.raise_for_status()
                        payload = response.json()
                    except Exception as exc:
                        if was_connected:
                            self.log.warning(f"swarm_bridge: telemetry poll failed: {exc!r}")
                        was_connected = False
                        await trio.sleep(poll_interval)
                        continue

                    if not was_connected:
                        self.log.info(
                            f"swarm_bridge: telemetry connected, {len(payload.get('bots', ()))} bot(s) reporting"
                        )
                        was_connected = True

                    for bot in payload.get("bots", ()):
                        uav = uavs_by_id.get(str(bot.get("id")))
                        if uav is None:
                            continue

                        lat, lon = bot.get("lat"), bot.get("lon")
                        position = (
                            GPSCoordinate(lat=lat, lon=lon, amsl=bot.get("alt"))
                            if lat is not None and lon is not None
                            else None
                        )

                        battery = None
                        if bot.get("battery_voltage") is not None:
                            battery = BatteryInfo()
                            battery.voltage = bot["battery_voltage"]

                        uav.update_status(
                            position=position,
                            heading=bot.get("heading"),
                            mode=self._mode_string(bot),
                            battery=battery,
                        )

                    await trio.sleep(poll_interval)
        finally:
            self._unregister_all_uavs()
