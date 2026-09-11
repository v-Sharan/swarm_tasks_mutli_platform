"""SRTM terrain elevation: tile download/cache + a clearance watchdog.

TerrainManager  -- downloads SRTM1 (.hgt) tiles from the AWS open-data
                   mirror and reads ground elevation (metres above mean
                   sea level) at a lat/lon. Each tile is memory-mapped on
                   first use and kept open, so repeated lookups (one per
                   drone per tick) cost only a seek + a 2-byte read, with
                   no per-call file open and no log spam.
TerrainMonitor  -- compares ground-elevation samples against a drone's
                   own AMSL altitude and reports when the ground rises to
                   within a clearance margin of it. Advisory only: it
                   never edits a mission or a goal point. It resolves
                   each sample from the ArduPilot .DAT tiles first (via
                   elevation_dat.DatTerrainManager -- the exact terrain
                   the flight controller flies to) and falls back to the
                   SRTM .hgt tiles wherever a .DAT tile/block is missing.

Run directly for a one-off lookup / bulk download:
    python3 elevation.py
"""

import gzip
import math
import mmap
import shutil
import struct
import urllib.request
from pathlib import Path

try:
    # ArduPilot .DAT terrain tiles -- the exact data the flight controller
    # flies to. Optional: if the module is missing, TerrainMonitor just
    # runs on SRTM .hgt as before.
    from elevation_dat import DatTerrainManager
except Exception:  # pragma: no cover - defensive
    DatTerrainManager = None

# SRTM1 = 1 arc-second, 3601 x 3601 samples per 1-degree tile.
_TILE_SIZE = 3601
_SRTM_VOID = -32768


class TerrainManager:

    BASE_URL = "https://s3.amazonaws.com/elevation-tiles-prod/skadi"

    def __init__(self, terrain_dir=None, auto_download=True):
        # Default to the repo's own terrain/ folder (next to this file),
        # so it resolves the same regardless of the process's working
        # directory -- the pre-downloaded tiles already live there.
        self.terrain_dir = (
            Path(terrain_dir)
            if terrain_dir
            else Path(__file__).resolve().parent / "terrain"
        )
        self.terrain_dir.mkdir(parents=True, exist_ok=True)
        self.auto_download = auto_download
        # tile name -> open mmap of its .hgt file, kept for the life of
        # the manager; the OS pages it in on demand.
        self._tiles = {}
        # Tiles already known to be unavailable -- don't retry the
        # download (or re-print the failure) on every single lookup.
        self._missing = set()

    def get_tile_name(self, lat, lon):
        lat = math.floor(lat)
        lon = math.floor(lon)

        lat_name = f"N{lat:02d}" if lat >= 0 else f"S{abs(lat):02d}"
        lon_name = f"E{lon:03d}" if lon >= 0 else f"W{abs(lon):03d}"

        return f"{lat_name}{lon_name}"

    def download_range(self, latitude, longitude, range_km):
        """
        Download all SRTM tiles covering a square area
        around the given latitude/longitude.

        range_km = distance from center in every direction.
        """

        # Approximate conversion
        lat_delta = range_km / 111.0

        # Longitude degrees vary with latitude
        lon_delta = range_km / (111.0 * math.cos(math.radians(latitude)))

        min_lat = math.floor(latitude - lat_delta)
        max_lat = math.floor(latitude + lat_delta)

        min_lon = math.floor(longitude - lon_delta)
        max_lon = math.floor(longitude + lon_delta)

        print()
        print("Terrain download area:")
        print(f"Center : {latitude}, {longitude}")
        print(f"Range  : {range_km} km")
        print(f"Lat    : {min_lat} -> {max_lat}")
        print(f"Lon    : {min_lon} -> {max_lon}")
        print()

        total = (max_lat - min_lat + 1) * (max_lon - min_lon + 1)

        count = 0

        for lat in range(min_lat, max_lat + 1):

            for lon in range(min_lon, max_lon + 1):

                count += 1

                print(f"[{count}/{total}]")

                self.download_tile(lat, lon)

        print()
        print("[TERRAIN] Range download completed.")

    def download_tile(self, lat, lon):

        lat = math.floor(lat)
        lon = math.floor(lon)

        tile = self.get_tile_name(lat, lon)

        hgt_file = self.terrain_dir / f"{tile}.hgt"
        gz_file = self.terrain_dir / f"{tile}.hgt.gz"

        # Already downloaded
        if hgt_file.exists():
            print(f"[TERRAIN] Using cached {tile}.hgt")
            return hgt_file

        # Latitude folder
        lat_folder = f"N{lat:02d}" if lat >= 0 else f"S{abs(lat):02d}"

        url = f"{self.BASE_URL}/{lat_folder}/{tile}.hgt.gz"

        print(f"[TERRAIN] Downloading {tile}.hgt")

        try:

            urllib.request.urlretrieve(url, gz_file)

            with gzip.open(gz_file, "rb") as source:
                with open(hgt_file, "wb") as destination:
                    shutil.copyfileobj(source, destination)

            gz_file.unlink()

            print(f"[TERRAIN] Downloaded {tile}.hgt")

            return hgt_file

        except Exception as e:

            print(f"[TERRAIN] Download failed: {e}")

            if gz_file.exists():
                gz_file.unlink()

            if hgt_file.exists():
                hgt_file.unlink()

            return None

    def _tile(self, latitude, longitude):
        """The mmap for the tile containing (latitude, longitude), or
        None if it isn't on disk and can't be downloaded. Opened once,
        then served from the cache."""
        name = self.get_tile_name(latitude, longitude)
        cached = self._tiles.get(name)
        if cached is not None:
            return cached
        if name in self._missing:
            return None

        hgt_file = self.terrain_dir / f"{name}.hgt"
        if not hgt_file.exists():
            got = (
                self.download_tile(math.floor(latitude), math.floor(longitude))
                if self.auto_download
                else None
            )
            if not got:
                self._missing.add(name)
                return None

        try:
            handle = open(hgt_file, "rb")
            try:
                # mmap dups the fd, so the file object can be closed
                # straight away and the mapping stays valid.
                tile = mmap.mmap(handle.fileno(), 0, access=mmap.ACCESS_READ)
            finally:
                handle.close()
        except OSError as e:
            print(f"[TERRAIN] Could not open {name}.hgt: {e}")
            self._missing.add(name)
            return None

        self._tiles[name] = tile
        return tile

    def get_elevation(self, latitude, longitude):
        """Ground elevation in metres AMSL at (latitude, longitude), or
        None when there's no data (tile unavailable, or an SRTM void)."""
        latitude = float(latitude)
        longitude = float(longitude)

        tile = self._tile(latitude, longitude)
        if tile is None:
            return None

        lat_base = math.floor(latitude)
        lon_base = math.floor(longitude)

        # Position inside the tile
        lat_fraction = latitude - lat_base
        lon_fraction = longitude - lon_base

        # Sample index. Row 0 is the tile's NORTH edge, hence 1 - frac.
        # Truncated (not rounded) to match the original read exactly.
        row = int((1.0 - lat_fraction) * (_TILE_SIZE - 1))
        col = int(lon_fraction * (_TILE_SIZE - 1))
        row = max(0, min(_TILE_SIZE - 1, row))
        col = max(0, min(_TILE_SIZE - 1, col))

        offset = (row * _TILE_SIZE + col) * 2
        elevation = struct.unpack(">h", tile[offset : offset + 2])[0]

        if elevation == _SRTM_VOID:
            return None
        return elevation

    def close(self):
        for tile in self._tiles.values():
            tile.close()
        self._tiles.clear()


class TerrainMonitor:
    """Terrain-clearance watchdog. Advisory only -- it reports, it never
    steers.

    The caller owns the coordinate frame, so it resolves its own sample
    points to lat/lon and passes them here together with the drone's
    current AMSL altitude. worst_breach() returns the highest sampled
    ground that sits within `clearance_m` of that altitude, or None when
    everything is clear.

    Ground elevation for each sample comes from the ArduPilot .DAT tiles
    first (elevation_dat.DatTerrainManager) with the SRTM .hgt tiles as
    the fallback; set prefer_dat=False to swap the order, or use_dat=
    False to run on .hgt only.
    """

    def __init__(
        self,
        origin,
        terrain_dir=None,
        clearance_m=30.0,
        lookahead_m=1000.0,
        sample_step_m=200.0,
        auto_download=True,
        use_dat=True,
        prefer_dat=True,
        dat_terrain_dir=None,
        dat_grid_spacing=None,
    ):
        self.origin = origin
        # SRTM .hgt source (downloadable, global coverage).
        self.manager = TerrainManager(terrain_dir, auto_download=auto_download)
        # ArduPilot .DAT source (elevation_dat) -- the exact terrain the
        # flight controller flies to. Preferred when a tile is present,
        # with .hgt as the fallback everywhere else.
        self.dat_manager = None
        if use_dat and DatTerrainManager is not None:
            try:
                self.dat_manager = DatTerrainManager(
                    dat_terrain_dir, grid_spacing=dat_grid_spacing
                )
            except Exception as exc:
                print(f"[TERRAIN] .DAT source disabled: {exc}")
        self.prefer_dat = prefer_dat
        self.clearance_m = clearance_m
        self.lookahead_m = lookahead_m
        self.sample_step_m = sample_step_m

    def prefetch(self, latitude, longitude, range_km):
        """Best-effort: pull the SRTM .hgt tiles covering the mission area
        now, so the first in-flight lookup doesn't stall on a download.
        Never fatal -- a missing tile with no network is just logged.
        (.DAT tiles are pre-placed on disk and have no download, so there
        is nothing to prefetch for that source.)"""
        try:
            self.manager.download_range(latitude, longitude, range_km)
        except Exception as exc:
            print(f"[TERRAIN] prefetch skipped: {exc}")

    def ground_elevation(self, lat, lon):
        """(height_amsl, source) at lat/lon, or (None, None).

        `source` is "dat" or "hgt". Tries the .DAT source first (or .hgt
        first when prefer_dat is False) and falls back to the other
        wherever the preferred one has no data for that point."""
        order = ("dat", "hgt") if self.prefer_dat else ("hgt", "dat")
        for source in order:
            mgr = self.dat_manager if source == "dat" else self.manager
            if mgr is None:
                continue
            height = mgr.get_elevation(lat, lon)
            if height is not None:
                return height, source
        return None, None

    def worst_breach(self, samples, drone_amsl):
        """samples: iterable of (lat, lon, dist_ahead_m).

        Returns (lat, lon, ground_amsl, dist_ahead_m, source) for the
        highest ground that comes within clearance_m of drone_amsl, or
        None if every sample clears. `source` is "dat" or "hgt"."""
        worst = None
        for lat, lon, dist in samples:
            ground, source = self.ground_elevation(lat, lon)
            if ground is None:
                continue
            if ground + self.clearance_m >= drone_amsl:
                if worst is None or ground > worst[2]:
                    worst = (lat, lon, ground, dist, source)
        return worst


if __name__ == "__main__":
    terrain = TerrainManager()
    takeoff = terrain.get_elevation(30.3061890, 80.3537493)
    print(takeoff)
    # crash = terrain.get_elevation(30.3020197, 80.3455317)
    # crash = terrain.get_elevation(30.3001283, 80.3466306)
    # print(crash - takeoff)
    # terrain.download_range(30.3020197, 80.3455317, 5)
