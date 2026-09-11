"""ArduPilot terrain elevation: reads the ``.DAT`` grid-block cache files.

DatTerrainManager -- reads ground elevation (metres above mean sea
                     level) at a lat/lon from ArduPilot terrain ``.DAT``
                     files: the format a flight controller / SITL stores
                     on the SD card (``APM/TERRAIN``) and that the
                     ArduPilot terrain generator (``terraingen/``) emits.
                     One 1-degree tile per file, named like
                     ``N30E080.DAT``, holding 2 KiB "grid blocks". Each
                     block is a 28x32 grid of int16 heights on a
                     ``spacing``-metre lattice, with an availability
                     bitmap and a CRC. This mirrors
                     ``elevation.TerrainManager.get_elevation()`` so the
                     two can be used interchangeably.

The reader is kept byte- and math-compatible with the generator in
``terraingen/`` (``terrain_gen.py`` writer, ``slice_graph.py``
``dat_altitude()`` reader, ``offline_check.py`` validator):

  * struct layout + CRC-16/XMODEM over the first 1821 bytes with the crc
    field zeroed (``offline_check.check_filled``);
  * the ``"4.1"`` coordinate format -- ``longitude_scale`` at the
    latitude midpoint -- with single-precision-float emulation, matching
    what runs on the vehicle;
  * a block is located by the ``grid_idx_x`` / ``grid_idx_y`` in its own
    trailer, then the height is bilinearly interpolated exactly as
    ``AP_Terrain::height_amsl`` (``slice_graph.dat_altitude``).

Unlike SRTM ``.hgt`` there is no public download for ``.DAT`` tiles -- a
flight controller fills them in over MAVLink from the GCS, SITL writes
them itself, or you pre-generate them with ``terraingen/fast_gen.py``.
This module only reads what is already on disk; a missing tile, a bad
CRC, or an unfilled grid square all yield ``None``. ``.DAT`` and
gzip-compressed ``.DAT.gz`` are both accepted.

Run directly for a one-off lookup:
    python3 elevation_dat.py <lat> <lon> [terrain_dir]
"""

import binascii
import gzip
import math
import struct
import sys
from pathlib import Path

# --- ArduPilot AP_Terrain constants (AP_Terrain.h / terrain_gen.py) -----
_MAVLINK_SIZE = 4
_BLOCK_MUL_X = 7
_BLOCK_MUL_Y = 8
# spacing between 28x32 grid blocks, in grid_spacing units (one square of
# overlap between neighbouring blocks so any point resolves from one block)
_BLOCK_SPACING_X = (_BLOCK_MUL_X - 1) * _MAVLINK_SIZE  # 24
_BLOCK_SPACING_Y = (_BLOCK_MUL_Y - 1) * _MAVLINK_SIZE  # 28
# a disk grid_block is 28 (north, gx) x 32 (east, gy)
_BLOCK_SIZE_X = _MAVLINK_SIZE * _BLOCK_MUL_X  # 28
_BLOCK_SIZE_Y = _MAVLINK_SIZE * _BLOCK_MUL_Y  # 32
_HEIGHT_COUNT = _BLOCK_SIZE_X * _BLOCK_SIZE_Y  # 896

_IO_BLOCK_BYTES = 2048       # grid_block padded to a 2 KiB boundary on disk
_IO_BLOCK_DATA_BYTES = 1821  # bytes actually used (CRC covers exactly these)
_GRID_FORMAT_VERSION = 1

# geodesy constant (AP_Math/definitions.h), narrowed to float32 like the
# generator does, because that is the precision the vehicle computes at.
def _f32(x):
    return struct.unpack("f", struct.pack("f", x))[0]

_LATLON_TO_M = _f32(0.011131884502145034)

# struct grid_block, PACKED little-endian:
#   uint64 bitmap; int32 lat; int32 lon; uint16 crc; uint16 version;
#   uint16 spacing; int16 height[28][32]; uint16 grid_idx_x;
#   uint16 grid_idx_y; int16 lon_degrees; int8 lat_degrees;
#   uint8 version_minor;
_HEADER_STRUCT = struct.Struct("<QiiHHH")            # through spacing (22 B)
_TRAILER_STRUCT = struct.Struct("<HHhb")             # grid_idx_x..lat_degrees
_HEIGHTS_STRUCT = struct.Struct(f"<{_HEIGHT_COUNT}h")
_HEIGHTS_OFFSET = _HEADER_STRUCT.size                # 22
_TRAILER_OFFSET = _HEIGHTS_OFFSET + _HEIGHTS_STRUCT.size  # 1814
_CRC_FIELD_OFFSET = 16                               # uint16 crc within header

assert _TRAILER_OFFSET + _TRAILER_STRUCT.size == 1821


def _crc16_xmodem(data):
    """CRC-16/CCITT-XMODEM (poly 0x1021, init 0). == ``fastcrc.crc16.xmodem``.

    ``binascii.crc_hqx`` is the same algorithm in C; keep a pure-Python
    fallback in case it is ever unavailable.
    """
    try:
        return binascii.crc_hqx(data, 0)
    except Exception:
        crc = 0
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
        return crc


def _longitude_scale(lat_deg):
    """cos(latitude in degrees), floored at 0.01, float32-emulated."""
    return max(_f32(math.cos(_f32(math.radians(lat_deg)))), 0.01)


def _diff_longitude_e7(lon1, lon2):
    """lon1 - lon2 in 1e-7 degrees, wrapped across the antimeridian."""
    if (lon1 < 0) == (lon2 < 0):
        return lon1 - lon2
    dlon = lon1 - lon2
    if dlon > 1800000000:
        dlon -= 3600000000
    elif dlon < -1800000000:
        dlon += 3600000000
    return dlon


def _distance_ne_e7(lat1_e7, lon1_e7, lat2_e7, lon2_e7):
    """Metres (north, east) from point 1 to point 2 -- Location's "4.1"
    get_distance_NE: longitude_scale taken at the latitude midpoint."""
    dlat = lat2_e7 - lat1_e7
    dlng = _diff_longitude_e7(lon2_e7, lon1_e7) * _longitude_scale(
        (lat1_e7 + lat2_e7) * 0.5 * 1.0e-7
    )
    return dlat * _LATLON_TO_M, dlng * _LATLON_TO_M


class _Tile:
    __slots__ = ("blocks", "ref_lat_e7", "ref_lon_e7", "spacing")

    def __init__(self, blocks, lat_degrees, lon_degrees, spacing):
        # (grid_idx_x, grid_idx_y) -> (bitmap, heights tuple)
        self.blocks = blocks
        self.ref_lat_e7 = lat_degrees * 10000000
        self.ref_lon_e7 = lon_degrees * 10000000
        self.spacing = spacing


class DatTerrainManager:
    """Reads ArduPilot ``.DAT`` terrain tiles from a directory.

    Same shape as ``elevation.TerrainManager``: construct once, call
    ``get_elevation(lat, lon)`` per sample, ``close()`` at the end. Each
    tile is read + CRC-verified once on first use and cached in memory;
    later lookups are an index computation and two dict hits.
    """

    def __init__(self, terrain_dir=None, grid_spacing=None):
        # Default to the repo's own DAT/ folder (next to this file), so it
        # resolves the same regardless of the process's working directory.
        self.terrain_dir = (
            Path(terrain_dir)
            if terrain_dir
            else Path(__file__).resolve().parent / "DAT"
        )
        # None -> take the spacing from each tile's own blocks (the tiles
        # in terraingen/ are generated at 30 m or 100 m). An int forces it
        # and rejects tiles that disagree.
        self.grid_spacing = int(grid_spacing) if grid_spacing else None
        # tile name -> _Tile, or None if it isn't on disk / has no data
        self._tiles = {}

    # -- tile / file handling ---------------------------------------------

    def get_tile_name(self, lat, lon):
        """ArduPilot degree-file name, e.g. ``N30E080`` (no extension)."""
        lat_d = math.floor(lat)
        lon_d = math.floor(lon)
        lat_name = f"N{lat_d:02d}" if lat_d >= 0 else f"S{abs(lat_d):02d}"
        lon_name = f"E{lon_d:03d}" if lon_d >= 0 else f"W{abs(lon_d):03d}"
        return f"{lat_name}{lon_name}"

    def _load_bytes(self, name):
        """Raw tile bytes (decompressing ``.DAT.gz``), or None if absent."""
        plain = self.terrain_dir / f"{name}.DAT"
        if plain.exists():
            data = plain.read_bytes()
            return data if len(data) >= _IO_BLOCK_DATA_BYTES else None
        packed = self.terrain_dir / f"{name}.DAT.gz"
        if packed.exists():
            try:
                with gzip.open(packed, "rb") as fh:
                    data = fh.read()
            except OSError as e:
                print(f"[TERRAIN] Could not read {packed.name}: {e}")
                return None
            return data if len(data) >= _IO_BLOCK_DATA_BYTES else None
        return None

    def _tile(self, name):
        """Parsed + validated _Tile for `name`, or None."""
        if name in self._tiles:
            return self._tiles[name]

        data = self._load_bytes(name)
        tile = self._parse_tile(data) if data is not None else None
        self._tiles[name] = tile
        return tile

    def _parse_tile(self, data):
        # DAT files are a whole number of 2 KiB blocks, except the last
        # block may have its 227-byte trailing pad omitted.
        size = len(data)
        if size % _IO_BLOCK_BYTES == 0:
            n_blocks = size // _IO_BLOCK_BYTES
        elif (size + _IO_BLOCK_BYTES - _IO_BLOCK_DATA_BYTES) % _IO_BLOCK_BYTES == 0:
            n_blocks = (size + _IO_BLOCK_BYTES - _IO_BLOCK_DATA_BYTES) // _IO_BLOCK_BYTES
        else:
            return None

        forced = self.grid_spacing
        spacing = forced
        blocks = {}
        tile_lat_deg = tile_lon_deg = 0

        for i in range(n_blocks):
            base = i * _IO_BLOCK_BYTES
            raw = data[base:base + _IO_BLOCK_DATA_BYTES]
            if len(raw) < _IO_BLOCK_DATA_BYTES:
                break

            header = _HEADER_STRUCT.unpack(raw[:_HEADER_STRUCT.size])
            bitmap, crc, version, blk_spacing = header[0], header[3], header[4], header[5]
            if version != _GRID_FORMAT_VERSION or bitmap == 0:
                continue  # empty / never-written block
            if forced is not None and blk_spacing != forced:
                continue
            if spacing is None:
                spacing = blk_spacing
            elif blk_spacing != spacing:
                continue

            zeroed = raw[:_CRC_FIELD_OFFSET] + b"\x00\x00" + raw[_CRC_FIELD_OFFSET + 2:]
            if _crc16_xmodem(zeroed) != crc:
                continue

            gix, giy, lon_deg, lat_deg = _TRAILER_STRUCT.unpack(
                raw[_TRAILER_OFFSET:_TRAILER_OFFSET + _TRAILER_STRUCT.size]
            )
            heights = _HEIGHTS_STRUCT.unpack(
                raw[_HEIGHTS_OFFSET:_HEIGHTS_OFFSET + _HEIGHTS_STRUCT.size]
            )
            blocks[(gix, giy)] = (bitmap, heights)
            tile_lat_deg, tile_lon_deg = lat_deg, lon_deg

        if not blocks:
            return None
        return _Tile(blocks, tile_lat_deg, tile_lon_deg, spacing)

    # -- lookup (ported from slice_graph.dat_altitude / height_amsl) ------

    @staticmethod
    def _bit_set(bitmap, idx_x, idx_y):
        """Is the 4x4 mavlink sub-grid covering (idx_x, idx_y) filled in?"""
        bitnum = (idx_y // _MAVLINK_SIZE) + _BLOCK_MUL_Y * (idx_x // _MAVLINK_SIZE)
        return (bitmap >> bitnum) & 1

    def get_elevation(self, latitude, longitude):
        """Ground elevation in metres AMSL at (latitude, longitude), or
        None when there's no data (tile / block not on disk, block failed
        its CRC, or a surrounding grid point isn't filled)."""
        latitude = float(latitude)
        longitude = float(longitude)

        tile = self._tile(self.get_tile_name(latitude, longitude))
        if tile is None:
            return None

        sp = tile.spacing
        # int() truncation on degrees*1e7, matching every tool in terraingen/
        p_lat = int(latitude * 1e7)
        p_lon = int(longitude * 1e7)
        off_n, off_e = _distance_ne_e7(tile.ref_lat_e7, tile.ref_lon_e7, p_lat, p_lon)

        idx_x = int(off_n / sp)
        idx_y = int(off_e / sp)
        block = tile.blocks.get((idx_x // _BLOCK_SPACING_X, idx_y // _BLOCK_SPACING_Y))
        if block is None:
            return None
        bitmap, heights = block

        lx = idx_x % _BLOCK_SPACING_X
        ly = idx_y % _BLOCK_SPACING_Y
        # one-square block overlap guarantees lx+1 / ly+1 stay in range
        if not (
            self._bit_set(bitmap, lx, ly)
            and self._bit_set(bitmap, lx, ly + 1)
            and self._bit_set(bitmap, lx + 1, ly)
            and self._bit_set(bitmap, lx + 1, ly + 1)
        ):
            return None

        # height[gx][gy] is row-major with gx north (0..27), gy east (0..31)
        row = lx * _BLOCK_SIZE_Y
        h00 = heights[row + ly]
        h01 = heights[row + ly + 1]
        h10 = heights[row + _BLOCK_SIZE_Y + ly]
        h11 = heights[row + _BLOCK_SIZE_Y + ly + 1]

        fx = (off_n - idx_x * sp) / sp
        fy = (off_e - idx_y * sp) / sp
        avg1 = (1.0 - fx) * h00 + fx * h10
        avg2 = (1.0 - fx) * h01 + fx * h11
        return (1.0 - fy) * avg1 + fy * avg2

    def close(self):
        self._tiles.clear()


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print(f"usage: python3 {Path(__file__).name} <lat> <lon> [terrain_dir]")
        raise SystemExit(2)

    lat, lon = float(sys.argv[1]), float(sys.argv[2])
    terrain_dir = sys.argv[3] if len(sys.argv) > 3 else None

    terrain = DatTerrainManager(terrain_dir)
    print(f"tile     : {terrain.get_tile_name(lat, lon)}.DAT")
    elevation = terrain.get_elevation(lat, lon)
    if elevation is None:
        print("elevation: no data")
    else:
        print(f"elevation: {elevation:.1f} m AMSL")
    terrain.close()
