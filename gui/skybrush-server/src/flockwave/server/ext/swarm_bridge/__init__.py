"""Skybrush server extension that bridges to this project's own main.py
swarm process: commands go out over UDP in main.py's exact wire format,
and telemetry comes in by polling main.py's /api/telemetry endpoint.
"""

from .extension import SwarmBridgeExtension
from .schema import schema

__all__ = ("construct", "dependencies", "description", "schema")

construct = SwarmBridgeExtension
dependencies = ()
description = "Bridges to this project's main.py swarm process over UDP (commands) and HTTP (telemetry)"
