"""Configuration schema for the swarm_bridge extension."""

schema = {
    "properties": {
        "num_bots": {
            "type": "number",
            "title": "Number of bots",
            "description": "Must match the number of vehicles main.py was started with.",
            "default": 10,
        },
        "id_format": {
            "type": "string",
            "title": "UAV ID format",
            "description": (
                "Python format string for bot ids, applied to 1..num_bots. "
                "main.py itself uses plain '1'..'N', not zero-padded."
            ),
            "default": "{0}",
        },
        "command_host": {
            "type": "string",
            "title": "Command UDP host",
            "description": "LAN IP main.py printed at startup.",
            "default": "127.0.0.1",
        },
        "command_port": {
            "type": "number",
            "title": "Command UDP port",
            "default": 12008,
        },
        "telemetry_url": {
            "type": "string",
            "title": "Telemetry URL",
            "description": "main.py's ConfigServer /api/telemetry endpoint.",
            "default": "http://127.0.0.1:8765/api/telemetry",
        },
        "poll_interval": {
            "type": "number",
            "title": "Telemetry poll interval (seconds)",
            "default": 0.5,
        },
        "default_guided_circle_direction": {
            "type": "number",
            "title": "Default guided-circle direction for single-point goals",
            "default": 1,
        },
        "default_guided_circle_radius": {
            "type": "number",
            "title": "Default guided-circle radius (m) for single-point goals",
            "default": 50,
        },
    },
    "required": [],
}
