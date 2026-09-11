"""Tiny HTTP panel for editing the live Variables instance.

Serves a one-page form listing every `self.*` attribute the Variables
object happens to have -- no per-field registration, so anything added
to Variables.__init__ shows up on its own. Edits are applied to the SAME
object main.py handed to Swarm, so the running search/goal workers pick
up the new value on their next tick; nothing has to be restarted. Every
applied edit is also written out via Variables.save() (see variable.py),
so it survives a process restart too -- Variables.__init__ calls load()
to re-apply that file on top of its defaults.

Stdlib only (http.server + json), and it runs on a daemon thread, so it
can never hold the process open or drag in a dependency.

    server = ConfigServer(variables)
    server.start()          # prints the URL

Caveats it enforces rather than hides:
  * A few fields are only read when the Simulation is CONSTRUCTED
    (world_size, speed, bank_angle_deg) -- editing them at runtime does
    nothing until restart, so they are flagged, not silently accepted.
    A restart DOES pick up the saved value, since it's re-applied before
    the Simulation is built.
"""

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from variable import _coerce

# Read once at Simulation construction -- a runtime edit is inert.
RESTART_REQUIRED = {"world_size", "speed", "bank_angle_deg", "terrain_use_dat"}

# Defined on Variables but read by no code -- editing does nothing at all.
# Better to say so on the page than to let a knob look live when it isn't.
UNUSED = {"sync_lead_distance_sim"}

NOTES = {
    "origin": "lat, lon of the local frame -- read from rectangles.yaml at startup",
    "world_size": "sim world in metres",
    "speed": "fixed-wing cruise used by the sim bots",
    "bank_angle_deg": "sets the sim turn radius",
    "vehicle_loiter_radius_min_m": "lookahead distance ahead of the bot, metres",
    "bot_target_speed_mps": "how fast the planner bot advances -- above the "
    "aircraft's real cruise and it will outrun the drone",
    "tick_interval_s": "worker loop period",
    "reposition_interval_s": "min seconds between MAVLink DO_REPOSITION per bot",
    "debug_reposition": "print commanded vs actual position every 2s per bot",
    "endDistance": "geoToCart/cartToGeo scale -- other fields depend on it",
    "sync_lead_distance_sim": "how far the bot leads the drone after a resync",
    "terrain_warn": "enable the terrain-clearance watchdog (console + UDP warnings)",
    "terrain_clearance_m": "warn when ground rises within this many metres of "
    "the drone's AMSL altitude",
    "terrain_lookahead_m": "how far ahead of the drone to sample terrain, metres",
    "terrain_sample_step_m": "spacing between terrain samples along the path ahead, metres",
    "terrain_warn_host": "UDP host the terrain_warning JSON packets are sent to",
    "terrain_warn_port": "UDP port for the terrain_warning packets",
    "terrain_use_dat": "consult the ArduPilot .DAT terrain tiles in DAT/ "
    "(elevation_dat) alongside SRTM .hgt -- restart to take effect",
    "terrain_prefer_dat": "try .DAT first and fall back to .hgt "
    "(off = .hgt first, .DAT fallback)",
}


def _public_fields(variables):
    """Every instance attribute, minus privates. Order is definition order."""
    return [k for k in vars(variables) if not k.startswith("_")]


def _describe(variables):
    fields = []
    for name in _public_fields(variables):
        value = getattr(variables, name)
        fields.append(
            {
                "name": name,
                "value": list(value) if isinstance(value, tuple) else value,
                "type": type(value).__name__,
                "restart": name in RESTART_REQUIRED,
                "unused": name in UNUSED,
                "note": NOTES.get(name, ""),
            }
        )
    return fields


PAGE = """<!doctype html>
<meta charset="utf-8">
<title>Swarm variables</title>
<style>
  :root { color-scheme: light dark; }
  body { font: 14px/1.5 system-ui, sans-serif; margin: 2rem auto; max-width: 820px;
         padding: 0 1rem; }
  h1 { font-size: 1.25rem; margin-bottom: .25rem; }
  p.sub { margin-top: 0; opacity: .7; }
  table { border-collapse: collapse; width: 100%; }
  th, td { text-align: left; padding: .5rem .6rem; border-bottom: 1px solid #8884;
           vertical-align: top; }
  th { font-weight: 600; opacity: .8; }
  td.name { font-family: ui-monospace, monospace; white-space: nowrap; }
  td.note { opacity: .65; font-size: .85em; }
  input[type=text] { width: 11rem; font-family: ui-monospace, monospace;
                     padding: .25rem .4rem; }
  input:disabled { opacity: .5; }
  .tag { font-size: .72em; padding: .05rem .35rem; border-radius: .35rem;
         border: 1px solid #8886; margin-left: .35rem; white-space: nowrap; }
  .bar { position: sticky; bottom: 0; padding: .9rem 0; background: Canvas;
         border-top: 1px solid #8884; display: flex; gap: .75rem;
         align-items: center; }
  button { font: inherit; padding: .4rem 1rem; }
  #status { opacity: .75; }
  .changed input { outline: 2px solid #e9a; }
</style>
<h1>Swarm variables</h1>
<p class="sub">Edits apply to the running process on the next worker tick.</p>
<table><thead><tr><th>Field</th><th>Value</th><th>Notes</th></tr></thead>
<tbody id="rows"></tbody></table>
<div class="bar">
  <button id="save">Save changes</button>
  <button id="reload">Reload</button>
  <span id="status"></span>
</div>
<script>
let current = {};

function render(fields) {
  current = {};
  const rows = document.getElementById("rows");
  rows.innerHTML = "";
  for (const f of fields) {
    current[f.name] = f.value;
    const tr = document.createElement("tr");

    const name = document.createElement("td");
    name.className = "name";
    name.textContent = f.name;
    if (f.restart) name.appendChild(tag("restart to apply"));
    if (f.unused) name.appendChild(tag("not read by any code"));
    tr.appendChild(name);

    const cell = document.createElement("td");
    const input = document.createElement("input");
    input.type = "text";
    input.dataset.name = f.name;
    input.value = Array.isArray(f.value) ? JSON.stringify(f.value)
                                         : String(f.value);
    input.addEventListener("input", () => {
      const same = input.value === (Array.isArray(f.value)
        ? JSON.stringify(f.value) : String(f.value));
      tr.classList.toggle("changed", !same);
    });
    cell.appendChild(input);
    tr.appendChild(cell);

    const note = document.createElement("td");
    note.className = "note";
    tr.appendChild(note);

    rows.appendChild(tr);
  }
}

function tag(text) {
  const s = document.createElement("span");
  s.className = "tag";
  s.textContent = text;
  return s;
}

function say(text) { document.getElementById("status").textContent = text; }

async function load() {
  const r = await fetch("/api/vars");
  render(await r.json());
  say("loaded");
}

async function save() {
  const payload = {};
  for (const input of document.querySelectorAll("input[data-name]")) {
    if (input.disabled) continue;
    const name = input.dataset.name;
    const was = Array.isArray(current[name]) ? JSON.stringify(current[name])
                                             : String(current[name]);
    if (input.value !== was) payload[name] = input.value;
  }
  if (!Object.keys(payload).length) { say("nothing changed"); return; }
  const r = await fetch("/api/vars", {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify(payload),
  });
  const result = await r.json();
  if (result.errors && result.errors.length) {
    say("errors: " + result.errors.join("; "));
  } else {
    say("applied: " + result.applied.join(", "));
  }
  render(result.fields);
}

document.getElementById("save").addEventListener("click", save);
document.getElementById("reload").addEventListener("click", load);
load();
</script>
"""


class _Handler(BaseHTTPRequestHandler):
    variables = None  # set by ConfigServer

    def log_message(self, fmt, *args):
        """Silence the per-request stderr line -- it would bury the
        swarm's own output, which is the thing worth reading."""

    def _send(self, code, body, content_type):
        payload = body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self._send(200, PAGE, "text/html; charset=utf-8")
        elif self.path == "/api/vars":
            self._send(
                200,
                json.dumps(_describe(self.variables)),
                "application/json",
            )
        else:
            self._send(404, "not found", "text/plain")

    def do_POST(self):
        if self.path != "/api/vars":
            self._send(404, "not found", "text/plain")
            return
        length = int(self.headers.get("Content-Length") or 0)
        try:
            updates = json.loads(self.rfile.read(length) or "{}")
        except ValueError as exc:
            self._send(400, json.dumps({"errors": [str(exc)]}), "application/json")
            return

        applied, errors = [], []
        known = set(_public_fields(self.variables))
        for name, raw in updates.items():
            if name not in known:
                errors.append(f"{name}: no such variable")
                continue
            try:
                value = _coerce(getattr(self.variables, name), raw)
            except (TypeError, ValueError) as exc:
                errors.append(f"{name}: {exc}")
                continue
            setattr(self.variables, name, value)
            applied.append(name)
            print(f"config: {name} = {value!r}")
            if name in RESTART_REQUIRED:
                print(
                    f"config: NOTE '{name}' is only read at startup -- restart to apply"
                )

        if applied:
            # Persist just the fields this request touched, merged onto
            # whatever is already saved -- so it survives a restart via
            # Variables.load() -- without freezing untouched fields like
            # origin/vehicle_loiter_radius_min_m, which are meant to be
            # recomputed fresh each startup unless explicitly overridden.
            try:
                self.variables.save(fields=applied)
            except OSError as exc:
                errors.append(f"save to disk failed: {exc}")
        self._send(
            200,
            json.dumps(
                {
                    "applied": applied,
                    "errors": errors,
                    "fields": _describe(self.variables),
                }
            ),
            "application/json",
        )


class ConfigServer:
    """Serves the panel on a daemon thread; start() never blocks."""

    # NOT 6000: that is the X11 display port, and it sits on Chrome's and
    # Firefox's hard-coded unsafe-port blocklist -- the browser refuses to
    # connect (ERR_UNSAFE_PORT / "This address is restricted") even though
    # the server is serving fine, which looks exactly like it never started.
    def __init__(self, variables, host="127.0.0.1", port=8765):
        self.variables = variables
        self.host = host
        self.port = port
        self._httpd = None
        self._thread = None

    def start(self):
        handler = type("_BoundHandler", (_Handler,), {"variables": self.variables})
        try:
            self._httpd = ThreadingHTTPServer((self.host, self.port), handler)
        except OSError as exc:
            # A busy port must not take the swarm down with it.
            print(f"Config server not started ({self.host}:{self.port}): {exc}")
            return self
        self._httpd.daemon_threads = True
        self._thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)
        self._thread.start()
        print(f"Config panel: http://{self.host}:{self.port}/")
        return self

    def stop(self):
        if self._httpd is not None:
            self._httpd.shutdown()
            self._httpd.server_close()
            self._httpd = None
