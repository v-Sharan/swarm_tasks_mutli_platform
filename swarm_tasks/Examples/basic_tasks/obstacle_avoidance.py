"""
Standalone, visual demo of the obstacle + neighbour-bot avoidance added to
guided.py's line_waypoint_guidance(avoid_bots=True, ...) -- the same code
path swarm.py uses to fly real vehicles, with the SAME tuning swarm.py's
_guide_and_reposition() uses (capture_radius=50, avoid_weight=30,
avoid_min_sep=avoid_radius=100). Run directly:

    python3 swarm_tasks/Examples/basic_tasks/obstacle_avoidance.py

Two fixed-wing bots are given straight-line goals that would otherwise fly
each of them THROUGH a rectangular obstacle, and would also cross each
other's path partway through -- so one run shows both behaviours:
	- bot vs obstacle: each bot curves around its own box
	- bot vs bot: the two curve apart again where their lines cross
Progress prints every 10 ticks; the plot shows it live. Each bot loiters
(circles) its goal once it arrives -- a fixed-wing can't just stop -- so
the run ends with both bots visibly circling in place.

How it actually avoids the boxes: guided.py's _detour_point() checks
whether the straight line from the bot to its goal passes within
avoid_min_sep of any obstacle, and if so, hands back a point beyond one
of the obstacle's edges to steer at instead -- i.e. it moves the point
being tracked, not just a small nudge added on top of the old (blocked)
one. That's needed because _avoidance_offset's own push IS magnitude-
capped (2 x turn_radius, ~141m here): fine for nudging clear of another
bot, but a solid obstacle bigger than that budget can't be outflanked by
a bounded push alone -- the bot just gets shoved back toward the line
it's trying to hold, indefinitely, instead of ever getting past it.
"""
import os, sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))

import time
import matplotlib.pyplot as plt
from shapely.geometry import Polygon, Point

from swarm_tasks.simulation import simulation as sim
from swarm_tasks.simulation import visualizer as viz
from swarm_tasks.controllers.guided import line_waypoint_guidance

# Two toy obstacles, placed dead ahead of each bot's straight-line goal.
# To watch this fly around the REAL mission's fence instead, swap this for:
#     from utils import read_obstacles
#     OBSTACLES = read_obstacles()
# -- exactly what swarm.py itself loads sim.env.obstacles from at startup.
BOX_A = Polygon([[400, 480], [400, 520], [450, 520], [450, 480]])  # dead ahead of bot 0
BOX_B = Polygon([[685, 100], [685, 150], [735, 150], [735, 100]])  # dead ahead of bot 1
OBSTACLES = [BOX_A, BOX_B]

# Same avoidance tuning swarm.py's _guide_and_reposition() uses.
CAPTURE_RADIUS = 50.0
AVOID_WEIGHT = 30.0
AVOID_MIN_SEP = 100.0
STEP_SIZE = 0.3  # sim-seconds per tick -- see Bot.move()'s docstring

s = sim.Simulation(vehicle_type='fixedwing', world_size=(1000, 1000), bank_angle_deg=30)
s.env.obstacles = OBSTACLES

# Bot 0: straight line through BOX_A, then on through (700,500) where it
# crosses bot 1's path. Bot 1: straight line up through BOX_B, then on
# through that same crossing point.
s._populate(positions=[(10, 500, 0), (700, 10, 1.5708)], num_bots=2)
s.swarm[0].set_goal(990, 500)
s.swarm[1].set_goal(700, 990)

gui = viz.Gui(s)
gui.run()

print("Straight-line (no avoidance) paths would fly THROUGH both boxes and")
print("cross each other near (700,500) -- watch the plot: both bots curve")
print("around their own box, then curve apart again where the paths cross.\n")

tick = 0
while not all(b.done for b in s.swarm) and tick < 2000:
    tick += 1
    for b in s.swarm:
        cmd = line_waypoint_guidance(
            b,
            capture_radius=CAPTURE_RADIUS,
            freeze_on_arrival=True,
            loiter_radius=b.turn_radius(),  # circle the goal on arrival -- can't just stop
            avoid_bots=True,
            avoid_weight=AVOID_WEIGHT,
            avoid_min_sep=AVOID_MIN_SEP,
            avoid_radius=AVOID_MIN_SEP,
        )
        b.move(cmd.dir, cmd.speed, step_size=STEP_SIZE)

    gui.show_goals([b.goal for b in s.swarm])
    gui.update()
    time.sleep(0.01)

    if tick % 10 == 0:
        d_a = BOX_A.exterior.distance(Point(*s.swarm[0].get_position()))
        d_b = BOX_B.exterior.distance(Point(*s.swarm[1].get_position()))
        d_bots = s.swarm[0].dist(*s.swarm[1].get_position())
        print(f"tick {tick:4d}:  bot0-BOX_A edge={d_a:5.1f}m   "
              f"bot1-BOX_B edge={d_b:5.1f}m   bot0-bot1={d_bots:5.1f}m")

if all(b.done for b in s.swarm):
    print(f"\nDone after {tick} ticks -- both bots are now loitering at their goals.")
else:
    print(f"\nSafety cap ({tick} ticks) hit -- at least one bot never arrived.")
print("Close the plot window to exit.")
plt.show(block=True)
