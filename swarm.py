import os, sys

sys.path.insert(
    0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
)
from swarm_tasks.simulation import simulation as sim
from swarm_tasks.mission.bezier_curve import BezierCurveMultiple
from swarm_tasks.simulation import visualizer as viz
from swarm_tasks.controllers.guided import line_waypoint_guidance
from swarm_tasks.utils import waypoints as wp
from swarm_tasks.Thread import Thread
from time import sleep
import threading
import traceback
from swarm_tasks.utils.locatePosition import geoToCart, cartToGeo
from utils import compute_lookahead_target, send_reposition, compass_to_math_rad
from time import time
import random
from swarm_tasks.mission.groupsplitauto import AutoSplitMission
from swarm_tasks.mission.groupsplitspecific import SpecificSplitMission
from math import sin, cos


class Swarm:
    def __init__(self, num_bots, Variables, vehicle):
        self.num_bots = num_bots
        self.Variables = Variables
        self.s = sim.Simulation(
            vehicle_type="fixedwing",
            world_size=self.Variables.world_size,
            speed=self.Variables.speed,
            bank_angle_deg=self.Variables.bank_angle_deg,
        )
        self.vehicle = vehicle
        self.gui = viz.Gui(self.s)
        self.current_function = {
            "goal": [False, None],
            "search": [False, None],
            "spilt": [False, None],
        }
        self.isToStop = False
        # Goal jobs run CONCURRENTLY -- one per assignment, each owning a
        # set of bot ids and its own stop Event, so stopping the job for
        # bot 1 leaves the job for bot 2 flying. current_function["goal"]
        # mirrors this list so running_funcs()/stopAll() still work.
        self.goal_jobs = []
        # Self-healing: bots taken out of service by remove_bot(), the
        # work they had left over, and what each bot was originally
        # given (so "covered 5 of 8" can be reported).
        self.removed_bots = set()
        self.orphan_work = []  # FIFO of {"from": id, "points": [...]}
        self.assigned_seq = {}  # swarm index -> the list it was handed
        self._heal_lock = threading.Lock()
        # Workers run on background threads; matplotlib/Qt calls are not
        # thread-safe, so they only publish state here and the main
        # thread's gui_tick() does the actual drawing.
        #
        # Per-bot render state, keyed by bot index:
        #     index -> (goal, (rx, ry), (tx, ty))
        # A dict rather than a single-slot Queue of whole-swarm frames:
        # each concurrent job only knows about its own bots, so one
        # worker's full snapshot would blank out every bot the other
        # jobs own. Each merges only its own slice in, under the lock.
        self._gui_state = {}
        self._gui_lock = threading.Lock()
        self._gui_close_requested = False
        self.check_in_time = 0
        # Per-bot timestamp of the last send_reposition() call, so the
        # tick loop (runs every tick_interval_s, e.g. 0.1s) can still
        # gate the actual MAVLink command out to reposition_interval_s
        # instead of spamming one every tick.
        self._last_reposition_time = {}
        # Throttle for the debug line (Variables.debug_reposition).
        self._last_debug_time = {}
        self._last_idle_pump = 0.0

    def stopAction(self):
        """call_back_action handed to Thread.stop(): the list of things
        this Swarm owns that must wind down.

        SIGNAL ONLY -- it must not block. Thread.stop() runs this
        BEFORE setting its stop event, and Thread.run() re-invokes the
        target whenever it returns while that event is still clear. So
        waiting for the worker here would deadlock: the worker would
        exit, run() would restart it, and we'd never get to the
        set(). Setting isToStop breaks searchSub's own `while True`,
        the worker returns, Thread.stop() then sets the event, and the
        join() in current_process() actually completes.

        The real teardown (closing the GUI, clearing per-run state)
        happens in teardown(), called after that join -- see
        current_process() in utils.py.
        """
        self.isToStop = True

    def teardown(self):
        """Release everything the just-stopped function owned, so the
        next one starts from a clean slate.

        Must run on the MAIN thread and only AFTER the worker thread
        has been joined -- current_process() does both. Leaves the plot
        open but drops what it was drawing, clears the per-run timers,
        and marks every function slot as no-longer-running so
        running_func() reports idle.
        """
        # The plot stays OPEN and pumping after a stop, so the last
        # state can still be looked at -- only what it draws is cleared.
        # The next run's _reset_gui() closes and rebuilds it.

        # Drop stale render state, so it can't be drawn onto the next
        # run's fresh axes.
        with self._gui_lock:
            self._gui_state.clear()
        self.goal_jobs = []
        # NOT removed_bots: taking a bot out of the swarm is a
        # membership fact that has to outlive a stop, or the next
        # search would silently fly it again. add_bot() puts it back.
        self.orphan_work = []
        self.assigned_seq.clear()
        # Cancel every bot's leftover goals -- the run they belonged to
        # is over. Without this a remove_bot() AFTER a stop harvests the
        # dead run's queue into orphan_work and the next run flies it;
        # that is where "covered 0/0 points, 14 left" came from.
        for bot in self.s.swarm:
            bot.cancel_goal()

        self.isToStop = False
        self._gui_close_requested = False
        self._last_reposition_time.clear()
        self._last_debug_time.clear()
        self.check_in_time = 0

        for key in self.current_function:
            self.current_function[key] = [False, None]

        print("Teardown complete: state reset, plot left open")

    def _reset_gui(self):
        """Close any existing plot and build a fresh Gui bound to the
        current simulation. A Gui caches per-run drawing state (trails,
        goal/gps/lookahead artist lists, coverage text) and its figure
        may already be closed, so reusing one across functions leaves
        the previous run's elements on the new axes -- rebuild instead.
        """
        if not self.Variables.gui: return
        if self.gui is not None:
            self.gui.close()
        self.gui = viz.Gui(self.s)

    def running_func(self):
        return next(
            ((k, v) for k, v in self.current_function.items() if v[0] is True), None
        )

    def running_funcs(self):
        """Every currently running function, not just the first one."""
        return [(k, v) for k, v in self.current_function.items() if v[0] is True]

    @staticmethod
    def _threads_of(slot):
        """A slot holds one Thread (search) or a list of them (goal) --
        normalise both to a list so callers don't have to care."""
        thread = slot[1]
        if thread is None:
            return []
        if isinstance(thread, (list, tuple)):
            return [t for t in thread if t is not None]
        return [thread]

    def _sync_goal_slot(self):
        """Mirror goal_jobs into current_function["goal"], so
        running_funcs(), stopAll() and current_process() keep working
        without knowing anything about per-bot jobs."""
        threads = [job["thread"] for job in self.goal_jobs]
        self.current_function["goal"] = [bool(threads), threads]

    def _assign(self, index, points):
        """Give a bot a goal list and remember it, so remove_bot() can
        say how much of it was covered before the bot dropped out."""
        points = [tuple(p) for p in points]
        self.s.swarm[index].set_goal_sequence(points)
        self.assigned_seq[index] = points

    @staticmethod
    def _remaining_points(bot, was_done=None):
        """What this bot still has to fly: the goal it is on (if it has
        not captured it yet) plus everything still queued behind it.

        was_done is the bot's done flag from BEFORE remove_bot() froze
        it -- reading bot.done here instead would drop the goal it was
        actively flying towards."""
        if was_done is None:
            was_done = bot.done
        points = []
        if bot.goal_exists() and not was_done and bot.goal is not None:
            points.append(tuple(bot.goal))
        points.extend(tuple(p) for p in bot.goal_queue)
        return points

    def remove_bot(self, bot_id, hand_over=True):
        """Take a bot out of service and leave its unflown points for
        whichever bot finishes its own work first.

        This is the self-heal entry point: pull bot 2 out after it has
        covered 5 of its 8 points and the remaining 3 go into
        orphan_work, to be claimed by the next bot that runs out of its
        own goals (see _claim_orphan_work). The removed bot is frozen
        and skipped by every worker loop until add_bot() brings it back.
        """
        bot_id = int(bot_id)
        index = bot_id - 1
        if not 0 <= index < len(self.s.swarm):
            print(f"remove_bot: no bot {bot_id}")
            return []
        bot = self.s.swarm[index]

        # Freeze BEFORE snapshotting the queue: _guide_and_reposition()
        # checks `if not b.done` before moving, so once this is set the
        # worker can no longer advance the queue out from under us and
        # make a point be handed over twice, or dropped.
        was_done = bot.done
        bot.done = True

        with self._heal_lock:
            if bot_id in self.removed_bots:
                print(f"Bot {bot_id} is already removed")
                bot.done = was_done
                return []
            self.removed_bots.add(bot_id)
            remaining = self._remaining_points(bot, was_done)
            assigned = self.assigned_seq.get(index, [])
            covered = max(0, len(assigned) - len(remaining))
            if hand_over and remaining:
                self.orphan_work.append({"from": bot_id, "points": remaining})

        # Drop its markers -- no worker commands it now.
        bot.clear_goal_queue()
        self._clear_state([index])
        print(
            f"Bot {bot_id} removed: covered {covered}/{len(assigned)} points, "
            f"{len(remaining)} left for the next free bot"
        )
        if not self.running_funcs():
            print(
                "  (nothing is running -- this only takes it out of the "
                "next search/goal; the plot has no live state to redraw)"
            )
        return remaining

    def add_bot(self, bot_id):
        """Put a removed bot back into service. It comes back idle, so
        the next tick of whichever job owns it will let it claim any
        orphaned work that is still waiting."""
        bot_id = int(bot_id)
        index = bot_id - 1
        if not 0 <= index < len(self.s.swarm):
            print(f"add_bot: no bot {bot_id}")
            return False
        with self._heal_lock:
            if bot_id not in self.removed_bots:
                print(f"Bot {bot_id} is already in the swarm")
                return False
            self.removed_bots.discard(bot_id)
        self.s.swarm[index].done = True  # idle => eligible to claim work
        print(f"Bot {bot_id} added back to the swarm")
        return True

    def _claim_orphan_work(self, index):
        """A bot that just ran out of goals takes over the oldest bundle
        left behind by a removed bot. Returns True if it picked one up.

        Called from the worker loops in place of a bare `if b.done:
        continue`, so the FIRST bot to finish gets the work -- which is
        exactly the hand-over rule: whoever is free first flies it.
        """
        with self._heal_lock:
            if not self.orphan_work or (index + 1) in self.removed_bots:
                return False
            work = self.orphan_work.pop(0)
        self._assign(index, work["points"])
        print(
            f"Self-heal: bot {index+1} took over {len(work['points'])} "
            f"points left by bot {work['from']}"
        )
        return True

    def _owned_indices(self):
        """Swarm indices currently commanded by SOME goal job."""
        return {i - 1 for job in self.goal_jobs for i in job["ids"]}

    def _publish_state(self, updates):
        """Merge one worker's slice of render state into the shared dict."""
        with self._gui_lock:
            self._gui_state.update(updates)

    def _clear_state(self, indices):
        """Forget these bots' render state, so a stopped job's markers
        don't sit frozen on the plot while other jobs keep drawing."""
        with self._gui_lock:
            for i in indices:
                self._gui_state.pop(i, None)

    def stop_goal_jobs(self, ids=None, timeout=5.0):
        """Stop the goal jobs owning any of `ids` (all of them if ids is
        None) and leave every other job running.

        The per-bot counterpart of stopAll(): re-assigning bots 1 and 2
        must stop whatever those two were doing without disturbing the
        job flying bot 3. Deliberately does NOT call teardown() -- the
        plot and the surviving jobs' state have to outlive this.
        """
        wanted = None if ids is None else set(ids)
        targets = [
            job for job in self.goal_jobs if wanted is None or (job["ids"] & wanted)
        ]
        if not targets:
            return []

        # Signal them all before joining any, so they wind down
        # concurrently instead of each waiting out the previous tick.
        for job in targets:
            job["stop"].set()
        for job in targets:
            # stop() also sets the Thread's own event, or Thread.run()
            # would just re-invoke the worker after it returned.
            job["thread"].stop()
            job["thread"].join(timeout=timeout)
            if job["thread"].is_alive():
                print(
                    f"WARNING: goal job {sorted(job['ids'])} did not stop "
                    f"within {timeout}s"
                )

        dead = {id(job) for job in targets}
        self.goal_jobs = [j for j in self.goal_jobs if id(j) not in dead]
        self._sync_goal_slot()
        stopped = [sorted(job["ids"]) for job in targets]
        print(f"Stopped goal jobs: {stopped}")
        return stopped

    def stopAll(self, call_back_action=None, timeout=5.0):
        """Stop every running function and tear down once they're all gone.

        Signals ALL the workers before joining any of them, so they wind
        down concurrently rather than each waiting out the previous
        one's tick. They share the isToStop flag, which is why no worker
        may clear it itself -- teardown() does that at the end, after
        every one of them has exited.

        Returns the names of the functions that were stopped.
        """
        if call_back_action is None:
            call_back_action = self.stopAction

        running = self.running_funcs()
        if not running:
            print("No running function to stop")
            return []

        # Each goal job's OWN event too: the shared isToStop that
        # call_back_action sets is not the only flag they watch.
        for job in self.goal_jobs:
            job["stop"].set()

        for name, slot in running:
            for thread in self._threads_of(slot):
                print(f"Stopping: {name}")
                thread.stop(call_back_action)

        stopped = []
        for name, slot in running:
            alive = False
            for thread in self._threads_of(slot):
                thread.join(timeout=timeout)
                if thread.is_alive():
                    alive = True
            if alive:
                # Daemon threads, so this can't block interpreter exit,
                # but the state it owns can't be safely cleared either.
                print(f"WARNING: '{name}' did not stop within {timeout}s")
            else:
                stopped.append(name)

        # Must run: it clears isToStop (nothing else does), empties
        # goal_jobs and the current_function slots, and closes the plot.
        # Without it a stop leaves isToStop True and dead jobs in the
        # list, so the NEXT goal skips its `if not self.goal_jobs`
        # reset and its worker breaks on the first tick.
        self.teardown()
        print(f"Stopped all: {stopped}")
        return stopped

    def gui_tick(self):
        """Call repeatedly from the main thread to drive the plot."""
        if not self.Variables.gui: return
        if self.gui is None:
            return
        if self._gui_close_requested:
            self.gui.close()
            self.gui = None
            self._gui_close_requested = False
            return
        with self._gui_lock:
            state = list(self._gui_state.items())
        if not state:
            # Nothing running, but keep pumping: update() ends in
            # plt.pause(), and without it the window stops redrawing
            # and the desktop greys it out as unresponsive. Throttled
            # to ~20Hz -- main.py calls this in a tight spin loop, and
            # a full redraw per iteration pegs a core for nothing.
            now = time()
            if now - self._last_idle_pump >= 0.05:
                self._last_idle_pump = now
                self.gui.update()
            return
        # Rebuild the whole-swarm view that every concurrent job fed into.
        num_bots = len(self.s.swarm)
        goals = []
        drone_positions = [None] * num_bots
        lookahead_targets = [None] * num_bots
        for i, (goal, position, lookahead) in state:
            if i >= num_bots:
                continue
            if goal is not None:
                goals.append(goal)
            drone_positions[i] = position
            lookahead_targets[i] = lookahead
        self.gui.show_goals(goals)
        self.gui.show_gps_positions(drone_positions)
        self.gui.show_lookahead_targets(lookahead_targets)
        self.gui.update()

    def _guide_and_reposition(self, i, b, x, y, theta, rx, ry):
        """One tick of guidance for a single bot: advance it along its
        line, compute the lookahead target for the real vehicle, and
        optionally push that target to the vehicle via send_reposition.

        Always advances the bot -- it's the path planner and must keep
        moving every tick regardless of vehicle proximity -- and pushes
        the target to the vehicle on the reposition_interval_s clock.
        Returns the computed (tx, ty).

        x, y: bot's pose at the START of this tick (captured by the
        caller before calling this, so the lookahead line is anchored
        to where the bot actually was this tick, not post-move).
        """
        dis = b.dist(rx, ry)
        along = (rx - x) * cos(theta) + (ry - y) * sin(theta)
        lead_state = True if along > 0 else False
        if dis > 100 and lead_state:
            b.x, b.y = rx, ry
            return None, None
        cmd = line_waypoint_guidance(
            b,
            capture_radius=50.0,
            freeze_on_arrival=not b.done,
            avoid_bots=True,
            avoid_weight=30.0,
            avoid_min_sep=100.0,
            avoid_radius=100.0,
        )
        step_size = (
            (self.Variables.bot_target_speed_mps * self.Variables.tick_interval_s)
            / 2.0
            / b.max_speed
        )
        # step_size = step_size if not send_command else step_size * 2
        b.move(cmd.dir, cmd.speed, step_size=step_size)

        # cmd.dir -- the bearing guidance just computed for THIS tick --
        # not the bot's pre-move theta, which is still the OLD leg's
        # bearing right when the bot rotates onto a new line segment.
        # Anchoring the lookahead projection to a stale heading is what
        # was placing tx,ty on the wrong side of the turn and forcing
        # the real vehicle into a sudden small-radius intercept turn.

        tx, ty = compute_lookahead_target(
            x,
            y,
            theta,
            rx,
            ry,
            self.Variables.vehicle_loiter_radius_min_m,
        )
        lead_lat, lead_lon = cartToGeo(
            self.Variables.origin,
            self.Variables.endDistance,
            [tx, ty],
        )

        now = time()
        last_sent = self._last_reposition_time.get(i, 0)
        if now - last_sent >= self.Variables.reposition_interval_s:
            send_reposition(
                self.vehicle.drones[i],
                lead_lat,
                lead_lon,
                self.Variables.heights[i],
                loiter_radius=0.0,
            )
            self._last_reposition_time[i] = now

            if getattr(self.Variables, "debug_reposition", False):
                if now - self._last_debug_time.get(i, 0) >= 2.0:
                    self._last_debug_time[i] = now
                    frame = self.vehicle.drones[i].location.global_relative_frame
                    print(
                        f"bot {i+1}: cmd ({lead_lat:.6f}, {lead_lon:.6f}) "
                        f"| drone ({frame.lat:.6f}, {frame.lon:.6f}) alt {frame.alt:.0f} "
                        f"| bot->drone {b.dist(rx, ry) * 2:.0f}m "
                        f"| lead {self.Variables.vehicle_loiter_radius_min_m:.0f}m"
                    )

        return tx, ty

    def search(
        self, center_lat, center_lon, effective_num_uavs, grid_space, coverage_area
    ):
        # Removed bots must not be planned for: the grid is split
        # between the ACTIVE bots only. Handing a removed bot a share of
        # the coverage area would leave that strip unflown -- a hole in
        # the search, and not one self-healing can fill, because those
        # points never belonged to a bot that dropped out mid-run.
        active = [
            i
            for i in range(len(self.vehicle.drones))
            if (i + 1) not in self.removed_bots
        ]
        if not active:
            print("search: every bot has been removed -- nothing to fly")
            return
        if len(active) != int(effective_num_uavs):
            print(
                f"search: planning for {len(active)} active bot(s) "
                f"{[i + 1 for i in active]} -- bot(s) "
                f"{sorted(self.removed_bots)} removed"
            )
        effective_num_uavs = len(active)

        pos = self.vehicle.get_positions(
            geoToCart=geoToCart,
            origin=self.Variables.origin,
            endDistance=self.Variables.endDistance,
        )
        self.s._populate(positions=pos, num_bots=len(self.vehicle.drones))
        # _populate() rebuilds every bot unfrozen; re-freeze the removed
        # ones so they don't read as active on the plot.
        for i in range(len(self.s.swarm)):
            if (i + 1) in self.removed_bots:
                self.s.swarm[i].done = True
        # Clear the shared stop signal before spawning a worker, or a
        # leftover True from a previous stop would break it instantly.
        self.isToStop = False
        # Fresh plot for this run -- teardown() closes the previous
        # one, and a re-run must not draw onto a closed figure.
        self._reset_gui()
        self.gui.show_env()
        self.gui.show_bots()
        curve = BezierCurveMultiple(
            self.Variables.origin,
            float(center_lat),
            float(center_lon),
            effective_num_uavs,
            int(grid_space),
            int(coverage_area),
            self.Variables.run_id,
        )

        curve.GridFormation()
        curve.generate_bezier_curve()
        goal_sequences = wp.load_goals_for_swarm(curve.filePaths)

        colors = ["red", "blue", "green", "orange", "purple"]
        for i, seq in enumerate(goal_sequences):
            xs, ys = zip(*seq)
            self.gui.ax.plot(
                xs,
                ys,
                "x--",
                color=colors[i % len(colors)],
                markersize=8,
                mew=2,
                alpha=0.6,
            )

        # zip against `active`, so sequence 0 goes to the first bot
        # still in the swarm rather than to bot 1 by position.
        for index, seq in zip(active, goal_sequences):
            self._assign(index, seq)

        self.gui.run()

        def searchSub():
            self.check_in_time = time()
            while True:
                # Break, but do NOT clear isToStop -- it's the shared
                # signal every running function watches. Clearing it
                # here would stop only whichever worker noticed first
                # and leave the rest running. teardown() clears it,
                # once all of them have exited.
                if self.isToStop:
                    print("Stoped search")
                    break
                sleep(self.Variables.tick_interval_s)
                updates = {}
                try:
                    for i, b in enumerate(self.s.swarm):
                        if (i + 1) in self.removed_bots:
                            continue
                        if (
                            b.done
                            and not self._claim_orphan_work(i)
                            and not b.loitering
                        ):
                            continue
                        v_lat, v_lon = (
                            self.vehicle.drones[i].location.global_relative_frame.lat,
                            self.vehicle.drones[i].location.global_relative_frame.lon,
                        )
                        vx, vy = geoToCart(
                            self.Variables.origin,
                            self.Variables.endDistance,
                            [v_lat, v_lon],
                        )
                        # rx, ry = (
                        #     vx / 2,
                        #     vy / 2,
                        # )  # real vehicle's position in the bot's sim frame

                        # Bot advances its own line every tick regardless of vehicle
                        # proximity -- it's the path planner here -- but arrival_gate
                        # stops it handing off to the NEXT queued waypoint until the
                        # real vehicle has also gotten near the current one, so sim
                        # kinematics (much faster than real flight) can't run the
                        # bot far ahead of where the vehicle actually is.
                        x, y, theta = b.get_pose()

                        tx, ty = self._guide_and_reposition(i, b, x, y, theta, vx, vy)
                        updates[i] = (
                            b.goal if b.goal_exists() else None,
                            (vx, vy),
                            (tx, ty),
                        )
                except Exception:
                    # Thread.run() has no handler, so an exception here
                    # would kill this job outright and the only sign
                    # would be drones that quietly stop being commanded.
                    print("ERROR in worker tick:")
                    traceback.print_exc()
                self._publish_state(updates)

        thread = Thread(target=searchSub)
        thread.start()

        self.current_function["search"] = [True, thread]

    def goal(self, guided_circle_direction, guided_circle_radius, goal_latlon, ids):
        goal_latlon = [
            [x, y]
            for x, y in [
                geoToCart(self.Variables.origin, self.Variables.endDistance, geo)
                for geo in goal_latlon
            ]
        ]
        print(goal_latlon)

        ids = [int(i) for i in ids]

        # A new assignment replaces what THESE bots were doing and
        # nothing else: stop only the jobs that own one of them, and
        # leave every other goal job flying. search() drives every bot
        # in the swarm, so it can't coexist with a per-bot goal at all.
        if self.current_function["search"][0]:
            self.stopAll()
        self.stop_goal_jobs(ids)

        dropped = [i for i in ids if i in self.removed_bots]
        if dropped:
            print(f"goal: ignoring removed bot(s) {dropped} -- add_ them back first")
            ids = [i for i in ids if i not in self.removed_bots]
        if not ids:
            print("goal: no active bots in this command")
            return

        owned = [i - 1 for i in ids]  # bot id N is swarm index N-1

        pos = self.vehicle.get_positions(
            geoToCart=geoToCart,
            origin=self.Variables.origin,
            endDistance=self.Variables.endDistance,
        )

        if len(self.s.swarm) != len(self.vehicle.drones):
            # First goal of the session: the swarm doesn't exist yet.
            # Only here, because _populate() DESTROYS and re-creates
            # every bot -- doing that on each command would wipe the
            # goal sequence and pose of the bots other jobs are flying.
            self.s._populate(positions=pos, num_bots=len(self.vehicle.drones))

        # Snap just THIS command's bots onto their drone's live x,y (and
        # heading) every time, so a new leg always starts from where the
        # aircraft actually is -- a bot that sat idle, or froze at its
        # last goal, would otherwise set off from a stale sim position.
        # Per-bot, so the bots other jobs are flying are left alone.
        for i in owned:
            if 0 <= i < len(self.s.swarm) and i < len(pos):
                bot = self.s.swarm[i]
                bot.x, bot.y, bot.theta = pos[i]
                print(
                    f"Bot {i+1} placed at drone position " f"({bot.x:.1f}, {bot.y:.1f})"
                )

        if not self.goal_jobs:
            # Nothing else is flying, so the plot can be rebuilt. With a
            # job already running, _reset_gui() would drop its artists.
            # Clear the shared stop signal too, or a leftover True from
            # a previous stop would break the new worker instantly.
            self.isToStop = False
            self._reset_gui()
            self.gui.show_env()
            self.gui.show_bots()

        colors = ["red", "blue", "green", "orange", "purple"]
        # for i, seq in enumerate(goal_latlon):

        xs, ys = zip(*goal_latlon)
        self.gui.ax.plot(
            xs,
            ys,
            "x--",
            color=colors[random.randint(0, 5) % len(colors)],
            markersize=8,
            mew=2,
            alpha=0.6,
        )

        self.gui.run()

        def goalSub(goal_latlon, ids, stop_event):
            for i in owned:
                if not 0 <= i < len(self.s.swarm):
                    continue
                print(f"Goal set to bot: {i+1}")
                self._assign(i, goal_latlon)

            while True:
                # This job's OWN event, so stopping the goal for one set
                # of bots leaves the others running. isToStop is still
                # honoured: a full stop has to take everything.
                if stop_event.is_set() or self.isToStop:
                    print(f"Goal for bots {ids} stopped")
                    self._clear_state(owned)
                    break
                sleep(self.Variables.tick_interval_s)
                updates = {}
                try:
                    # Bots some OTHER job is flying: hands off. Snapping
                    # them to their raw drone position below would drag
                    # them back every tick and fight that job's guidance.
                    busy = self._owned_indices()
                    for i, b in enumerate(self.s.swarm):
                        v_lat, v_lon = (
                            self.vehicle.drones[i].location.global_relative_frame.lat,
                            self.vehicle.drones[i].location.global_relative_frame.lon,
                        )
                        vx, vy = geoToCart(
                            self.Variables.origin,
                            self.Variables.endDistance,
                            [v_lat, v_lon],
                        )
                        # rx, ry = (
                        #     vx / 2,
                        #     vy / 2,
                        # )
                        if not (i + 1) in ids:
                            # Not ours. If nothing else is flying it either,
                            # park its marker on the drone's live position so
                            # the plot still tracks that aircraft.
                            if i not in busy:
                                b.x, b.y, b.theta = (
                                    vx,
                                    vy,
                                    compass_to_math_rad(self.vehicle.drones[i].heading),
                                )
                            continue
                        if (i + 1) in self.removed_bots:
                            continue
                        if (
                            b.done
                            and not self._claim_orphan_work(i)
                            and not b.loitering
                        ):
                            if b.done:
                                print(f"Bot: {i+1} is completed goal")
                            continue

                        # Bot advances its own line every tick regardless of vehicle
                        # proximity -- it's the path planner here -- but arrival_gate
                        # stops it handing off to the NEXT queued waypoint until the
                        # real vehicle has also gotten near the current one, so sim
                        # kinematics (much faster than real flight) can't run the
                        # bot far ahead of where the vehicle actually is.
                        x, y, theta = b.get_pose()

                        tx, ty = self._guide_and_reposition(i, b, x, y, theta, vx, vy)
                        updates[i] = (
                            b.goal if b.goal_exists() else None,
                            (vx, vy),
                            (tx, ty),
                        )
                except Exception:
                    # Thread.run() has no handler, so an exception here
                    # would kill this job outright and the only sign
                    # would be drones that quietly stop being commanded.
                    print("ERROR in worker tick:")
                    traceback.print_exc()
                self._publish_state(updates)

        stop_event = threading.Event()
        thread = Thread(target=goalSub, args=(goal_latlon, ids, stop_event))
        thread.start()

        self.goal_jobs.append({"ids": set(ids), "thread": thread, "stop": stop_event})
        self._sync_goal_slot()
        print(
            f"Goal thread started for bots {ids} -- running jobs: "
            f"{[sorted(job['ids']) for job in self.goal_jobs]}"
        )

    def AutosplitMission(
        self, selected_uav_ids, grid_space, coverage_area, center_lat_lon_array
    ):
        print("Auto Split Mission")
        active = [
            i
            for i in range(len(self.vehicle.drones))
            if (i + 1) not in self.removed_bots
        ]
        if not active:
            print("Auto Split Mission: every bot has been removed -- nothing to fly")
            return

        if len(active) != len(selected_uav_ids):
            print(
                f"Auto Split Mission: planning for {len(active)} active bot(s) "
                f"{[i + 1 for i in active]} -- bot(s) "
                f"{sorted(self.removed_bots)} removed"
            )

        pos = self.vehicle.get_positions(
            geoToCart=geoToCart,
            origin=self.Variables.origin,
            endDistance=self.Variables.endDistance,
        )
        self.s._populate(positions=pos, num_bots=len(self.vehicle.drones))
        # _populate() rebuilds every bot unfrozen; re-freeze the removed
        # ones so they don't read as active on the plot.
        for i in range(len(self.s.swarm)):
            if (i + 1) in self.removed_bots:
                self.s.swarm[i].done = True

        split = AutoSplitMission(
            origin=self.Variables.origin,
            center_lat_lons=center_lat_lon_array,
            drone_list=selected_uav_ids,
            grid_spacing=int(grid_space),
            coverage_area=int(coverage_area),
            run_id=self.Variables.run_id,
        )
        split.GroupSplitting(
            center_lat_lons=center_lat_lon_array,
            num_of_drones=len(selected_uav_ids),
            grid_spacing=int(grid_space),
            coverage_area=int(coverage_area),
        )
        print(split.filePath)
        goal_sequences = wp.load_goals_for_swarm(split.filePath)
        colors = ["red", "blue", "green", "orange", "purple"]
        for i, seq in enumerate(goal_sequences):
            xs, ys = zip(*seq)
            self.gui.ax.plot(
                xs,
                ys,
                "x--",
                color=colors[i % len(colors)],
                markersize=8,
                mew=2,
                alpha=0.6,
            )

        # zip against `active`, so sequence 0 goes to the first bot
        # still in the swarm rather than to bot 1 by position.
        for index, seq in zip(active, goal_sequences):
            self._assign(index, seq)

        self.gui.run()

        def AutoSplitMissionSub():
            while True:
                # Break, but do NOT clear isToStop -- it's the shared
                # signal every running function watches. Clearing it
                # here would stop only whichever worker noticed first
                # and leave the rest running. teardown() clears it,
                # once all of them have exited.
                if self.isToStop:
                    print("Stopped Auto Mission")
                    break
                sleep(self.Variables.tick_interval_s)
                updates = {}
                try:
                    for i, b in enumerate(self.s.swarm):
                        if (i + 1) in self.removed_bots:
                            continue
                        if (
                            b.done
                            and not self._claim_orphan_work(i)
                            and not b.loitering
                        ):
                            continue
                        v_lat, v_lon = (
                            self.vehicle.drones[i].location.global_relative_frame.lat,
                            self.vehicle.drones[i].location.global_relative_frame.lon,
                        )
                        vx, vy = geoToCart(
                            self.Variables.origin,
                            self.Variables.endDistance,
                            [v_lat, v_lon],
                        )
                        # rx, ry = (
                        #     vx / 2,
                        #     vy / 2,
                        # )  # real vehicle's position in the bot's sim frame

                        # Bot advances its own line every tick regardless of vehicle
                        # proximity -- it's the path planner here -- but arrival_gate
                        # stops it handing off to the NEXT queued waypoint until the
                        # real vehicle has also gotten near the current one, so sim
                        # kinematics (much faster than real flight) can't run the
                        # bot far ahead of where the vehicle actually is.
                        x, y, theta = b.get_pose()

                        tx, ty = self._guide_and_reposition(i, b, x, y, theta, vx, vy)
                        updates[i] = (
                            b.goal if b.goal_exists() else None,
                            (vx, vy),
                            (tx, ty),
                        )
                except Exception:
                    # Thread.run() has no handler, so an exception here
                    # would kill this job outright and the only sign
                    # would be drones that quietly stop being commanded.
                    print("ERROR in worker tick:")
                    traceback.print_exc()
                self._publish_state(updates)

        thread = Thread(target=AutoSplitMissionSub)
        thread.start()

        self.current_function["spilt"] = [True, thread]

    def SpecificSplit(self, center_lat_lon_array, uav_array, grid_space, coverage_area):
        print("Specific Split Mission")
        active = [
            i
            for i in range(len(self.vehicle.drones))
            if (i + 1) not in self.removed_bots
        ]
        if not active:
            print(
                "Specific Split Mission: every bot has been removed -- nothing to fly"
            )
            return

        if len(active) != len(uav_array):
            print(
                f"Specific Split Mission: planning for {len(active)} active bot(s) "
                f"{[i + 1 for i in active]} -- bot(s) "
                f"{sorted(self.removed_bots)} removed"
            )

        pos = self.vehicle.get_positions(
            geoToCart=geoToCart,
            origin=self.Variables.origin,
            endDistance=self.Variables.endDistance,
        )
        self.s._populate(positions=pos, num_bots=len(self.vehicle.drones))

        for i in range(len(self.s.swarm)):
            if (i + 1) in self.removed_bots:
                self.s.swarm[i].done = True

        split = SpecificSplitMission(
            origin=self.Variables.origin,
            center_lat_lons=center_lat_lon_array,
            drone_array=uav_array,
            grid_spacing=grid_space,
            coverage_area=coverage_area,
            run_id=self.Variables.run_id,
        )
        split.GroupSplitting(
            center_lat_lons=center_lat_lon_array,
            drone_array=uav_array,
            grid_spacing=grid_space,
            coverage_area=coverage_area,
        )

        goal_sequences = wp.load_goals_for_swarm(split.filePaths)
        colors = ["red", "blue", "green", "orange", "purple"]
        for i, seq in enumerate(goal_sequences):
            xs, ys = zip(*seq)
            self.gui.ax.plot(
                xs,
                ys,
                "x--",
                color=colors[i % len(colors)],
                markersize=8,
                mew=2,
                alpha=0.6,
            )

        # zip against `active`, so sequence 0 goes to the first bot
        # still in the swarm rather than to bot 1 by position.
        for index, seq in zip(active, goal_sequences):
            self._assign(index, seq)

        self.gui.run()

        def SpecificSplitSub():
            while True:
                if self.isToStop:
                    print("Stopped Auto Mission")
                    break
                sleep(self.Variables.tick_interval_s)
                updates = {}
                try:
                    for i, b in enumerate(self.s.swarm):
                        if (i + 1) in self.removed_bots:
                            continue
                        if (
                            b.done
                            and not self._claim_orphan_work(i)
                            and not b.loitering
                        ):
                            continue
                        v_lat, v_lon = (
                            self.vehicle.drones[i].location.global_relative_frame.lat,
                            self.vehicle.drones[i].location.global_relative_frame.lon,
                        )
                        vx, vy = geoToCart(
                            self.Variables.origin,
                            self.Variables.endDistance,
                            [v_lat, v_lon],
                        )
                        # rx, ry = (
                        #     vx / 2,
                        #     vy / 2,
                        # )  # real vehicle's position in the bot's sim frame

                        # Bot advances its own line every tick regardless of vehicle
                        # proximity -- it's the path planner here -- but arrival_gate
                        # stops it handing off to the NEXT queued waypoint until the
                        # real vehicle has also gotten near the current one, so sim
                        # kinematics (much faster than real flight) can't run the
                        # bot far ahead of where the vehicle actually is.
                        x, y, theta = b.get_pose()

                        tx, ty = self._guide_and_reposition(i, b, x, y, theta, vx, vy)
                        updates[i] = (
                            b.goal if b.goal_exists() else None,
                            (vx, vy),
                            (tx, ty),
                        )
                except Exception:
                    # Thread.run() has no handler, so an exception here
                    # would kill this job outright and the only sign
                    # would be drones that quietly stop being commanded.
                    print("ERROR in worker tick:")
                    traceback.print_exc()
                self._publish_state(updates)

        thread = Thread(target=SpecificSplitSub)
        thread.start()

        self.current_function["spilt"] = [True, thread]
