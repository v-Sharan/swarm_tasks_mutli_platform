import json
from swarm import Swarm
from variable import Variables
from swarm_tasks.network.UDP import UDPReceiver, CallBack
from vehicle import Vehicles
from utils import current_process,generate_heights
from swarm_tasks.config_server import ConfigServer
from swarm_tasks.network.network import NetworkIP

if __name__ == "__main__":
    callBack = CallBack()
    print("Callback")
    ip = NetworkIP.get_ip()
    print(f"Ip address: {ip}")
    vehicles = Vehicles(
        [
            f"udpin:{ip}:14551",
            f"udpin:{ip}:14552",
            f"udpin:{ip}:14553",
            # f"udpin:{ip}:14554",
            # f"udpin:{ip}:14555",
            # f"udpin:{ip}:14556",
        ]
    )
    receiver = UDPReceiver((ip, 12008), callback=callBack.call_back)
    receiver.start()
    vehicle_loiter_radius_min_m = vehicles.Loiter_param()
    variables = Variables(vehicle_loiter_radius_min_m=vehicle_loiter_radius_min_m)
    variables.heights = [100 + i * 10 for i in range(len(vehicles.drones))]
    # Live-editable config panel. Same object Swarm holds, so an edit
    # lands on the workers' next tick -- no restart.
    ConfigServer(variables).start()
    swarm = Swarm(num_bots=len(vehicles.drones), Variables=variables, vehicle=vehicles)
    while True:
        # Matplotlib/Qt calls must happen on the main thread; searchSub()
        # (background thread) only publishes state for this to draw.
        swarm.gui_tick()

        data = callBack.get_current_data()

        if data is None:
            continue

        elif data.startswith("stop"):
            current_process(swarm, stop=True, call_back_action=swarm.stopAction)
            callBack.set_data()
            continue

        elif data.startswith("search"):
            current_process(swarm, stop=True, call_back_action=swarm.stopAction)
            print("decoded_index", data)
            msg_parts = data.split(",", 6)
            selected_uav_raw = msg_parts[6] if len(msg_parts) > 6 else None
            _, center_lat, center_lon, num_uavs, grid_space, coverage_area = msg_parts[
                :6
            ]
            swarm.search(
                center_lat=center_lat,
                center_lon=center_lon,
                effective_num_uavs=len(vehicles.drones),
                grid_space=grid_space,
                coverage_area=coverage_area,
            )
            callBack.set_data()

        elif data.startswith("goal"):
            print(f"Goal message received: {data}")
            # No stop here: goal() stops only the jobs owning the bots
            # in this message, so a goal for bot 2 leaves bot 1 flying.
            msg_parts = data.split("_")
            swarm.goal(
                guided_circle_direction=msg_parts[2],
                guided_circle_radius=msg_parts[3],
                goal_latlon=json.loads(msg_parts[1]),
                ids=json.loads(msg_parts[4]),
            )
            callBack.set_data()

        elif data.startswith("remove") or data.startswith("add"):
            # Self-heal control, e.g.  remove_["02"]   or   add_2
            # A removed bot is frozen and its unflown points go to
            # whichever bot finishes its own work first.
            action, _, raw = data.partition("_")
            try:
                bot_ids = json.loads(raw)
            except ValueError:
                bot_ids = [p for p in raw.replace(",", " ").split() if p]
            if not isinstance(bot_ids, list):
                bot_ids = [bot_ids]
            for bot_id in bot_ids:
                if action == "remove":
                    swarm.remove_bot(bot_id)
                else:
                    swarm.add_bot(bot_id)
            callBack.set_data()
            continue

        elif data.startswith("split"):
            current_process(swarm, stop=True, call_back_action=swarm.stopAction)
            msg_parts = data.split("_")
            print("msg_parts", msg_parts, len(msg_parts))
            selected_uav_ids = [int(u) for u in json.loads(msg_parts[2])]
            grid_space = json.loads(msg_parts[3])
            coverage_area = json.loads(msg_parts[4])
            center_lat_lon_array = json.loads(msg_parts[1])  # All other coordinates
            swarm.AutosplitMission(
                selected_uav_ids, grid_space, coverage_area, center_lat_lon_array
            )
            callBack.set_data()

        elif data.startswith("specificsplit"):
            msg_parts = data.split("_")
            print("msg_parts", msg_parts)
            center_lat_lon_array = json.loads(msg_parts[1])  # All other coordinates
            uav_array = json.loads(msg_parts[2])
            grid_space = json.loads(msg_parts[3])
            coverage_area = json.loads(msg_parts[4])
            swarm.SpecificSplit(
                center_lat_lon_array, uav_array, grid_space, coverage_area
            )
            callBack.set_data()
            
        elif data.startswith("different"):
            msg_parts = data.split(",")
            start = int(msg_parts[1])
            diff = int(msg_parts[2])
            variables.heights = generate_heights(start_height=start,num_drones=len(vehicles.drones),difference=diff)
            print(variables.heights)
            callBack.set_data()
            
        else:
            print(f"in else data: {data}")
            # sys.exit()
            callBack.set_data()
