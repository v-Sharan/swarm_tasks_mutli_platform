/**
 * @file Thunks that send this project's custom swarm commands to the
 * backend over the generic Flockwave OBJ-CMD message, dispatched by
 * SwarmBridgeDriver's handle_multi_command_<name>/handle_command_<name>
 * methods (gui/skybrush-server/src/flockwave/server/ext/swarm_bridge/driver.py)
 * into main.py's exact wire format (main.py:43-137).
 */

import { performMassOperation } from '~/utils/messaging';

/**
 * Builds a thunk-like sender for one of main.py's named commands. `kwds`
 * is spread as keyword arguments into the matching
 * handle_multi_command_<command>()/handle_command_<command>() method on
 * the server, so its keys must match that method's parameter names
 * exactly.
 */
function createSwarmCommandSender(
  command: string,
  name: string,
  { skipConfirmation = false }: { skipConfirmation?: boolean } = {}
) {
  const send = performMassOperation<
    Record<string, unknown>,
    { command: string; kwds: Record<string, unknown> }
  >({
    type: 'OBJ-CMD',
    name,
    skipConfirmation,
    mapper: (kwds) => ({ command, kwds }),
  });

  return (uavIds: string[], kwds: Record<string, unknown> = {}) =>
    send(uavIds, kwds);
}

/** goal_<json goal_latlon>_<direction>_<radius>_<json ids> */
export const sendGoalCommand = createSwarmCommandSender('goal', 'Goal');

/** stop -- global stop of whatever main.py is currently running. */
const sendStop = createSwarmCommandSender('stop', 'Stop', {
  skipConfirmation: true,
});
export const sendStopCommand = () => sendStop([], {});

/** search,<center_lat>,<center_lon>,<num_uavs>,<grid_space>,<coverage_area> */
const sendSearch = createSwarmCommandSender('search', 'Search', {
  skipConfirmation: true,
});
export const sendSearchCommand = (kwds: Record<string, unknown>) =>
  sendSearch([], kwds);

/** split_<json center_lat_lon_array>_<json selected_uav_ids>_<json grid_space>_<json coverage_area> */
export const sendSplitCommand = createSwarmCommandSender('split', 'Split');

/** specificsplit_<json center_lat_lon_array>_<json uav_array>_<json grid_space>_<json coverage_area> */
const sendSpecificSplit = createSwarmCommandSender(
  'specificsplit',
  'Specific split',
  { skipConfirmation: true }
);
export const sendSpecificSplitCommand = (kwds: Record<string, unknown>) =>
  sendSpecificSplit([], kwds);

/** different,<start>,<diff> -- sets every bot's commanded height. */
const sendDifferent = createSwarmCommandSender(
  'different',
  'Different heights',
  { skipConfirmation: true }
);
export const sendDifferentHeightsCommand = (kwds: Record<string, unknown>) =>
  sendDifferent([], kwds);

/** remove_<json [id]> -- self-heal: freeze this one bot. */
export const sendRemoveCommand = createSwarmCommandSender(
  'remove',
  'Remove from swarm'
);

/** add_<json [id]> -- bring this one bot back into service. */
export const sendAddCommand = createSwarmCommandSender(
  'add',
  'Add back to swarm'
);
