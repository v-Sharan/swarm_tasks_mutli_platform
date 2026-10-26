/**
 * @file Dialog exposing main.py's swarm-wide commands (search / split /
 * specific split / different heights / multi-waypoint goal) that don't fit
 * the stock "select UAVs, click a button" vocabulary. Stop and the per-UAV
 * add/remove self-heal commands are plain buttons elsewhere (see
 * UAVOperationsButtonGroup), not part of this dialog.
 */

import Box from '@mui/material/Box';
import Button from '@mui/material/Button';
import DialogActions from '@mui/material/DialogActions';
import DialogContent from '@mui/material/DialogContent';
import Tab from '@mui/material/Tab';
import Tabs from '@mui/material/Tabs';
import Typography from '@mui/material/Typography';
import PropTypes from 'prop-types';
import { Form } from 'react-final-form';
import { connect } from 'react-redux';

import { DraggableDialog } from '@skybrush/mui-components';

import { TextField } from '~/components/forms';
import DronePlaceholderList from '~/components/uavs/DronePlaceholderList';
import {
  sendGoalCommand,
  sendSplitCommand,
  sendSearchCommand,
  sendSpecificSplitCommand,
  sendDifferentHeightsCommand,
} from '~/features/swarm-ops/actions';
import {
  closeSwarmOpsDialog,
  setSwarmOpsTab,
} from '~/features/swarm-ops/slice';
import { getSelectedUAVIds } from '~/features/uavs/selectors';

/** "12.9,80.0;12.91,80.01" -> [[12.9, 80.0], [12.91, 80.01]] */
const parseLatLonList = (text) =>
  text
    .split(';')
    .map((part) => part.trim())
    .filter(Boolean)
    .map((pair) => pair.split(',').map(Number));

const GoalForm = ({ uavIds, onSubmit }) => (
  <Form
    initialValues={{ waypoints: '', direction: '1', radius: '50' }}
    onSubmit={(values) =>
      onSubmit(uavIds, {
        goal_latlon: parseLatLonList(values.waypoints),
        direction: Number(values.direction),
        radius: Number(values.radius),
      })
    }
  >
    {({ handleSubmit }) => (
      <Box>
        <DronePlaceholderList title='Selection:' items={uavIds} mt={0} mb={1} />
        <Typography variant='body2' color='textSecondary' sx={{ mb: 1 }}>
          Waypoints, one "lat,lon" pair per segment separated by ";"
        </Typography>
        <TextField
          name='waypoints'
          label='Waypoints'
          placeholder='12.92,80.03;12.93,80.04'
          fullWidth
          margin='normal'
        />
        <TextField
          name='direction'
          label='Guided circle direction'
          fullWidth
          margin='normal'
        />
        <TextField
          name='radius'
          label='Guided circle radius (m)'
          fullWidth
          margin='normal'
        />
        <DialogActions>
          <Button color='primary' onClick={handleSubmit} disabled={uavIds.length === 0}>
            Send goal
          </Button>
        </DialogActions>
      </Box>
    )}
  </Form>
);

GoalForm.propTypes = {
  uavIds: PropTypes.arrayOf(PropTypes.string),
  onSubmit: PropTypes.func,
};

const SearchForm = ({ onSubmit }) => (
  <Form
    initialValues={{
      centerLat: '',
      centerLon: '',
      gridSpace: '10',
      coverageArea: '200',
    }}
    onSubmit={(values) =>
      onSubmit({
        center_lat: Number(values.centerLat),
        center_lon: Number(values.centerLon),
        grid_space: Number(values.gridSpace),
        coverage_area: Number(values.coverageArea),
      })
    }
  >
    {({ handleSubmit }) => (
      <Box>
        <Typography variant='body2' color='textSecondary' sx={{ mb: 1 }}>
          Targets every active bot in the swarm -- selection is ignored.
        </Typography>
        <TextField name='centerLat' label='Center latitude' fullWidth margin='normal' />
        <TextField name='centerLon' label='Center longitude' fullWidth margin='normal' />
        <TextField name='gridSpace' label='Grid spacing (m)' fullWidth margin='normal' />
        <TextField name='coverageArea' label='Coverage area (m)' fullWidth margin='normal' />
        <DialogActions>
          <Button color='primary' onClick={handleSubmit}>
            Start search
          </Button>
        </DialogActions>
      </Box>
    )}
  </Form>
);

SearchForm.propTypes = { onSubmit: PropTypes.func };

const SplitForm = ({ uavIds, onSubmit }) => (
  <Form
    initialValues={{ region: '', gridSpace: '10', coverageArea: '200' }}
    onSubmit={(values) =>
      onSubmit(uavIds, {
        center_lat_lon_array: parseLatLonList(values.region),
        grid_space: Number(values.gridSpace),
        coverage_area: Number(values.coverageArea),
      })
    }
  >
    {({ handleSubmit }) => (
      <Box>
        <DronePlaceholderList title='Selection:' items={uavIds} mt={0} mb={1} />
        <Typography variant='body2' color='textSecondary' sx={{ mb: 1 }}>
          Region corners, "lat,lon" pairs separated by ";". Divides the
          region evenly among the selected bots.
        </Typography>
        <TextField
          name='region'
          label='Region corners'
          placeholder='12.9,80.0;12.91,80.0;12.91,80.01;12.9,80.01'
          fullWidth
          margin='normal'
        />
        <TextField name='gridSpace' label='Grid spacing (m)' fullWidth margin='normal' />
        <TextField name='coverageArea' label='Coverage area (m)' fullWidth margin='normal' />
        <DialogActions>
          <Button color='primary' onClick={handleSubmit} disabled={uavIds.length === 0}>
            Split
          </Button>
        </DialogActions>
      </Box>
    )}
  </Form>
);

SplitForm.propTypes = {
  uavIds: PropTypes.arrayOf(PropTypes.string),
  onSubmit: PropTypes.func,
};

const SpecificSplitForm = ({ onSubmit }) => (
  <Form
    initialValues={{ region: '', groups: '', gridSpace: '10', coverageArea: '200' }}
    onSubmit={(values) =>
      onSubmit({
        center_lat_lon_array: parseLatLonList(values.region),
        uav_array: values.groups
          .split(';')
          .map((group) => group.split(',').map((id) => Number(id.trim())).filter((n) => !Number.isNaN(n))),
        grid_space: Number(values.gridSpace),
        coverage_area: Number(values.coverageArea),
      })
    }
  >
    {({ handleSubmit }) => (
      <Box>
        <Typography variant='body2' color='textSecondary' sx={{ mb: 1 }}>
          Region corners, "lat,lon" pairs separated by ";". Groups: bot ids
          per region, separated by ";" (e.g. "1,2;3,4" = bots 1+2 to the
          first sub-area, 3+4 to the second).
        </Typography>
        <TextField
          name='region'
          label='Region corners'
          placeholder='12.9,80.0;12.91,80.0;12.91,80.01;12.9,80.01'
          fullWidth
          margin='normal'
        />
        <TextField name='groups' label='Bot id groups' placeholder='1,2;3,4' fullWidth margin='normal' />
        <TextField name='gridSpace' label='Grid spacing (m)' fullWidth margin='normal' />
        <TextField name='coverageArea' label='Coverage area (m)' fullWidth margin='normal' />
        <DialogActions>
          <Button color='primary' onClick={handleSubmit}>
            Specific split
          </Button>
        </DialogActions>
      </Box>
    )}
  </Form>
);

SpecificSplitForm.propTypes = { onSubmit: PropTypes.func };

const DifferentHeightsForm = ({ onSubmit }) => (
  <Form
    initialValues={{ start: '100', diff: '10' }}
    onSubmit={(values) =>
      onSubmit({ start: Number(values.start), diff: Number(values.diff) })
    }
  >
    {({ handleSubmit }) => (
      <Box>
        <Typography variant='body2' color='textSecondary' sx={{ mb: 1 }}>
          Sets every bot's commanded altitude at once -- selection is
          ignored.
        </Typography>
        <TextField name='start' label='Start height (m)' fullWidth margin='normal' />
        <TextField name='diff' label='Height step between bots (m)' fullWidth margin='normal' />
        <DialogActions>
          <Button color='primary' onClick={handleSubmit}>
            Apply heights
          </Button>
        </DialogActions>
      </Box>
    )}
  </Form>
);

DifferentHeightsForm.propTypes = { onSubmit: PropTypes.func };

const TABS = [
  { value: 'goal', label: 'Goal' },
  { value: 'search', label: 'Search' },
  { value: 'split', label: 'Split' },
  { value: 'specificsplit', label: 'Specific split' },
  { value: 'different', label: 'Heights' },
];

const SwarmOpsDialog = ({
  open,
  selectedTab,
  uavIds,
  onClose,
  onTabChange,
}) => (
  <DraggableDialog open={open} maxWidth='xs' title='Swarm ops' onClose={onClose}>
    <Tabs
      value={selectedTab}
      onChange={(_event, value) => onTabChange(value)}
      variant='scrollable'
    >
      {TABS.map((tab) => (
        <Tab key={tab.value} value={tab.value} label={tab.label} />
      ))}
    </Tabs>
    <DialogContent>
      {selectedTab === 'goal' && (
        <GoalForm uavIds={uavIds} onSubmit={sendGoalCommand} />
      )}
      {selectedTab === 'search' && <SearchForm onSubmit={sendSearchCommand} />}
      {selectedTab === 'split' && (
        <SplitForm uavIds={uavIds} onSubmit={sendSplitCommand} />
      )}
      {selectedTab === 'specificsplit' && (
        <SpecificSplitForm onSubmit={sendSpecificSplitCommand} />
      )}
      {selectedTab === 'different' && (
        <DifferentHeightsForm onSubmit={sendDifferentHeightsCommand} />
      )}
    </DialogContent>
  </DraggableDialog>
);

SwarmOpsDialog.propTypes = {
  open: PropTypes.bool,
  selectedTab: PropTypes.string,
  uavIds: PropTypes.arrayOf(PropTypes.string),
  onClose: PropTypes.func,
  onTabChange: PropTypes.func,
};

export default connect(
  // mapStateToProps
  (state) => ({
    open: state.swarmOps.dialog.open,
    selectedTab: state.swarmOps.dialog.selectedTab,
    uavIds: getSelectedUAVIds(state),
  }),
  // mapDispatchToProps
  {
    onClose: closeSwarmOpsDialog,
    onTabChange: setSwarmOpsTab,
  }
)(SwarmOpsDialog);
