"""Install a separate longitudinal retry without changing the Omni controller."""
from copy import deepcopy
from .runtime_config import merge_parameters


def configure_retry(parameters: dict, overlay: dict, lattice_path: str) -> dict:
    retry = deepcopy(parameters)
    merge_parameters(retry, overlay)
    controller = parameters['controller_server']['ros__parameters']
    controller['Longitudinal'] = retry['controller_server']['ros__parameters']['FollowPath']
    controller['controller_plugins'] = list(dict.fromkeys([*controller['controller_plugins'], 'Longitudinal']))
    # NodeLattice's static motion table cannot retain two primitive sets in one
    # process. A namespaced server owns the retry table and its own costmap.
    planner = deepcopy(retry['planner_server']['ros__parameters'])
    planner['Longitudinal'] = planner.pop('GridBased')
    planner['Longitudinal']['lattice_filepath'] = lattice_path
    planner['planner_plugins'] = ['Longitudinal']
    costmap = deepcopy(parameters['global_costmap']['global_costmap']['ros__parameters'])
    for layer in costmap.get('plugins', []):
        config = costmap.get(layer, {})
        if 'StaticLayer' in config.get('plugin', ''):
            config.setdefault('map_topic', '/map')
        if 'map_topic' in config:
            config['map_topic'] = '/'+config['map_topic'].lstrip('/')
        for source in config.get('observation_sources', '').split():
            topic = config[source]['topic']
            config[source]['topic'] = '/'+topic.lstrip('/')
    bt = parameters['bt_navigator']['ros__parameters']
    bt.setdefault('plugin_lib_names', []).append('cleany_slip_recovery_bt_node')
    return {'/traction_retry/planner_server': {'ros__parameters':planner},
            '/traction_retry/global_costmap/global_costmap': {'ros__parameters':costmap}}
