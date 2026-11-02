"""Recording policy compatibility and queue bounds, without Monitor or ROS."""

import pytest
from cleany_logger import Recorder, RecordingControls
from cleany_logger.recording import defaults, rate_profiles, validate


def enabled(key="point_cloud"):
    value = defaults()
    value["enabled"][key] = True
    value["hz"][key] = 100
    return value


def test_queue_loss_and_size_limit_are_explicit(tmp_path):
    controls = RecordingControls(queue_limit=4)
    controls.configure(enabled())
    ticket = controls.reserve('/camera/head/depth/points')
    controls.offer(ticket, '/camera/head/depth/points', 'sensor_msgs/msg/PointCloud2', b'12345', 1.)
    assert controls.view()['status']['point_cloud']['dropped'] == 1
    recorder = Recorder(tmp_path, controls=controls, session_limit=1024)
    controls.offer(ticket, '/camera/head/depth/points', 'sensor_msgs/msg/PointCloud2', b'12', 1.)
    recorder.append_raw()
    assert recorder.reason and recorder.path.exists()
    assert recorder.db.execute('SELECT count(*) FROM raw_index').fetchone()[0] == 0
    recorder.close()


@pytest.mark.parametrize('change', [{'hz':None}, {'limit_mib':0}, {'active':'yes'}, {'hz':{'point_cloud':float('nan')}}])
def test_invalid_configuration_is_rejected(change):
    with pytest.raises(ValueError):
        validate(dict(defaults(), **change))


def test_raw_only_rate_settings_migrate_without_losing_custom_values():
    settings = defaults()
    settings['hz'] = {key:3.5 for key in settings['topics']}
    migrated = validate(settings)
    assert migrated['hz']['point_cloud'] == 3.5
    assert migrated['hz']['motion'] == 10.
    assert migrated['hz']['resources'] == .5
    invalid = defaults()
    invalid['hz']['motion'] = float('nan')
    with pytest.raises(ValueError):
        validate(invalid)


def test_individual_rates_migrate_and_reject_invalid_settings():
    old = defaults()
    old.pop('source_hz')
    old['hz']['motion'] = .5
    old['hz']['paths'] = 3.
    migrated = validate(old)
    assert migrated['source_hz']['odometry_topic'] == .5
    assert migrated['source_hz']['velocity_topics:0'] == .5
    assert migrated['source_hz']['local_path_topic'] == 3.
    migrated['source_hz']['path_topic'] = 1.
    migrated['hz']['paths'] = 9.
    assert validate(migrated)['source_hz']['path_topic'] == 1.
    assert validate(migrated)['source_hz']['local_path_topic'] == 3.
    for rates in [None, {'unknown':1}, {'path_topic':0}, {'rosout_topic':float('nan')}, {'velocity_topics:0':True}]:
        with pytest.raises(ValueError):
            validate({**defaults(), 'source_hz':rates})


def test_uniform_defaults_upgrade_once_and_preserve_custom_rates():
    old = defaults()
    old.pop('rate_defaults_version')
    old['source_hz'] = {key:10. for key in old['source_hz']}
    old['source_hz']['local_path_topic'] = 3.
    old['hz'].update(lidar=2.,camera=2.,point_cloud=3.5)
    migrated = validate(old)
    assert migrated['source_hz']['path_topic'] == 1.
    assert migrated['source_hz']['map_topic'] == .2
    assert migrated['source_hz']['runtime_events_topic'] is None
    assert migrated['source_hz']['local_path_topic'] == 3.
    assert migrated['hz']['lidar'] == 3.
    assert migrated['hz']['camera'] == 1.
    assert migrated['hz']['point_cloud'] == 3.5
    migrated['source_hz']['path_topic'] = 10.
    migrated['hz']['camera'] = 2.
    assert validate(migrated) == migrated


def test_legacy_checkbox_presets_migrate_without_resetting_selections_or_rates():
    for preset in ('basic','obstacles','custom'):
        old = defaults()
        old['preset'] = preset
        old['enabled']['camera'] = True
        old['source_hz']['local_path_topic'] = 3.
        migrated = validate(old)
        assert migrated['preset'] == 'standard'
        assert migrated['enabled'] == old['enabled']
        assert migrated['source_hz'] == old['source_hz']


def test_profile_rates_stay_within_publisher_and_collection_bounds(monkeypatch):
    from cleany_logger.sources import SCHEMA, raw_maximum_references
    # A changed publisher bound must affect every profile, not only the maximum label.
    monkeypatch.setitem(SCHEMA['path_topic'], 'maximum', {'hz': .5})
    profiles = rate_profiles(resource_interval_s=2.)
    for profile in profiles.values():
        for identity, hz in profile['source_hz'].items():
            if hz is not None:
                assert hz <= SCHEMA[identity.split(':')[0]]['maximum']['hz']
        assert profile['source_hz']['scan_topic'] <= 2.
        assert profile['source_hz']['controller_evaluation_topic'] <= 2.
        assert profile['hz']['resources'] <= .5
        for key, reference in raw_maximum_references().items():
            assert profile['hz'][key] <= reference['hz']
    for field in ('source_hz', 'hz'):
        for key, low in profiles['low'][field].items():
            standard = profiles['standard'][field][key]
            precise = profiles['precise'][field][key]
            assert low is standard is precise is None or low <= standard <= precise


def test_profile_upgrade_replaces_previous_generated_rates_and_keeps_user_edits():
    old = defaults()
    old['rate_defaults_version'] = 2
    old['source_hz'].update(runtime_debug_topic=5., local_path_topic=3.)
    old['hz'].update(voxel_world=2., camera=3.)
    old['enabled']['camera'] = True
    old['source_enabled']['path_topic'] = False
    updated = validate(old)
    assert updated['source_hz']['runtime_debug_topic'] == 1.
    assert updated['source_hz']['local_path_topic'] == 3.
    assert updated['hz']['voxel_world'] == .5
    assert updated['hz']['camera'] == 3.
    assert updated['enabled'] == old['enabled']
    assert updated['source_enabled'] == old['source_enabled']
    assert updated['rate_defaults_version'] == 4
    assert validate(updated) == updated


def test_v3_precise_migration_keeps_custom_rates_and_separates_semantic_maxima():
    from cleany_logger.recording import _version_three_rate_profiles
    settings = defaults()
    old = _version_three_rate_profiles()['precise']
    settings.update(preset='precise', rate_defaults_version=3, source_hz=old['source_hz'])
    settings['hz'].update(old['hz'])
    settings['hz']['lidar'] = 4.
    updated = validate(settings)
    assert updated['hz']['lidar'] == 4.
    assert updated['hz']['camera'] == 5.
    assert updated['hz']['point_cloud'] == 5.
    assert updated['source_hz']['stop_polygon_topic'] == 1.
    assert updated['rate_defaults_version'] == 4
