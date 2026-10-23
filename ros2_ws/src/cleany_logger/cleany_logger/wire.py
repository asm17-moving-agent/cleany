"""Typed ROS control boundary. Disk policies remain plain Python values."""
from .recording import LABELS, RAW, defaults, validate
from .sources import SCHEMA, resolved


def config_message(settings, source_config=None):
    from cleany_interfaces.msg import RecordingConfig, RecordingCategory, RecordingSource
    value = RecordingConfig(preset=settings['preset'], limit_mib=settings['limit_mib'])
    value.categories = [RecordingCategory(name=key, enabled=settings['enabled'][key],
        rate_hz=float(settings['hz'].get(key, -1)), topic=settings['topics'].get(key, '')) for key in LABELS]
    sources = {**resolved(source_config or {}), **settings['sources']}
    for name, spec in SCHEMA.items():
        topics = [] if spec.get('internal') else sources[name] if spec['multiple'] else [sources[name]]
        identities = [f'{name}:{i}' for i in range(len(topics))] if spec['multiple'] else [name]
        hz = [settings['source_hz'].get(key) for key in identities]
        value.sources.append(RecordingSource(name=name, topics=topics,
            enabled=[settings['source_enabled'].get(key, True) for key in identities],
            rate_hz=[-1.0 if rate is None else float(rate) for rate in hz]))
    return value


def config_dict(message):
    value = defaults()
    value.update(preset=message.preset, limit_mib=message.limit_mib)
    if len(message.categories) != len(LABELS) or {item.name for item in message.categories} != set(LABELS):
        raise ValueError('All recording categories must be supplied once')
    for item in message.categories:
        value['enabled'][item.name] = item.enabled
        if item.name in value['hz']:
            value['hz'][item.name] = item.rate_hz
        if item.name in RAW:
            value['topics'][item.name] = item.topic
    if len(message.sources) != len(SCHEMA) or {item.name for item in message.sources} != set(SCHEMA):
        raise ValueError('All recording sources must be supplied once')
    for item in message.sources:
        spec = SCHEMA[item.name]
        expected = len(item.topics) if spec['multiple'] else 1
        if len(item.enabled) != expected or len(item.rate_hz) != expected:
            raise ValueError('Recording source arrays must match')
        if spec.get('internal'):
            if item.topics: raise ValueError('Computed sources have no ROS topic')
        elif spec['multiple']:
            value['sources'][item.name] = list(item.topics)
        else:
            if len(item.topics) != 1: raise ValueError('Single source requires one topic')
            value['sources'][item.name] = item.topics[0]
        identities = [f'{item.name}:{i}' for i in range(expected)] if spec['multiple'] else [item.name]
        for index, key in enumerate(identities):
            value['source_enabled'][key] = item.enabled[index]
            value['source_hz'][key] = None if item.rate_hz[index] == -1 else item.rate_hz[index]
    return validate(value)


def status_message(view):
    from cleany_interfaces.msg import RecordingStatus, RecordingCounter
    result = RecordingStatus(boot_id=view['boot_id'], session_id=view['session_id'],
        state=view['state'], reason=view['reason'], revision=view['revision'],
        updated_at=view['updated_at'], stored_bytes=view['stored_bytes'],
        resource_interval_s=view['resource_interval_s'],
        bt_event_repeat_interval_s=view['bt_event_repeat_interval_s'])
    result.counters = [RecordingCounter(name=key, count=int(item.get('count', 0)),
        dropped=int(item.get('dropped', 0)), received_at=float(item.get('received_at', 0)),
        error=item.get('error', ''), state=item['state']) for key,item in view['status'].items()]
    return result


def status_dict(message):
    return dict(boot_id=message.boot_id, session_id=message.session_id, state=message.state,
        reason=message.reason, revision=message.revision, updated_at=message.updated_at,
        stored_bytes=message.stored_bytes, resource_interval_s=message.resource_interval_s,
        bt_event_repeat_interval_s=message.bt_event_repeat_interval_s,
        status={item.name:dict(count=item.count, dropped=item.dropped, received_at=item.received_at,
            error=item.error, state=item.state) for item in message.counters})
