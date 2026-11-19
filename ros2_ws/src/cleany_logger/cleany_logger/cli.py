"""Command-line recording control; importing help and policy edits needs no ROS."""
from __future__ import annotations

import argparse
import copy
import json
import math
import sys
from dataclasses import dataclass
from typing import Protocol, Sequence

from .recording import RAW, validate
from .sources import SCHEMA, resolved, validate_sources


class LoggerUnavailable(RuntimeError):
    """Discovery or an RPC response did not arrive before its deadline."""


class LoggerRejected(RuntimeError):
    """The logger rejected an operation against its current state."""


class ControlClient(Protocol):
    def call(self, operation: str, *, settings: dict | None = None,
             session_id: str = '', revision: int = 0) -> dict: ...
    def close(self) -> None: ...


@dataclass(frozen=True)
class TopicEdit:
    target: str
    topics: tuple[str, ...] | None = None
    enabled: bool | None = None
    hz: str | None = None


def topic_rows(settings: dict) -> list[dict]:
    sources = {**resolved({}), **settings['sources']}
    rows = [dict(id=key, group=key, enabled=settings['enabled'][key],
                 selected=settings['enabled'][key], group_enabled=settings['enabled'][key],
                 topic=settings['topics'][key], hz=settings['hz'][key], raw=True)
            for key in RAW]
    for name, spec in SCHEMA.items():
        topics = ['<computed>'] if spec.get('internal') else sources[name]
        if not spec['multiple'] and not spec.get('internal'):
            topics = [topics]
        group_on = settings['enabled'][spec['group']]
        for index, topic in enumerate(topics):
            identity = f'{name}:{index}' if spec['multiple'] else name
            selected = settings['source_enabled'].get(identity, True)
            rows.append(dict(id=identity, group=spec['group'], enabled=group_on and selected,
                             selected=selected, group_enabled=group_on, topic=topic,
                             hz=settings['source_hz'].get(identity, spec['default_hz']), raw=False))
    return rows


def edit_topic(settings: dict, edit: TopicEdit) -> dict:
    """Patch one explicit role, preserving the effective selection of other roles."""
    value = copy.deepcopy(validate(settings))
    if edit.topics is None and edit.enabled is None and edit.hz is None:
        raise ValueError('Specify --topic, --enable, --disable or --hz')
    rate = None
    if edit.hz is not None and edit.hz != 'all':
        rate = float(edit.hz)
        if not math.isfinite(rate) or not .1 <= rate <= 1000:
            raise ValueError('--hz must be 0.1..1000 or all')
    if edit.target in RAW:
        if edit.topics is not None:
            if len(edit.topics) != 1:
                raise ValueError('A sensor role accepts one --topic')
            validate_sources({'path_topic': edit.topics[0]})
            value['topics'][edit.target] = edit.topics[0]
        if edit.enabled is not None:
            value['enabled'][edit.target] = edit.enabled
        if edit.hz is not None:
            if rate is None:
                raise ValueError('Sensor originals require a numeric --hz')
            value['hz'][edit.target] = rate
        return validate(value)

    name, separator, index_text = edit.target.partition(':')
    if name not in SCHEMA:
        raise ValueError(f'Unknown recording role: {edit.target}; use topics to list IDs')
    spec = SCHEMA[name]
    sources = {**resolved({}), **value['sources']}
    if separator:
        if not spec['multiple'] or not index_text.isdigit() or int(index_text) >= len(sources[name]):
            raise ValueError('Invalid source index; use topics to list IDs')
    if edit.topics is not None:
        if spec.get('internal'):
            raise ValueError('Computed observations have no ROS topic')
        if separator:
            if len(edit.topics) != 1:
                raise ValueError('An indexed source accepts one --topic')
            sources[name][int(index_text)] = edit.topics[0]
        elif spec['multiple']:
            sources[name] = list(edit.topics)
        else:
            if len(edit.topics) != 1:
                raise ValueError('This source accepts one --topic')
            sources[name] = edit.topics[0]
        value['sources'] = validate_sources(sources)
    identities = ([edit.target] if separator or not spec['multiple'] else
                  [f'{name}:{i}' for i in range(len(sources[name]))])
    if edit.enabled is not None:
        if edit.enabled and not value['enabled'][spec['group']]:
            # Enabling a parent must not accidentally enable its other children.
            for row in topic_rows(value):
                if not row['raw'] and row['group'] == spec['group']:
                    value['source_enabled'][row['id']] = False
            value['enabled'][spec['group']] = True
        for identity in identities:
            value['source_enabled'][identity] = edit.enabled
    if edit.hz is not None:
        for identity in identities:
            value['source_hz'][identity] = rate
    return validate(value)


def execute(client: ControlClient, args: argparse.Namespace) -> dict:
    current = client.call('get_config')
    if args.command == 'status':
        return current
    if args.command == 'start':
        result = client.call('start', revision=current['revision'])
        if result['state'] != 'RECORDING':
            raise LoggerRejected(result.get('reason') or 'Logger did not enter RECORDING')
        return result
    if args.command == 'stop':
        result = client.call('stop', session_id=args.session or current['session_id'],
                             revision=current['revision'])
        if result['state'] != 'IDLE':
            raise LoggerRejected(result.get('reason') or 'Logger did not finish stopping')
        return result
    if args.topic_action == 'set':
        if current['state'] in ('RECORDING', 'STOPPING'):
            raise LoggerRejected('Stop recording before changing topics')
        settings = edit_topic(current['settings'], TopicEdit(args.target,
            tuple(args.topic) if args.topic is not None else None, args.enabled, args.hz))
        settings['active'] = False
        current = client.call('configure', settings=settings, revision=current['revision'])
    return {**current, 'topics': topic_rows(current['settings'])}


def parser() -> argparse.ArgumentParser:
    def options(command: argparse.ArgumentParser) -> None:
        command.add_argument('--node', default=argparse.SUPPRESS, help='Logger node path (default: /cleany_logger)')
        command.add_argument('--timeout', type=float, default=argparse.SUPPRESS, help='Deadline per RPC in seconds (default: 3)')
        command.add_argument('--json', action='store_true', default=argparse.SUPPRESS, help='Machine-readable output')
    root = argparse.ArgumentParser(description='Control the independent Cleany logger over ROS services.')
    root.set_defaults(node='/cleany_logger', timeout=3., json=False)
    options(root)
    commands = root.add_subparsers(dest='command', required=True)
    for name in ('status', 'start', 'stop'):
        command = commands.add_parser(name)
        options(command)
        if name == 'stop':
            command.add_argument('--session', help='Expected session UUID; default: query the current session')
    topics = commands.add_parser('topics', help='List recording roles or edit a stopped logger')
    options(topics)
    topics.set_defaults(topic_action='list')
    actions = topics.add_subparsers(dest='topic_action')
    listing = actions.add_parser('list')
    options(listing)
    change = actions.add_parser('set')
    options(change)
    change.add_argument('target', help='Role ID from topics, e.g. camera, path_topic or velocity_topics:0')
    change.add_argument('--topic', action='append', help='Absolute ROS topic; repeat for velocity_topics')
    selection = change.add_mutually_exclusive_group()
    selection.add_argument('--enable', dest='enabled', action='store_true')
    selection.add_argument('--disable', dest='enabled', action='store_false')
    change.set_defaults(enabled=None)
    change.add_argument('--hz', help='0.1..1000; all means every received message for source roles')
    return root


def render(value: dict) -> str:
    lines = [f"{value['state']}  session={value['session_id']}  revision={value['revision']}",
             f"stored={value['stored_bytes']} bytes  boot={value['boot_id']}"]
    if value.get('reason'):
        lines.append(value['reason'])
    if 'topics' in value:
        lines.append('ID  GROUP  ENABLED  TOPIC  HZ')
        for row in value['topics']:
            hz = 'all' if row['hz'] is None else f"{row['hz']:g}"
            lines.append(f"{row['id']}  {row['group']}  {'yes' if row['enabled'] else 'no'}  {row['topic']}  {hz}")
    else:
        for name, counter in value['status'].items():
            lines.append(f"{name}: {counter['state']}  saved={counter['count']} dropped={counter['dropped']} {counter['error']}".rstrip())
    return '\n'.join(lines)


def main(argv: Sequence[str] | None = None, *, client_factory=None) -> int:
    args = parser().parse_args(argv)
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser().error('--timeout must be a finite positive number')
    client = None
    try:
        if client_factory is None:
            from .cli_client import RosControlClient
            client_factory = RosControlClient
        client = client_factory(args.node, args.timeout)
        value = execute(client, args)
        print(json.dumps(value, ensure_ascii=False) if args.json else render(value))
        if args.command == 'topics' and args.topic_action == 'set' and args.target in ('camera', 'point_cloud', 'voxel_layer', 'voxel_world'):
            if value['settings']['enabled'][args.target]:
                warning = 'Camera, point cloud and voxel originals can use substantial disk space; select topic and Hz carefully.'
                print(json.dumps({'warning': warning}) if args.json else warning, file=sys.stderr)
        return 0
    except LoggerUnavailable as exc:
        code, message = 3, str(exc)
    except LoggerRejected as exc:
        code, message = 4, str(exc)
    except (ValueError, ImportError) as exc:
        code, message = 2, str(exc)
    except KeyboardInterrupt:
        code, message = 130, 'Interrupted; an in-flight command may have completed. Recheck status'
    finally:
        if client is not None:
            client.close()
    print(json.dumps({'error': message, 'exit_code': code}) if args.json else f'logger: {message}', file=sys.stderr)
    return code


if __name__ == '__main__':
    raise SystemExit(main())
