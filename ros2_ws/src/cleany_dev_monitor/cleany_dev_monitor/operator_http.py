"""Authenticated optional commands with durable jobs and explicit rollback gating."""

import asyncio
import hmac
import json
from pathlib import Path
import secrets
import time
from uuid import UUID

from aiohttp import web

from .operator_jobs import alive, atomic_json, cancel_external_client, command_report, process_start, runtime_info
from .operator_policy import capabilities, validate_action
from .proposals import ProposalStore, identifier, policy_settings


class OperatorControl:
    def __init__(self, config):
        self.config = config
        self.root = Path(config.get('control_directory', '~/.local/state/cleany/dev-monitor/operator')).expanduser()
        self.jobs = self.root / 'jobs'
        self.repo = Path(config.get('control_repository_root', '')).resolve()
        self.sessions, self.owner, self.failures = {}, '', {}
        self.port = None
        self.state, self.state_time = {}, 0.
        self.mutation_lock, self.state_lock = asyncio.Lock(), asyncio.Lock()
        self.session_seconds = max(60, int(config.get('control_session_seconds', 3600)))
        self.proposals = ProposalStore(self.root)
        self.proposal_policy = policy_settings(config)

    @staticmethod
    async def body(request):
        try:
            value = await request.json()
        except (ValueError, UnicodeError):
            raise ValueError('JSON 요청이 필요합니다.')
        if not isinstance(value, dict):
            raise ValueError('JSON 객체 요청이 필요합니다.')
        return value

    def enabled(self):
        return self.config.get('control_enabled') is True and (self.root / 'enabled').is_file()

    def job_history(self):
        rows = []
        if self.jobs.exists():
            for path in sorted(self.jobs.glob('*.json'), key=lambda p: p.stat().st_mtime, reverse=True)[:20]:
                try:
                    job = json.loads(path.read_text())
                    if job['state'] == 'RUNNING' and (not alive(job.get('owner_pid'))
                            or job.get('owner_start') != process_start(job.get('owner_pid'))):
                        job = {**job, 'state': 'UNKNOWN', 'message': '명령 프로세스 상태를 확인하지 못했습니다. 실행기를 조회하세요.'}
                    if job['state'] == 'PENDING' and time.time() - job['created_at'] > 15:
                        job = {**job, 'state': 'UNKNOWN', 'message': '명령 시작을 확인하지 못했습니다. 상태를 다시 조회하세요.'}
                    if job['action'] in ('run', 'approve') and job['state'] in ('SUCCEEDED', 'FAILED', 'INTERRUPTED') and 'result' not in job:
                        job = {**job, **command_report(path.with_suffix('.log'))}
                    rows.append(job)
                except (OSError, ValueError, KeyError):
                    pass
        return rows

    def session(self, request):
        sid = request.cookies.get('cleany_operator', '')
        value = self.sessions.get(sid)
        if value and value['expires'] > time.monotonic() and sid == self.owner:
            return value
        return None

    def require(self, request):
        if not self.enabled():
            raise web.HTTPNotFound(text='웹 제어가 비활성화되어 있습니다.')
        if request.headers.get('Origin') != f'{request.scheme}://{request.host}':
            raise web.HTTPForbidden(text='같은 화면에서 보낸 요청만 허용합니다.')
        actor = self.session(request)
        if actor is None:
            raise web.HTTPUnauthorized(text='조작 권한 인증이 필요합니다.')
        return actor

    def get_port(self):
        if self.port is None:
            factory = self.config.get('_operator_port_factory')
            if factory is None:
                from .operator_ros import OperatorROS
                factory = OperatorROS
            self.port = factory()
        return self.port

    async def snapshot(self, fresh=False):
        async with self.state_lock:
            if fresh or time.monotonic() - self.state_time > 1.5:
                try:
                    state = await asyncio.to_thread(self.get_port().snapshot)
                    self.state = {**state, 'available': True}
                except Exception as error:
                    self.state = dict(available=False, message=str(error))
                self.state_time = time.monotonic()
        return dict(self.state)

    async def status(self, request):
        if not self.enabled():
            return web.json_response(dict(enabled=False, authenticated=False))
        state = await self.snapshot()
        info, jobs = runtime_info(), self.job_history()
        active = next((j for j in jobs if j['state'] in ('PENDING', 'RUNNING')), None)
        actor = self.session(request)
        unknown = any(j['state'] == 'UNKNOWN' for j in jobs)
        return web.json_response(dict(enabled=True, authenticated=bool(actor), actor=actor['name'] if actor else '',
            state=state, runtime=info, active_job=active, jobs=jobs, proposal=self.proposals.current(),
            capabilities=capabilities(state, busy=bool(active) or info['client_busy'] or unknown, managed=info['managed'])))

    async def login(self, request):
        if not self.enabled():
            raise web.HTTPNotFound()
        if request.headers.get('Origin') != f'{request.scheme}://{request.host}':
            raise web.HTTPForbidden()
        data = await self.body(request)
        key = self.root / 'key'
        if not key.is_file() or key.stat().st_mode & 0o077:
            raise web.HTTPServiceUnavailable(text='조작키 파일 권한을 확인하세요.')
        remote = request.remote or ''
        attempts, since = self.failures.get(remote, (0, time.monotonic()))
        if time.monotonic() - since > 60:
            attempts, since = 0, time.monotonic()
        if attempts >= 6:
            raise web.HTTPTooManyRequests(text='잠시 후 다시 인증하세요.')
        if not isinstance(data.get('key'), str) or not hmac.compare_digest(data['key'].encode(), key.read_text().strip().encode()):
            if len(self.failures) > 128:
                self.failures.clear()
            self.failures[remote] = attempts + 1, since
            raise web.HTTPUnauthorized(text='조작키가 맞지 않습니다.')
        name = data.get('name', '')
        if not isinstance(name, str):
            raise ValueError('조작자 이름이 필요합니다.')
        name = name.strip()
        if not 1 <= len(name) <= 40:
            raise ValueError('조작자 이름을 입력하세요.')
        owner = self.sessions.get(self.owner)
        if owner and owner['expires'] > time.monotonic() and not self.session(request) and data.get('takeover') is not True:
            raise web.HTTPConflict(text='다른 브라우저가 제어권을 사용 중입니다. 가져오기를 명시하세요.')
        sid = secrets.token_urlsafe(32)
        self.sessions.clear()
        self.sessions[sid] = dict(name=name, expires=time.monotonic()+self.session_seconds)
        self.owner = sid
        response = web.json_response(dict(authenticated=True, actor=name))
        response.set_cookie('cleany_operator', sid, httponly=True, samesite='Strict', max_age=self.session_seconds,
                            secure=request.scheme == 'https', path='/api/control')
        return response

    async def logout(self, request):
        self.require(request)
        self.sessions.clear()
        self.owner = ''
        response = web.json_response(dict(authenticated=False))
        response.del_cookie('cleany_operator', path='/api/control')
        return response

    async def submit(self, request):
        actor = self.require(request)
        data = await self.body(request)
        action, args = data.get('action'), validate_action(data.get('action'), data.get('args', {}))
        try:
            request_id = str(UUID(data['request_id']))
        except (ValueError, KeyError, TypeError, AttributeError):
            raise ValueError('요청 UUID가 필요합니다.')
        async with self.mutation_lock:
            self.require(request)
            self.jobs.mkdir(parents=True, exist_ok=True)
            path = self.jobs / f'{request_id}.json'
            if path.exists():
                prior = json.loads(path.read_text())
                if prior['action'] != action or prior['args'] != args:
                    raise web.HTTPConflict(text='동일 요청 ID의 내용이 다릅니다.')
                return web.json_response(prior)
            if any(j['state'] in ('PENDING', 'RUNNING', 'UNKNOWN') for j in self.job_history()):
                raise web.HTTPConflict(text='진행 중이거나 결과를 확인하지 못한 명령이 있습니다.')
            state = await self.snapshot(fresh=True)
            actor = self.require(request)
            info = runtime_info()
            reason = capabilities(state, busy=info['client_busy'], managed=info['managed'])[action]
            if reason:
                raise web.HTTPConflict(text=reason)
            if action == 'recover' and dict(execution_id=args['execution_id'], revision=args['revision']) not in state.get('blocked', []):
                raise web.HTTPConflict(text='차단 기록이 변경되었습니다. 상태를 다시 조회하세요.')
            proposal = None
            if action in ('approve', 'skip'):
                proposal = self.proposals.claimable(args['proposal_id'])
            job = dict(id=request_id, action=action, args=args, actor=actor['name'],
                       state='PENDING', created_at=time.time(), owner_pid=0)
            if action in ('approve', 'skip', 'propose'):
                job.update(proposal=proposal or self.proposals.current(), proposal_policy=self.proposal_policy)
            atomic_json(path, job)
            if action not in ('approve', 'skip'):
                self.proposals.invalidate()
            runner = self.config.get('_operator_launch')
            try:
                if runner:
                    await runner(path, self.repo)
                else:
                    process = await asyncio.create_subprocess_exec('systemd-run', '--user', '--scope', '--quiet',
                        '--unit', f'cleany-operator-{request_id}', str(self.repo / 'tools/dev-monitor-operator-job'),
                        str(path), stdout=asyncio.subprocess.DEVNULL, stderr=asyncio.subprocess.DEVNULL,
                        start_new_session=True)
                    # The worker writes its own durable result independently of this HTTP process.
                    asyncio.create_task(process.wait())
            except Exception as error:
                atomic_json(path, {**job, 'state': 'FAILED', 'message': str(error)})
                raise
            return web.json_response(job, status=202)

    async def proposal_image(self, request):
        if not self.enabled() or not self.session(request):
            raise web.HTTPUnauthorized()
        path = self.proposals.directory / f"{identifier(request.match_info['proposal'])}.jpg"
        if not path.is_file():
            raise web.HTTPNotFound()
        return web.FileResponse(path, headers={'Cache-Control': 'no-store'})

    async def cancel(self, request):
        self.require(request)
        data = await self.body(request)
        mode = data.get('mode', 'IMMEDIATE')
        if mode not in ('IMMEDIATE', 'CHECKPOINT'):
            raise ValueError('지원하지 않는 취소 모드입니다.')
        state = await self.snapshot(fresh=True)
        self.require(request)
        execution_id = state.get('active_execution_id')
        if execution_id:
            result = await asyncio.to_thread(self.get_port().cancel, execution_id, mode)
            return web.json_response({**result, 'stop_confirmed': False})
        if mode != 'IMMEDIATE':
            raise web.HTTPConflict(text='현재 단계 후 중단은 실물 조작 중에만 가능합니다.')
        job = next((j for j in self.job_history() if j['state'] in ('PENDING', 'RUNNING')), None)
        if job is None:
            if cancel_external_client():
                return web.json_response(dict(accepted=True, stop_confirmed=False,
                    message='CLI 명령 중단 요청됨. 실물 정지 확인은 별도입니다.'))
            raise web.HTTPConflict(text='웹에서 시작한 진행 중 명령이 없습니다. 실행기 상태를 확인하세요.')
        (self.jobs / f"{job['id']}.cancel").touch()
        return web.json_response(dict(accepted=True, stop_confirmed=False, message='명령 중단 요청됨. 실물 정지 확인은 별도입니다.'))

    async def acknowledge(self, request):
        actor = self.require(request)
        data = await self.body(request)
        if data.get('confirmed') is not True or not isinstance(data.get('note'), str) or not data['note'].strip():
            raise ValueError('실물 상태 확인과 메모가 필요합니다.')
        job_id = str(UUID(request.match_info['job']))
        async with self.mutation_lock:
            job = next((j for j in self.job_history() if j['id'] == job_id and j['state'] == 'UNKNOWN'), None)
            if job is None:
                raise web.HTTPConflict(text='확인이 필요한 명령 기록이 아닙니다.')
            state = await self.snapshot(fresh=True)
            actor = self.require(request)
            if (state.get('active_execution_id') or state.get('arm_return_active')
                    or state.get('recovery_active') or runtime_info()['client_busy']):
                raise web.HTTPConflict(text='진행 중인 동작을 먼저 확인하세요.')
            job.update(state='ACKNOWLEDGED', acknowledged_by=actor['name'], acknowledgement=data['note'][:500])
            atomic_json(self.jobs/f'{job_id}.json', job)
        return web.json_response(dict(success=True, message='명령 기록만 확인했습니다. 실행기의 차단과 오류는 유지됩니다.'))

    async def close(self, app):
        if self.port is not None:
            await asyncio.to_thread(self.port.close)


def attach_control(app, config):
    control = OperatorControl(config)
    app.router.add_get('/api/control/status', control.status)
    app.router.add_post('/api/control/login', control.login)
    app.router.add_post('/api/control/logout', control.logout)
    app.router.add_post('/api/control/jobs', control.submit)
    app.router.add_get('/api/control/proposals/{proposal}/image', control.proposal_image)
    app.router.add_post('/api/control/cancel', control.cancel)
    app.router.add_post('/api/control/jobs/{job}/acknowledge', control.acknowledge)
    app.on_cleanup.append(control.close)
    return control
