"""In-process control double for HTTP contracts; runtime tests use ROS processes."""
from cleany_logger import ArchiveReader
from cleany_logger.controller import SessionController
from cleany_dev_monitor.server import create_app as http_app


class Control:
    def __init__(self, hub, config, recorder):
        self.controller = SessionController(hub, config, recorder)

    def status(self):
        return self.controller.view()

    async def get_view(self):
        return self.status()

    async def change(self, settings, *, new_session=False):
        state = self.status()
        if not settings['active']:
            if state['state'] == 'RECORDING':
                return self.controller.command('stop', session_id=state['session_id'])
            return self.controller.command('configure', settings=settings)
        if state['state'] == 'RECORDING':
            if new_session:
                return self.controller.command('start')
            from cleany_logger.controller import ControlConflict
            raise ControlConflict('Stop recording before changing settings')
        self.controller.command('configure', settings=settings)
        return self.controller.command('start')


def create_app(hub, config, recorder, assets):
    return http_app(hub, config, ArchiveReader(recorder.directory), assets, Control(hub, config, recorder))
