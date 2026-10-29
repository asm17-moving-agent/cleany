"""The temporary release return shares preparation only across its ordered phases."""
from contextlib import contextmanager

import pytest

from test_temporary_return import temporary


@pytest.mark.parametrize('failure', ['', 'retreat', 'home'])
def test_return_preparation_closes_on_success_and_each_failed_phase(temporary, failure):
    node, context, _, _, _ = temporary
    trace = []
    @contextmanager
    def scope(arm):
        trace.append('open')
        try: yield
        finally: trace.append('close')
    def phase(name):
        def run(*args, **kwargs):
            trace.append(name)
            if name == failure: raise RuntimeError(name+' failed')
        return run
    node.real.return_preparation = scope
    node.pickup_release_retreat, node.move_arm_return = phase('retreat'), phase('home')
    context.release_confirmed = True
    if failure:
        with pytest.raises(RuntimeError, match=failure+' failed'): node.return_arm(context)
        assert not context.arm_recovered
    else:
        node.return_arm(context)
        assert context.arm_recovered
    assert trace == (['open', 'retreat', 'close'] if failure == 'retreat'
                     else ['open', 'retreat', 'home', 'close'])


def test_return_preparation_does_not_wrap_unreleased_return(temporary):
    node, context, _, _, commands = temporary
    def forbidden(*args): pytest.fail('Unreleased return opened a handoff scope')
    node.real.return_preparation = forbidden
    node.pickup_release_retreat = forbidden
    context.release_confirmed = False
    node.return_arm(context)
    assert commands == ['return'] and context.arm_recovered
