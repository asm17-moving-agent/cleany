import pytest
from cleany_skill_executor.core.execution_ownership import ExecutionOwnership


def test_legacy_pipeline_and_action_server_cannot_both_own_physical_execution(tmp_path):
    path = tmp_path / 'motion.lock'
    server = ExecutionOwnership(path)
    try:
        with pytest.raises(RuntimeError, match='Another physical manipulation'):
            ExecutionOwnership(path)
    finally:
        server.close()
    resumed = ExecutionOwnership(path)
    resumed.close()
