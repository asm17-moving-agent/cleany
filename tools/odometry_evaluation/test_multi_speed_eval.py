"""다중 속도 도구의 사전 검사와 결과 호환성을 하드웨어 없이 검증합니다."""
import json
import pytest
import run_multi_speed_eval as runner
from evaluation_results import reference_error


def test_reference_error_accepts_old_and_prefers_new_schema():
    assert reference_error({"physical_ground_truth": {"error_x_mm": 2}}) == {"error_x_mm": 2}
    assert reference_error({"physical_ground_truth": {"error_x_mm": 2},
                            "reference_error": {"error_x_mm": 3}}) == {"error_x_mm": 3}


def test_missing_output_stops_before_any_subprocess(monkeypatch):
    monkeypatch.delenv("CLEANY_DATA_ROOT", raising=False)
    monkeypatch.setattr(runner, "run_single_eval", lambda **kw: pytest.fail("motion before output validation"))
    with pytest.raises(SystemExit) as error:
        runner.main([])
    assert error.value.code == 2


def test_dry_run_never_creates_output_directory(monkeypatch, tmp_path):
    output = tmp_path / "absent"
    calls = []
    monkeypatch.setattr(runner, "run_single_eval", lambda **kw: calls.append(kw) or {"dry_run": True})
    assert runner.main(["--dry-run", "--output-dir", str(output), "--speeds", ".2", "--runs-per-speed", "1"]) == 0
    assert calls[0]["output_path"].parent == output and calls[0]["dry_run"]
    assert not output.exists()


def test_failed_run_preserves_configuration_and_aborts_batch(monkeypatch, tmp_path):
    monkeypatch.setenv("CLEANY_DATA_ROOT", str(tmp_path))
    calls = []
    monkeypatch.setattr(runner, "run_single_eval", lambda **kw: calls.append(kw))
    assert runner.main(["--speeds", ".2", ".3"]) == 1
    assert len(calls) == 1
    output = calls[0]["output_path"].parent
    assert output.parent == tmp_path / "runs/navigation/control"
    assert json.loads((output / "batch_config.json").read_text())["speeds"] == [.2, .3]


def test_existing_output_is_not_reused(monkeypatch, tmp_path):
    monkeypatch.setattr(runner, "run_single_eval", lambda **kw: pytest.fail("existing run reused"))
    with pytest.raises(FileExistsError):
        runner.main(["--output-dir", str(tmp_path)])


@pytest.mark.parametrize('arguments', [['--runs-per-speed', '0'], ['--distance', '-1'],
    ['--speeds', 'nan'], ['--speeds', '0'], ['--dwell-between', '-1']])
def test_invalid_batch_is_rejected_before_subprocess(monkeypatch, tmp_path, arguments):
    monkeypatch.setattr(runner, 'run_single_eval', lambda **kw: pytest.fail('invalid batch ran'))
    output = tmp_path/'run'
    with pytest.raises(SystemExit) as error:
        runner.main(['--output-dir', str(output), *arguments])
    assert error.value.code == 2 and not output.exists()
