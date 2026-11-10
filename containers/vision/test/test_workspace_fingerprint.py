import importlib.util
from pathlib import Path

spec = importlib.util.spec_from_file_location('workspace_fingerprint',
    Path(__file__).parents[1]/'scripts/workspace_fingerprint.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_role_build_receipt_tracks_dependency_changes_and_ignores_other_role(tmp_path):
    for name, deps in (('cleany_grasping',['cleany_interfaces']), ('cleany_perception',[]),
                       ('cleany_interfaces', [])):
        package = tmp_path/name; package.mkdir()
        (package/'package.xml').write_text('<package><name>'+name+'</name>'+''.join(
            '<depend>'+dep+'</depend>' for dep in deps)+'</package>')
        (package/'source.py').write_text('first')
    first = module.fingerprint(tmp_path,'anygrasp')
    (tmp_path/'cleany_perception/source.py').write_text('second')
    assert module.fingerprint(tmp_path,'anygrasp') == first
    (tmp_path/'cleany_interfaces/source.py').write_text('second')
    assert module.fingerprint(tmp_path,'anygrasp') != first
