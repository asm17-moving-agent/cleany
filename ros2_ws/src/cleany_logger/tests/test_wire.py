"""Wire contracts reject malformed settings before they reach a writer."""
import pytest

pytest.importorskip('cleany_interfaces.msg._recording_config')
from cleany_logger.recording import defaults, validate
from cleany_logger.wire import config_dict, config_message


def test_typed_policy_roundtrip_preserves_source_choices_and_event_rates():
    settings = validate(defaults())
    settings['sources']['path_topic'] = '/another/plan'
    settings['source_enabled']['velocity_topics:1'] = False
    settings['source_hz']['rosout_topic'] = None
    settings['enabled']['camera'] = True
    result = config_dict(config_message(settings))
    assert result['sources']['path_topic'] == '/another/plan'
    assert result['source_enabled']['velocity_topics:1'] is False
    assert result['source_hz']['rosout_topic'] is None
    assert result['enabled']['camera']
    assert config_dict(config_message(result)) == result


def test_reject_duplicate_categories_and_mismatched_source_arrays():
    message = config_message(defaults())
    message.categories[0].name = message.categories[1].name
    with pytest.raises(ValueError, match='categories'):
        config_dict(message)
    message = config_message(defaults())
    message.sources[0].enabled = []
    with pytest.raises(ValueError, match='arrays'):
        config_dict(message)
