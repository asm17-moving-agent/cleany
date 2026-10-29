import pytest

from cleany_perception.object_labels import ObjectLabelNormalizer


@pytest.mark.parametrize('source,expected', [
    ('  Wireless   Mouse ', 'computer mouse'), ('paper cup', 'cup'),
    ('soda can', 'beverage can'), ('Remote Control', 'remote control'),
    ('broken glass', 'broken glass'), ('battery', 'battery'),
    ('unfamiliar tool', 'unfamiliar tool'),
])
def test_aliases_are_idempotent_and_preserve_unmapped_types(source, expected):
    normalize = ObjectLabelNormalizer()
    assert normalize(source) == normalize(normalize(source)) == expected


@pytest.mark.parametrize('aliases', [
    ['mouse'], ['=cup'], ['cup='], ['a=b=c'],
    ['mouse=computer mouse', 'Mouse=rodent'], ['a=b', 'b=a'],
])
def test_invalid_alias_configuration_is_rejected(aliases):
    with pytest.raises(ValueError):
        ObjectLabelNormalizer(aliases)


def test_explicit_alias_chain_reaches_one_type_without_guessing():
    normalize = ObjectLabelNormalizer(['remote=controller', 'controller=remote control'])
    assert normalize('remote') == normalize('controller') == 'remote control'
    assert normalize('tv remote controller') == 'tv remote controller'
