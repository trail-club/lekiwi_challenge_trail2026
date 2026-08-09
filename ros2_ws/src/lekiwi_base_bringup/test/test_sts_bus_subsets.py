import pytest

from lekiwi_base_bringup.sts_bus import StsBus


def _bus():
    bus = object.__new__(StsBus)
    bus.ids = list(range(1, 10))
    return bus


def test_selected_ids_defaults_to_whole_bus():
    assert _bus()._selected_ids(None) == list(range(1, 10))


def test_selected_ids_preserves_wheel_subset_order():
    assert _bus()._selected_ids([7, 8, 9]) == [7, 8, 9]


def test_selected_ids_rejects_ids_outside_owned_bus():
    with pytest.raises(ValueError, match="10"):
        _bus()._selected_ids([7, 10])
