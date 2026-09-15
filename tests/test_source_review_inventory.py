import pytest
from scripts.review_asymmetric_dependents import review_slots


def test_more_than_twenty_movements_must_not_be_truncated_by_plotting():
    rows=[dict(road=str(i)) for i in range(100,124)]
    with pytest.raises(ValueError,match='omit'):review_slots(rows,range(20))
    result=review_slots(rows,range(25))
    assert len(result)==24 and [r['road'] for _,r in result]==[r['road'] for r in rows]
    assert result[-1][1]['road']=='123'


def test_empty_and_duplicate_review_inventory_are_rejected():
    with pytest.raises(ValueError,match='unique'):review_slots([],range(25))
    with pytest.raises(ValueError,match='unique'):review_slots([dict(road='100')]*2,range(25))
