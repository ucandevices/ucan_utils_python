import can_calc_bittiming


def test_answer():

    bt = can_calc_bittiming.bt()
    bt.bitrate = 100*1000
    btc = can_calc_bittiming.btc()
    assert can_calc_bittiming.CAN_CALC_BITTIMING(bt,btc)