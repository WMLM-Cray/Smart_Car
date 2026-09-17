"""看门狗状态机的测试:python3 -m pytest teleop/test_watchdog.py

不需要 ROS,时间全靠手喂。跑在 PC 上就行。
"""
from watchdog import ASSERT_FOR, SPEED_MID, TIMEOUT, Watchdog, moves

FWD = 1610          # 会让车往前走
REV = 1470          # 会让车往后走


def test_only_moving_commands_arm_it():
    assert moves(FWD) and moves(REV)
    assert not moves(SPEED_MID)
    assert not moves(1599) and not moves(1501)      # 电调死区内,车不动


def test_fires_after_the_publisher_goes_quiet():
    d = Watchdog()
    d.on_cmd(FWD, 0.0)
    assert not d.tick(0.0 + TIMEOUT)                # 还没超时
    assert d.tick(0.0 + TIMEOUT + 0.01)             # 超了 -> 补中位
    assert d.tick(0.0 + TIMEOUT + ASSERT_FOR)       # 补够 ASSERT_FOR


def test_stops_asserting_and_stays_quiet():
    """补完就休眠 —— 否则它会一直往话题上灌中位,跟导航抢底盘。"""
    d = Watchdog()
    d.on_cmd(FWD, 0.0)
    t = TIMEOUT + 0.01
    assert d.tick(t)
    assert not d.tick(t + ASSERT_FOR + 0.01)
    assert not d.tick(t + 100.0)                    # 再久也不会自己醒


def test_its_own_neutral_does_not_keep_it_alive():
    """看门狗发的中位会被自己收到,不能当成'上游还活着'。"""
    d = Watchdog()
    d.on_cmd(FWD, 0.0)
    for i in range(20):                             # 自己发的中位一路回灌
        d.on_cmd(SPEED_MID, i * 0.05)
    assert d.tick(TIMEOUT + 0.01)                   # 照样该补中位


def test_a_live_publisher_keeps_it_from_firing():
    d = Watchdog()
    for i in range(40):                             # 20Hz 一直发油门
        t = i * 0.05
        d.on_cmd(FWD, t)
        assert not d.tick(t)


def test_traffic_resuming_cancels_the_assert():
    d = Watchdog()
    d.on_cmd(FWD, 0.0)
    t = TIMEOUT + 0.01
    assert d.tick(t)                                # 正在补中位
    d.on_cmd(FWD, t + 0.05)                         # 上游又活了
    assert not d.tick(t + 0.05)                     # 立刻让位,别抢


def test_creep_pulses_are_not_cancelled():
    """低速爬行:每 0.3s 才有一个 1600 脉冲,其余都是中位。看门狗不能插进去。"""
    d = Watchdog()
    for i in range(60):                             # 20Hz x 3s
        t = i * 0.05
        d.on_cmd(FWD if i % 6 == 0 else SPEED_MID, t)
        assert not d.tick(t), f'第 {i} 拍({t:.2f}s)抢了爬行脉冲'
