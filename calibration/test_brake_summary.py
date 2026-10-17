"""calibration/brake_summary 在入库的 2026-10-12 刹车数据上的结论不变。"""
from calibration.brake_summary import main, summarize
import glob
import os

DIR = os.path.join(os.path.dirname(__file__), '..', 'docs', '实测', '2026-10-12-brake')


def _row(name):
    return summarize(glob.glob(os.path.join(DIR, name + '_*.csv'))[0])


def test_below_1470_brakes_and_1500_still_coasts():
    assert 1.2 < _row('b25')['a'] < 1.8                  # 1470 从 2 m/s:真刹车
    assert _row('b05')['a'] < 1.0                        # 1500:还是滑行
    assert _row('b04')['back'] > 0.1                     # 1 m/s 起刹到低速还给 1400:倒退


def test_main_prints_every_run(capsys):
    main([DIR])
    assert len(capsys.readouterr().out.strip().splitlines()) == 1 + len(glob.glob(os.path.join(DIR, 'b*.csv')))
