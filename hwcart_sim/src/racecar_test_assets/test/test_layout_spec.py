"""Assert the shipped layout equals the task-book C2/C3 test-fixture spec.

This is the round-1 layout contract (a TEST FIXTURE, not an official track):
- 10 cones: red_01..05 at y=+0.75, blue_01..05 at y=-0.75,
  x = 1.2, 2.4, 3.6, 4.8, 6.0 (same-side pitch 1.2 m), all z=0, yaw=0
- board_01 (blue_board) at (3.0, -0.75, 0), yaw=0, between blue_02/blue_03,
  must not overlap the neighbouring cone bases
- reference_box at (8.0, 0, 0) -> front (+X) surface at x=7.75 m

Run: python3 test/test_layout_spec.py
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'tools'))
import asset_params as P  # noqa: E402
import generate_world as gw  # noqa: E402

EXPECTED_CONE_X = [1.2, 2.4, 3.6, 4.8, 6.0]
ROW_Y = 0.75


def main():
    failures = []

    def check(name, ok, detail=''):
        print(f'[{"PASS" if ok else "FAIL"}] {name}' + (f'  ({detail})' if detail else ''))
        if not ok:
            failures.append(name)

    try:
        objs = {o['id']: o for o in gw.load_layout(gw.DEFAULT_CONFIG)}
    except gw.LayoutError as e:
        print(f'[FAIL] config invalid: {e}')
        return 1

    expected_ids = {f'red_{i:02d}' for i in range(1, 6)} | \
                   {f'blue_{i:02d}' for i in range(1, 6)} | {'board_01', 'reference_box'}
    check('exactly the 12 expected object ids', set(objs) == expected_ids,
          f'{len(objs)} objects')

    for side, sign in (('red', +1), ('blue', -1)):
        ids = [f'{side}_{i:02d}' for i in range(1, 6)]
        check(f'{side} row: 5 cones present', all(i in objs for i in ids))
        xs = [objs[i]['x'] for i in ids if i in objs]
        check(f'{side} row: x = {EXPECTED_CONE_X} (pitch 1.2 m)',
              len(xs) == 5 and all(abs(a - b) < 1e-9 for a, b in zip(xs, EXPECTED_CONE_X)),
              str(xs))
        ok_yz = all(objs[i]['y'] == sign * ROW_Y and objs[i]['z'] == 0
                    and objs[i]['yaw'] == 0 for i in ids if i in objs)
        check(f'{side} row: y = {sign * ROW_Y:+.2f}, z = 0, yaw = 0', ok_yz)
        check(f'{side} row: model = cone_{side}',
              all(objs[i]['model'] == f'cone_{side}' for i in ids if i in objs))

    check('row separation = 1.50 m (y +0.75 / -0.75)',
          'red_01' in objs and 'blue_01' in objs
          and abs((objs['red_01']['y'] - objs['blue_01']['y']) - 1.5) < 1e-9)

    b = objs.get('board_01')
    check('board_01: blue_board at (3.0, -0.75, 0), yaw=0',
          b is not None and b['model'] == 'blue_board' and b['x'] == 3.0
          and b['y'] == -0.75 and b['z'] == 0 and b['yaw'] == 0)

    # board must not intrude into neighbouring cone bases (blue_02 / blue_03)
    half_len = P.BOARD_LENGTH / 2
    half_base = P.CONE_BASE_SIZE / 2
    gap_left = (3.0 - half_len) - (2.4 + half_base)   # board start - blue_02 base end
    gap_right = (3.6 - half_base) - (3.0 + half_len)  # blue_03 base start - board end
    check('board does not overlap blue_02/blue_03 bases (x-direction)',
          gap_left > 0 and gap_right > 0,
          f'gaps {gap_left:.2f} m / {gap_right:.2f} m')

    r = objs.get('reference_box')
    front = r['x'] - P.REFBOX_SIZE[0] / 2 if r else None
    check('reference_box: model at (8.0, 0, 0), yaw=0',
          r is not None and r['model'] == 'reference_box' and r['x'] == 8.0
          and r['y'] == 0 and r['z'] == 0 and r['yaw'] == 0)
    check('reference_box front (+X) surface at x = 7.75 m',
          front is not None and abs(front - 7.75) < 1e-9,
          f'x={front}')

    if failures:
        print(f'== FAILED: {len(failures)} check(s): {failures} ==')
        return 1
    print('== PASSED: layout matches the task-book C2/C3 test-fixture spec ==')
    return 0


if __name__ == '__main__':
    sys.exit(main())
