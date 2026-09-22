"""generate_world.py must reject invalid configs with a clear non-zero error.

Covers the task-book D minimum: duplicate id, non-finite coordinate,
unknown model type - plus malformed header/field-count/numeric errors and an
empty file. Also checks that a valid config still succeeds (control).

Run: python3 test/test_validation_errors.py
"""
import subprocess
import sys
import tempfile
from pathlib import Path

TOOLS = Path(__file__).resolve().parent.parent / 'tools'
GEN = TOOLS / 'generate_world.py'

VALID = 'id,model,x,y,z,yaw\nred_01,cone_red,1.2,0.75,0,0\n'
CASES = [
    ('duplicate id',
     'id,model,x,y,z,yaw\na,cone_red,0,0,0,0\na,cone_blue,1,0,0,0\n',
     'duplicate'),
    ('unknown model type',
     'id,model,x,y,z,yaw\na,cone_purple,0,0,0,0\n',
     'unknown model'),
    ('non-finite x (nan)',
     'id,model,x,y,z,yaw\na,cone_red,nan,0,0,0\n',
     'non-finite'),
    ('non-finite y (inf)',
     'id,model,x,y,z,yaw\na,cone_red,0,inf,0,0\n',
     'non-finite'),
    ('non-numeric coordinate',
     'id,model,x,y,z,yaw\na,cone_red,one,0,0,0\n',
     'non-numeric'),
    ('malformed header',
     'id,model,x,y\na,cone_red,0,0\n',
     'header'),
    ('wrong field count',
     'id,model,x,y,z,yaw\na,cone_red,0,0,0\n',
     'fields'),
    ('empty file', '', 'empty'),
    ('invalid id characters',
     'id,model,x,y,z,yaw\n1st,cone_red,0,0,0,0\n',
     'invalid id'),
]


def run_gen(config_text):
    with tempfile.TemporaryDirectory() as td:
        cfg = Path(td) / 'bad.csv'
        out = Path(td) / 'out.world'
        cfg.write_text(config_text, encoding='utf-8')
        proc = subprocess.run(
            [sys.executable, str(GEN), '--config', str(cfg), '--output', str(out)],
            capture_output=True, text=True)
        wrote = out.exists()
    return proc.returncode, proc.stdout + proc.stderr, wrote


def main():
    failures = []
    for name, content, needle in CASES:
        rc, output, wrote = run_gen(content)
        ok = rc != 0 and needle in output.lower() and not wrote
        print(f'[{"PASS" if ok else "FAIL"}] reject: {name}')
        if not ok:
            failures.append(name)
            print(f'    rc={rc}, output={output.strip()!r}, wrote_output={wrote}')

    rc, output, wrote = run_gen(VALID)
    ok = rc == 0 and wrote
    print(f'[{"PASS" if ok else "FAIL"}] accept: valid config (control)')
    if not ok:
        failures.append('valid control')
        print(f'    rc={rc}, output={output.strip()!r}')

    if failures:
        print(f'== FAILED: {failures} ==')
        return 1
    print('== PASSED: validation rejects all invalid inputs, accepts valid ==')
    return 0


if __name__ == '__main__':
    sys.exit(main())
