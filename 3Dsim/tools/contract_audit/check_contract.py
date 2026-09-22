"""Read-only offline audit. Never imports project code or runs ROS/SSH."""
import argparse
import ast
import hashlib
import json
import math
from pathlib import Path
import subprocess
from datetime import datetime, timezone


FILES = {
    'sim': 'sim/params.py',
    'gazebo': 'hwcart_sim/src/racecar_sim/config/params.yaml',
    'teleop': 'tools/teleop/realcar_teleop.yaml',
    'safety': 'tools/realcar/cmd_vel_safety.py',
    'launcher': 'tools/teleop/teleop_manual.sh',
}


def literal(node):
    # Deliberately reject dynamic expressions; never eval/import target files.
    return ast.literal_eval(node)


def class_defaults(source, name):
    tree = ast.parse(source)
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == name)
    return {n.target.id: literal(n.value) for n in cls.body
            if isinstance(n, ast.AnnAssign) and isinstance(n.target, ast.Name) and n.value is not None}


def safety_defaults(source):
    result = {}
    for n in ast.walk(ast.parse(source)):
        if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute):
            if n.func.attr == 'declare_parameter' and len(n.args) >= 2:
                result[literal(n.args[0])] = literal(n.args[1])
    return result


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def read_sources(root):
    import yaml
    text, hashes, errors = {}, {}, []
    for name, rel in FILES.items():
        try:
            path = (root / rel).resolve()
            if not path.is_relative_to(root):
                raise ValueError('source symlink escapes requested repository')
            raw = path.read_bytes()
            hashes[rel] = hashlib.sha256(raw).hexdigest()
            text[name] = raw.decode('utf-8-sig')
        except Exception as exc:
            errors.append((name, str(exc)))
    values = {}
    for name in ('sim', 'gazebo', 'teleop', 'safety'):
        if name not in text:
            continue
        try:
            if name == 'sim':
                values[name] = class_defaults(text[name], 'CarParams')
            elif name == 'safety':
                values[name] = safety_defaults(text[name])
            else:
                data = yaml.safe_load(text[name])
                values[name] = data['racecar_sim'] if name == 'gazebo' else data['racecar_teleop']['ros__parameters']
        except Exception as exc:
            errors.append((name, str(exc)))
    return values, text, hashes, errors


def audit_values(contract, values, texts, errors=()):
    findings = []
    def add(key, status, detail):
        findings.append({'id': key, 'status': status, 'detail': detail})
    for name, error in errors:
        add('input.' + name, 'ERROR', error)
    for spec in contract['parameters']:
        for profile, field in spec['bindings'].items():
            actual = values.get(profile, {}).get(field)
            ok = finite(actual) and math.isclose(actual, spec['expected'], abs_tol=1e-6, rel_tol=0)
            add(spec['id'] + '.' + profile, 'PASS' if ok else 'FAIL',
                {'field': field, 'expected': spec['expected'], 'actual': actual})
    t, s = values.get('teleop', {}), values.get('safety', {})
    for field in ('max_speed_forward', 'max_speed_reverse'):
        a, cap = t.get(field), s.get('max_linear')
        ok = finite(a) and finite(cap) and 0 <= a <= cap and cap > 0
        add('manual_limit.' + field, 'PASS' if ok else 'FAIL', {'requested': a, 'safety_default': cap})
    delta, length, scale = t.get('max_steering_angle'), t.get('wheelbase'), t.get('low_speed_scale')
    valid = finite(delta) and 0 < delta < math.pi/2 and finite(length) and length > 0
    valid = valid and finite(scale) and 0 < scale <= 1
    add('manual.mapping_domain', 'PASS' if valid else 'FAIL', '0<steer<pi/2; wheelbase>0; 0<low_speed_scale<=1')
    if valid and all(finite(t.get(k)) for k in ('max_speed_forward', 'max_speed_reverse')):
        omega = max(abs(t['max_speed_forward']), abs(t['max_speed_reverse'])) * math.tan(delta) / length
        cap = s.get('max_angular')
        add('manual.angular_limit', 'PASS' if finite(cap) and 0 <= omega <= cap else 'FAIL', {'computed_omega': omega, 'safety_default': cap})
    timeout, watchdog = t.get('input_timeout'), s.get('cmd_timeout')
    add('manual.timeout_order', 'PASS' if finite(timeout) and finite(watchdog) and 0 < timeout <= watchdog else 'FAIL',
        {'input_timeout': timeout, 'safety_default_timeout': watchdog, 'note': 'ordering only; not a runtime stop-time guarantee'})
    pwm = values.get('sim', {}).get('pwm_mid')
    expected_pwm = contract['facts']['driver_motor_offset']['value']
    add('pwm.profile_separation', 'KNOWN_DIFFERENCE' if finite(pwm) and pwm != expected_pwm else 'REVIEW',
        {'sim_pwm_mid': pwm, 'reported_driver_offset': expected_pwm, 'note': 'do not auto-replace or deploy sim PWM mapping'})
    # Only active shell lines: lexical review hints, NOT a shell execution proof.
    active = [(i, line.strip()) for i, line in enumerate(texts.get('launcher', '').splitlines(), 1)
              if line.strip() and not line.lstrip().startswith('#')]
    kill_lines = [i for i, line in active if 'pkill' in line]
    if kill_lines:
        add('launcher.global_cleanup', 'REVIEW', {'lines': kill_lines, 'note': 'name-based kill is not limited to processes created by this launcher'})
    add('scope.runtime', 'UNVERIFIED', 'static source defaults only; no live parameters, effective remaps, ROS graph, vehicle serial or stopping behavior checked')
    return findings


def revision(root):
    try:
        # Read only, per-command safe.directory. No persistent git configuration.
        args = ['git', '-c', 'safe.directory=' + root.as_posix(), '-C', str(root)]
        sha = subprocess.check_output(args + ['rev-parse', 'HEAD'], stderr=subprocess.DEVNULL, timeout=10).decode().strip()
        return sha
    except Exception:
        return None


def exit_code(findings):
    if any(f['status'] == 'ERROR' for f in findings):
        return 2
    return 1 if any(f['status'] == 'FAIL' for f in findings) else 0


def json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    if isinstance(value, dict):
        return {k: json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    return value


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--repo', type=Path, required=True)
    ap.add_argument('--contract', type=Path, default=Path(__file__).with_name('contract.json'))
    ap.add_argument('--output', type=Path, required=True, help='new report file outside target repo')
    args = ap.parse_args()
    root, output = args.repo.resolve(), args.output.resolve()
    if output.is_relative_to(root) or output.exists():
        ap.error('output must be a NEW file outside target repo (no overwrite)')
    try:
        contract_raw = args.contract.read_bytes()
        contract = json.loads(contract_raw)
        if contract['schema_version'] != 1:
            raise ValueError('unsupported contract schema')
        values, texts, hashes, errors = read_sources(root)
        findings = audit_values(contract, values, texts, errors)
    except Exception as exc:
        print('INPUT ERROR:', exc)
        return 2
    rc = exit_code(findings)
    report = {'schema_version': 1, 'created_utc': datetime.now(timezone.utc).isoformat(),
              'repository': str(root), 'revision': revision(root), 'source_sha256': hashes,
              'contract_sha256': hashlib.sha256(contract_raw).hexdigest(),
              'findings': findings, 'known_differences': contract['known_differences'],
              'unverified': contract['unverified'], 'exit_code': rc,
              'decision': 'STATIC_CHECK_ONLY_NOT_DEPLOYMENT_APPROVAL',
              'evidence_note': 'hashes bind actual read files, including uncommitted edits; HEAD alone is not the snapshot; no atomic snapshot guaranteed'}
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('x', encoding='utf-8') as handle:
        json.dump(json_safe(report), handle, ensure_ascii=False, indent=2, allow_nan=False)
    for f in findings:
        if f['status'] != 'PASS':
            print(f['status'], f['id'], f['detail'])
    print('PASS:', sum(f['status'] == 'PASS' for f in findings), 'EXIT:', rc, 'REPORT:', output)
    return rc


if __name__ == '__main__':
    raise SystemExit(main())
