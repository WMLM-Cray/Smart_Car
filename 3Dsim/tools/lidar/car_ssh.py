#!/usr/bin/env python3
"""在车上跑一条命令,输出直接流到本机 stdout。只读工具,不做任何写操作。

凭据走环境变量(避免进 shell 历史和进程列表),或写在同目录的 .car_env 文件里:
    CAR_HOST  车 IP          必填
    CAR_USER  登录用户        必填
    CAR_PASS  密码           必填(或用 CAR_KEY 指定私钥)
    CAR_PORT  SSH 端口        可选,默认 22
    CAR_KEY   私钥路径        可选,给了就优先用密钥

用法:
    python car_ssh.py "ros2 topic list"
    python car_ssh.py -t "top -b -n1"        # 强制分配 tty
    python car_ssh.py -o scan.bin "ros2 topic echo /scan"

stdout 是远端 stdout 原样,stderr 是远端 stderr,退出码 = 远端退出码。
想扒二进制/大文件就直接重定向:python car_ssh.py "cmd" > local_file
"""
import os
import sys
from pathlib import Path

import paramiko

ENV_FILE = Path(__file__).resolve().parent / ".car_env"


def load_env_file() -> None:
    """把同目录下的 .car_env(KEY=VALUE 每行一条)读进环境变量。

    这样密码写在本地文件里就行,不用打进聊天记录,也不会进 shell 历史。
    已存在的环境变量优先,不被文件覆盖。
    """
    if not ENV_FILE.is_file():
        return
    for raw in ENV_FILE.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, _, value = line.partition("=")
        key, value = key.strip(), value.strip().strip('"').strip("'")
        if key and key not in os.environ:
            os.environ[key] = value


def main() -> int:
    load_env_file()
    argv = sys.argv[1:]
    force_tty = False
    if argv and argv[0] == "-t":
        force_tty = True
        argv = argv[1:]
    if not argv:
        print(__doc__, file=sys.stderr)
        return 2
    command = " ".join(argv)

    host = os.environ.get("CAR_HOST")
    user = os.environ.get("CAR_USER")
    password = os.environ.get("CAR_PASS")
    keyfile = os.environ.get("CAR_KEY")
    port = int(os.environ.get("CAR_PORT", "22"))

    if not host or not user:
        print("缺少 CAR_HOST / CAR_USER 环境变量", file=sys.stderr)
        return 2
    if not password and not keyfile:
        print("缺少 CAR_PASS(或 CAR_KEY)环境变量", file=sys.stderr)
        return 2

    client = paramiko.SSHClient()
    # 首次连接会拿到未知主机密钥;实验室小车不做固定指纹校验,接受并继续。
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        client.connect(
            hostname=host,
            port=port,
            username=user,
            password=password,
            key_filename=keyfile,
            timeout=15,
            banner_timeout=20,
            auth_timeout=20,
            look_for_keys=not password,   # 有密码就别去翻本机密钥,免得触发 MaxAuthTries
            allow_agent=False,
        )
    except paramiko.AuthenticationException:
        print("认证失败:用户名或密码不对", file=sys.stderr)
        return 3
    except Exception as exc:                       # noqa: BLE001 - 直接报给用户看
        print(f"连接 {user}@{host}:{port} 失败: {type(exc).__name__}: {exc}",
              file=sys.stderr)
        return 4

    try:
        _stdin, stdout, stderr = client.exec_command(
            command, get_pty=force_tty, timeout=None
        )
        out = stdout.channel.makefile("rb")
        while True:
            chunk = out.read(65536)
            if not chunk:
                break
            sys.stdout.buffer.write(chunk)
            sys.stdout.buffer.flush()
        err = stderr.read()
        if err:
            sys.stderr.buffer.write(err)
            sys.stderr.buffer.flush()
        rc = stdout.channel.recv_exit_status()
    except KeyboardInterrupt:
        rc = 130
    finally:
        client.close()
    return rc


if __name__ == "__main__":
    sys.exit(main())
