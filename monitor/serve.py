#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
小车监视与操作台 —— 只读本地服务(前端 + 拉取的 PNG + 状态/能力声明)
====================================================================
这是"操作台"的极简后端, 只做三件只读的事:
  1. 托管 monitor/ 下的静态前端(index.html / css / js);
  2. 提供 /media/map.png 与 /media/camera.png(由 pull.sh / fetch_monitor.sh 从车机拉取落地);
  3. 提供 /api/status 与 /api/capabilities 两份 JSON, 供前端判断可用性与新鲜度。

明确不含任何控制接口: 不转发 /cmd_vel、不提交导航目标、不停止电机、不做 SSH。
控制/导航/停止均未接入(见 INTERFACE_MAP.md)。本服务只读, 不是安全边界——
现场仍靠物理断电与车端安全链, 不要拿它当紧急停止装置。

依赖: 仅 Python3 标准库。
用法:
  python3 serve.py [--host 0.0.0.0] [--port 8123] [--media-dir MEDIA] [--config config.json]
  默认绑定 0.0.0.0(服务只读、无凭证, 便于手机经同一热点访问); 仅本机用可 --host 127.0.0.1。
"""
import argparse
import json
import os
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))

# 能力声明默认值(控制类一律 False; config.json 可覆盖, 但不建议擅自改 True)
DEFAULT_CAPABILITIES = {
    "mapView": True,        # 地图监视(拉图显示)  —— 已接通
    "camera": True,         # 相机画面(拉图显示)  —— 已接通(取决于相机是否出帧)
    "navigation": False,    # 单点导航             —— 未接入(需 HTTP 控制后端)
    "multipoint": False,    # 多点导航             —— 未接入
    "slam": False,          # 建图/存图/载图        —— 未接入
    "softwareStop": False,  # 软件停止             —— 未接入
    "mapLayers": False,     # 结构化独立图层开关    —— 无(PNG 为合成图, 不能假装分图层)
    "mapMetadata": False,   # 地图分辨率/原点/frame —— 未通过 HTTP 提供
    "videoKind": "png-poll"  # 视频来源 = PNG 轮询(非 MJPEG/RTSP)
}

# 允许托管出站的文件扩展名白名单(防止目录穿越/误泄)
SAFE_EXT = {".html", ".js", ".css", ".svg", ".png", ".json", ".ico", ".txt", ".md"}
SAFE_CTYPE = {
    ".html": "text/html; charset=utf-8",
    ".js": "text/javascript; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".svg": "image/svg+xml",
    ".png": "image/png",
    ".json": "application/json; charset=utf-8",
    ".ico": "image/x-icon",
    ".txt": "text/plain; charset=utf-8",
    ".md": "text/plain; charset=utf-8",
}


class Server(BaseHTTPRequestHandler):
    server_version = "hwcart-console/1.0"
    cfg = {}  # main() 注入: {"mode": ..., "media_dir": ..., "capabilities": ...}

    def log_message(self, fmt, *args):
        pass  # 静默默认访问日志, 只留错误到 stderr

    def _send(self, code, body, ctype):
        data = body if isinstance(body, bytes) else body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        # 只读、无凭证; 放开跨域以便手机经热点访问。永远不携带 Cookie/凭证。
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Cache-Control", "no-store, max-age=0")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Content-Length", "0")
        self.end_headers()

    def do_GET(self):
        path = self.path.split("?", 1)[0].split("#", 1)[0]

        if path in ("/", "/index.html"):
            return self._file("index.html", "text/html; charset=utf-8")

        if path == "/api/status":
            return self._send(200, json.dumps(self._status(), ensure_ascii=False),
                              "application/json; charset=utf-8")

        if path == "/api/capabilities":
            caps = self.cfg.get("capabilities", DEFAULT_CAPABILITIES)
            return self._send(200, json.dumps(caps, ensure_ascii=False),
                              "application/json; charset=utf-8")

        # 媒体图: 只映射这两个固定文件名, 不接受任意路径
        if path in ("/media/map.png", "/media/camera.png"):
            fn = "monitor.png" if path.endswith("map.png") else "camera.png"
            p = os.path.join(self.cfg["media_dir"], fn)
            if os.path.isfile(p):
                with open(p, "rb") as f:
                    return self._send(200, f.read(), "image/png")
            return self._send(404, "no media", "text/plain; charset=utf-8")

        # 其余静态资源(css/js 等), 白名单扩展名 + 防穿越
        rel = path.lstrip("/")
        ext = os.path.splitext(rel)[1].lower()
        if rel and ".." not in rel.split("/") and ext in SAFE_EXT:
            p = os.path.abspath(os.path.join(HERE, rel))
            if p.startswith(HERE) and os.path.isfile(p):  # 限制在 monitor/ 内
                with open(p, "rb") as f:
                    return self._send(200, f.read(), SAFE_CTYPE.get(ext, "application/octet-stream"))
        return self._send(404, "not found", "text/plain; charset=utf-8")

    def do_POST(self):
        # 无任何写接口
        return self._send(405, "read-only", "text/plain; charset=utf-8")

    def _file(self, rel, ctype):
        p = os.path.join(HERE, rel)
        if os.path.isfile(p):
            with open(p, "rb") as f:
                return self._send(200, f.read(), ctype)
        return self._send(404, "not found", "text/plain; charset=utf-8")

    def _stat(self, fn):
        p = os.path.join(self.cfg["media_dir"], fn)
        if os.path.isfile(p):
            st = os.stat(p)
            # genTime 恒为 None: PNG 不携带机器可读的生成时刻(车机钟还慢 5 个月),
            # 真实新鲜度画在图像左上角 scan/tf/image 戳上, 由前端如实标注"源图生成时间未知"。
            return {"exists": True, "mtime": st.st_mtime, "size": st.st_size,
                    "genTime": None}
        return {"exists": False, "mtime": None, "size": None, "genTime": None}

    def _status(self):
        return {
            "serverTime": time.time(),
            "mode": self.cfg.get("mode", "real"),
            "media": {
                "map": self._stat("monitor.png"),
                "camera": self._stat("camera.png"),
            },
        }


def load_config(path):
    cfg = {"mode": "real", "media_dir": os.path.join(HERE, "media"),
           "capabilities": DEFAULT_CAPABILITIES}
    if path and os.path.isfile(path):
        try:
            with open(path, "r", encoding="utf-8") as f:
                raw = json.load(f)
            if isinstance(raw, dict):
                cfg["mode"] = raw.get("mode", cfg["mode"])
                md = raw.get("media_dir")
                if md:
                    cfg["media_dir"] = os.path.join(HERE, md) if not os.path.isabs(md) else md
                if isinstance(raw.get("capabilities"), dict):
                    cfg["capabilities"] = {**DEFAULT_CAPABILITIES, **raw["capabilities"]}
        except Exception as e:  # 配置坏了就用默认, 不因此起不来
            print("warn: 读取配置失败, 用默认: %s" % e, file=__import__("sys").stderr)
    return cfg


def main():
    ap = argparse.ArgumentParser(description="小车监视与操作台只读本地服务")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8123)
    ap.add_argument("--media-dir", default=None)
    ap.add_argument("--config", default=None)
    args = ap.parse_args()

    cfg = load_config(args.config if args.config else os.path.join(HERE, "config.json"))
    if args.media_dir:
        cfg["media_dir"] = args.media_dir
    os.makedirs(cfg["media_dir"], exist_ok=True)
    Server.cfg = cfg

    httpd = ThreadingHTTPServer((args.host, args.port), Server)
    print("== 小车监视与操作台(只读) ==")
    print("  前端:  http://%s:%d/" % ("127.0.0.1" if args.host == "0.0.0.0" else args.host, args.port))
    print("  手机:  需与本机同一热点, 用 http://<本机局域网IP>:%d/" % args.port)
    print("  媒体目录: %s" % cfg["media_dir"])
    print("  模式: %s   控制/导航/停止: 均未接入(只读)" % cfg["mode"])
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n已停止")


if __name__ == "__main__":
    main()