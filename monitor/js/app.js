'use strict';
/*
 * 小车监视与操作台 —— 前端主逻辑
 * 设计红线:
 *   - 只读。本页面不向任何控制话题(/cmd_vel、/car_cmd_vel、/teleop_cmd_vel、
 *     /teleop/cmd_vel)发指令, 不做 SSH, 不实现导航/停止控制。
 *   - 不把多个数据源合成"系统正常"; 缺证据一律显示"未知/未接入"。
 *   - 演示数据只在显式"演示模式"下出现, 全页常亮标识, 不连控制接口。
 *   - 断线/切后台/刷新不自动重发任何控制命令(本页本就无控制命令可发)。
 */

const App = {
  // ---- 状态 ----
  kind: 'http',          // 'http' | 'demo'
  baseUrl: '',
  adapter: null,
  caps: null,
  status: null,
  connState: 'offline',  // 'offline' | 'connecting' | 'online' | 'unavailable'
  recent: [],            // 最近地址
  watching: false,       // 视频是否在观看
  mapLoadedOnce: false,
  mapLastMtime: null,
  camLastMtime: null,
  camLastChangeAt: null,
  statusTimer: null,
  lastStatusOkAt: null,
  anomalies: [],
  view: 'overview',

  // 地图 pan/zoom
  mx: { scale: 1, tx: 0, ty: 0, fitted: false },

  // ---- DOM 缓存 ----
  $: (id) => document.getElementById(id),

  init() {
    this.loadRecent();
    this.baseUrl = localStorage.getItem('hwcart_console_base') || '';
    this.$('backend-addr').value = this.baseUrl;
    this.bindNav();
    this.bindMap();
    this.bindVideo();
    this.bindSettings();
    this.bindDemo();
    this.bindVisibility();
    this.renderRecent();

    // 显式入口: ?demo=1 进入演示模式(全页常亮标识, 不连控制), ?view=x 指定初始页(测试/截图用)
    const q = new URL(location.href).searchParams;
    this.showView(q.get('view') || 'overview');
    if (q.get('demo') === '1') {
      this.useDemo();
      this.setConn('online');
      this.onStatus();
    } else {
      this.useHttp(this.baseUrl);
      this.connect();
    }
    this.statusTimer = setInterval(() => this.tick(), 2000);
  },

  /* ---------------- 后端构造与连接 ---------------- */
  useHttp(baseUrl) {
    this.kind = 'http';
    this.baseUrl = baseUrl || '';
    this.adapter = makeBackend('http', this.baseUrl);
    this.$('demo-off').classList.add('hidden');
    this.$('demo-on').classList.remove('hidden');
    this.$('demo-banner').classList.add('hidden');
  },
  useDemo() {
    this.kind = 'demo';
    this.adapter = makeBackend('demo');
    this.$('demo-on').classList.add('hidden');
    this.$('demo-off').classList.remove('hidden');
    this.$('demo-banner').classList.remove('hidden');
  },

  setConn(state) {
    this.connState = state;
    const p = this.$('conn-pill');
    p.className = 'pill';
    const map = {
      offline: ['pill-offline', '未连接'],
      connecting: ['pill-connecting', '连接中'],
      online: ['pill-online', '在线'],
      unavailable: ['pill-unavailable', '服务不可用'],
    };
    const [cls, txt] = map[state] || map.offline;
    p.classList.add(cls); p.textContent = txt;
    this.$('conn-state').textContent =
      txt + (this.kind === 'demo' ? '（演示）' : '');
  },

  async connect() {
    if (this.kind === 'demo') { this.setConn('online'); this.tick(); return; }
    this.setConn('connecting');
    try {
      this.status = await this.adapter.getStatus();
      this.caps = await this.adapter.getCapabilities();
      this.lastStatusOkAt = Date.now();
      this.setConn('online');
      this.onStatus();
    } catch (e) {
      // 超时/网络错 = offline; HTTP 有响应但非 2xx 已在 adapter 抛 HTTP错误 = 服务不可用
      this.caps = await this.adapter.getCapabilities().catch(() => null);
      const isHttpErr = String(e && e.message).startsWith('HTTP');
      this.setConn(isHttpErr ? 'unavailable' : 'offline');
      this.pushAnomaly(isHttpErr ? 'err' : 'warn',
        isHttpErr ? '后端响应异常：' + e.message : '无法连接后端：' + (e && e.message));
      this.onStatus();
    }
  },

  disconnect() {
    this.setConn('offline');
    this.stopPolling();
    this.status = null;
    this.caps = null;
    this.onStatus();
  },

  async tick() {
    if (this.kind === 'demo') { this.onStatus(); return; }
    if (this.connState === 'offline' || this.statusTimer === null) return;
    try {
      this.status = await this.adapter.getStatus();
      if (!this.caps) this.caps = await this.adapter.getCapabilities();
      this.lastStatusOkAt = Date.now();
      if (this.connState !== 'online') this.setConn('online');
    } catch (e) {
      const isHttpErr = String(e && e.message).startsWith('HTTP');
      this.setConn(isHttpErr ? 'unavailable' : 'offline');
      this.pushAnomaly(isHttpErr ? 'err' : 'warn',
        '状态拉取失败：' + (e && e.message));
    }
    this.onStatus();
  },

  stopPolling() {
    if (this.statusTimer) { clearInterval(this.statusTimer); this.statusTimer = null; }
    this.stopWatching();
  },

  /* ---------------- 状态 → 界面 ---------------- */
  onStatus() {
    this.renderMode();
    this.renderSources();
    this.renderTask();
    this.renderAnomalies();
    this.renderDiag();
    this.renderMap();
    this.renderVideo();
  },

  get media() {
    return (this.status && this.status.media) || { map: {}, camera: {} };
  },
  get mode() {
    return (this.kind === 'demo') ? 'demo' : ((this.status && this.status.mode) || 'unknown');
  },

  renderMode() {
    const m = this.mode;
    const b = this.$('mode-badge');
    b.className = 'badge';
    const map = { real: ['badge-real', '实车'], sim: ['badge-sim', '仿真'], demo: ['badge-demo', '演示'] };
    const [cls, txt] = map[m] || ['badge-unknown', '未知'];
    b.classList.add(cls); b.textContent = txt;
    this.$('ov-mode').textContent = map[m] ? map[m][1] : '未知';
    this.$('ov-addr').textContent =
      this.kind === 'demo' ? '演示（本地模拟）' : (this.baseUrl || '本机服务（同源）');
    this.$('set-mode').textContent = map[m] ? map[m][1] : '未知';
  },

  sourceRow(name, state, label) {
    const dot = state === 'ok' ? 'dot-ok' : state === 'err' ? 'dot-err' : state === 'warn' ? 'dot-warn' : 'dot-unknown';
    return `<div class="src-row"><span class="src-dot ${dot}"></span><span class="src-name">${name}</span><span class="src-state ${state}">${label}</span></div>`;
  },

  renderSources() {
    const caps = this.caps || DEFAULT_CAPABILITIES;
    const media = this.media;
    let rows = '';

    // 地图
    if (caps.mapView !== false && media.map && media.map.exists) {
      rows += this.sourceRow('地图', 'ok', '可用');
    } else if (caps.mapView !== false) {
      rows += this.sourceRow('地图', 'warn', '无图（未接入/未拉到）');
    } else {
      rows += this.sourceRow('地图', 'unknown', '未接入');
    }
    // 雷达: 无程序化新鲜度判据, 只能靠画面左上角 scan 戳
    rows += this.sourceRow('雷达', 'unknown', '未知（以地图画面左上角 scan 戳为准）');
    // 定位
    rows += this.sourceRow('定位', 'unknown', '未接入（仅 ROS /amcl_pose）');
    // 视频
    if (caps.camera !== false && media.camera && media.camera.exists) {
      rows += this.sourceRow('视频', 'ok', '已出图（非直播判定）');
    } else if (caps.camera !== false) {
      rows += this.sourceRow('视频', 'warn', '未出帧/未接入');
    } else {
      rows += this.sourceRow('视频', 'unknown', '未接入');
    }
    this.$('ov-sources').innerHTML = rows;
  },

  renderTask() {
    // 任务器无 HTTP 通路, 状态里没有任务信息 => 诚实显示"无/未接入"
    const t = (this.status && this.status.task) || null;
    this.$('ov-task').textContent = (t && t.current) || '无（任务器未接入）';
    this.$('task-current').textContent = (t && t.current) || '无（任务器未接入）';
    this.$('ov-result').textContent = (t && t.lastResult) || '无';
    this.$('task-result').textContent = (t && t.lastResult) || '无';
  },

  pushAnomaly(level, text) {
    const now = new Date();
    const stamp = now.toLocaleTimeString('zh-CN', { hour12: false });
    // 相同文本不重复堆, 只更新时间
    const existing = this.anomalies.find(a => a.text === text);
    if (existing) { existing.time = stamp; return; }
    this.anomalies.unshift({ level, text, time: stamp });
    if (this.anomalies.length > 8) this.anomalies.length = 8;
  },

  renderAnomalies() {
    if (this.kind !== 'demo' && this.connState === 'offline') {
      this.pushAnomaly('warn', '后端未连接');
    }
    if (this.anomalies.length === 0) {
      this.$('ov-anomalies').innerHTML = '<div class="anomaly">无</div>';
      return;
    }
    this.$('ov-anomalies').innerHTML = this.anomalies.map(a =>
      `<div class="anomaly ${a.level === 'err' ? 'lv-err' : 'lv-warn'}"><span class="txt">${a.text}</span><div class="t">${a.time}</div></div>`
    ).join('');
  },

  renderDiag() {
    const m = this.media;
    const lines = [
      'kind=' + this.kind,
      'connState=' + this.connState,
      'baseUrl=' + (this.baseUrl || '(同源)'),
      'mode=' + this.mode,
      'map.exists=' + !!(m.map && m.map.exists) + ' mtime=' + (m.map && m.map.mtime ? new Date(m.map.mtime * 1000).toLocaleTimeString('zh-CN', { hour12: false }) : 'null') + ' genTime=' + (m.map && m.map.genTime !== null ? m.map.genTime : '未知'),
      'camera.exists=' + !!(m.camera && m.camera.exists) + ' genTime=' + (m.camera && m.camera.genTime !== null ? m.camera.genTime : '未知'),
      'lastStatusOk=' + (this.lastStatusOkAt ? new Date(this.lastStatusOkAt).toLocaleTimeString('zh-CN', { hour12: false }) : '从未'),
      'caps=' + JSON.stringify(this.caps || null),
    ];
    this.$('ov-diag').textContent = lines.join('\n');
  },

  /* ---------------- 地图 ---------------- */
  bindMap() {
    const stage = this.$('map-stage');
    const img = this.$('map-img');
    this.$('map-zoom-in').addEventListener('click', () => this.mapZoom(1.25));
    this.$('map-zoom-out').addEventListener('click', () => this.mapZoom(0.8));
    this.$('map-fit').addEventListener('click', () => this.fitMap());
    this.$('map-fullscreen').addEventListener('click', () => this.toggleFullscreen(stage));
    this.$('map-legend-toggle').addEventListener('click', () => this.$('map-legend').classList.toggle('hidden'));

    img.addEventListener('load', () => {
      this.mapLoadedOnce = true;
      this.lastStatusOkAt = Date.now();
      this.$('map-lastfetch').textContent = new Date().toLocaleTimeString('zh-CN', { hour12: false });
      if (!this.mx.fitted) this.fitMap();
      this.$('map-stale').classList.add('hidden');
    });
    img.addEventListener('error', () => {
      this.$('map-stale').classList.remove('hidden');
    });

    // 触摸/鼠标 平移 + 双指缩放
    const pointers = new Map();
    let pinching = false, pinch0 = 0, pinchScale0 = 1, pinchMid = null;

    stage.addEventListener('pointerdown', (e) => {
      stage.setPointerCapture && stage.setPointerCapture(e.pointerId);
      pointers.set(e.pointerId, { x: e.clientX, y: e.clientY });
      if (pointers.size === 2) {
        pinching = true;
        const [a, b] = [...pointers.values()];
        pinch0 = Math.hypot(a.x - b.x, a.y - b.y);
        pinchScale0 = this.mx.scale;
        pinchMid = this.midpoint(stage, a, b);
      } else {
        img.classList.add('dragging');
      }
    });
    stage.addEventListener('pointermove', (e) => {
      if (!pointers.has(e.pointerId)) return;
      const prev = pointers.get(e.pointerId);
      const dx = e.clientX - prev.x, dy = e.clientY - prev.y;
      if (pinching) {
        pointers.set(e.pointerId, { x: e.clientX, y: e.clientY });
        const [a, b] = [...pointers.values()];
        const d = Math.hypot(a.x - b.x, a.y - b.y);
        if (pinch0 > 0) {
          const mid = this.midpoint(stage, a, b);
          this.zoomAtStage(mid.x, mid.y, Math.min(8, Math.max(0.3, pinchScale0 * d / pinch0)));
        }
        return;
      }
      this.mx.tx += dx; this.mx.ty += dy;
      pointers.set(e.pointerId, { x: e.clientX, y: e.clientY });
      this.applyMap();
    });
    const up = (e) => {
      pointers.delete(e.pointerId);
      if (pointers.size < 2) { pinching = false; }
      if (pointers.size === 0) img.classList.remove('dragging');
    };
    stage.addEventListener('pointerup', up);
    stage.addEventListener('pointercancel', up);
    stage.addEventListener('wheel', (e) => {
      e.preventDefault();
      const r = stage.getBoundingClientRect();
      const px = e.clientX - r.left, py = e.clientY - r.top;
      this.mapZoomAt(px, py, e.deltaY < 0 ? 1.15 : 1 / 1.15);
    }, { passive: false });
  },

  midpoint(stage, a, b) {
    const r = stage.getBoundingClientRect();
    return { x: (a.x + b.x) / 2 - r.left, y: (a.y + b.y) / 2 - r.top };
  },

  mapZoom(factor) {
    const stage = this.$('map-stage');
    const r = stage.getBoundingClientRect();
    this.mapZoomAt(r.width / 2, r.height / 2, factor);
  },
  mapZoomAt(px, py, factor) {
    this.zoomAtStage(px, py, Math.min(8, Math.max(0.3, this.mx.scale * factor)));
  },
  zoomAtStage(px, py, newScale) {
    const old = this.mx.scale;
    const cx = (px - this.mx.tx) / old;
    const cy = (py - this.mx.ty) / old;
    this.mx.tx = px - cx * newScale;
    this.mx.ty = py - cy * newScale;
    this.mx.scale = newScale;
    this.applyMap();
  },
  fitMap() {
    const stage = this.$('map-stage'), img = this.$('map-img');
    if (!img.naturalWidth) return;
    const s = Math.min(stage.clientWidth / img.naturalWidth, stage.clientHeight / img.naturalHeight);
    this.mx.scale = s;
    this.mx.tx = (stage.clientWidth - img.naturalWidth * s) / 2;
    this.mx.ty = (stage.clientHeight - img.naturalHeight * s) / 2;
    this.mx.fitted = true;
    this.applyMap();
  },
  applyMap() {
    const img = this.$('map-img');
    img.style.transform = `translate(${this.mx.tx}px, ${this.mx.ty}px) scale(${this.mx.scale})`;
  },

  renderMap() {
    const caps = this.caps || DEFAULT_CAPABILITIES;
    const map = this.media.map;
    const stage = this.$('map-stage'), img = this.$('map-img'), empty = this.$('map-empty');
    if (this.kind === 'demo') {
      empty.classList.add('hidden');
      img.classList.remove('hidden');
      this.setMapSrc(this.adapter.mapImageUrl());
      this.$('map-gentime').textContent = '演示数据';
      return;
    }
    // 地图元数据未通过 HTTP 提供 => 明确告知无法点图导航
    this.$('map-gentime').textContent = (map && map.genTime !== null)
      ? new Date(map.genTime * 1000).toLocaleString('zh-CN')
      : '未知（以画面左上角 scan/tf 戳为准）';
    if (caps.mapView === false || !map || !map.exists) {
      empty.classList.remove('hidden');
      img.classList.add('hidden');
      return;
    }
    empty.classList.add('hidden');
    img.classList.remove('hidden');
    if (map.mtime !== this.mapLastMtime) {
      this.mapLastMtime = map.mtime;
      this.setMapSrc(this.adapter.mapImageUrl(Date.now()));
    }
  },
  setMapSrc(url) {
    const img = this.$('map-img');
    if (img.dataset.src !== url) { img.dataset.src = url; img.src = url; }
  },

  /* ---------------- 视频 ---------------- */
  bindVideo() {
    this.$('vid-start').addEventListener('click', () => this.startWatching());
    this.$('vid-stop').addEventListener('click', () => this.stopWatching());
    this.$('vid-reconnect').addEventListener('click', () => this.reconnectVideo());
    this.$('vid-fullscreen').addEventListener('click', () => this.toggleFullscreen(this.$('vid-stage')));
    this.$('vid-img').addEventListener('load', () => {
      this.$('vid-empty').classList.add('hidden');
    });
    this.$('vid-img').addEventListener('error', () => {
      this.setVidStatus('warn', '画面加载失败，可能尚未出帧');
    });
  },

  startWatching() {
    if (this.kind === 'demo') { this.watching = true; this.syncVideoUi(); return; }
    const media = this.media;
    if (this.caps && this.caps.camera === false) {
      this.setVidStatus('err', '后端声明相机未接入');
      return;
    }
    if (!media.camera || !media.camera.exists) {
      this.setVidStatus('warn', '尚未出帧（车机暂无 camera.png）');
      return;
    }
    this.watching = true;
    this.camLastMtime = media.camera.mtime;
    this.camLastChangeAt = Date.now();
    this.setVidSrc(this.adapter.cameraImageUrl(Date.now()));
    this.setVidStatus('ok', '观看中（PNG 轮询，非直播判定）');
    this.syncVideoUi();
  },

  stopWatching() {
    this.watching = false;
    // 停止轮询并在本会话内释放图引用; 车机侧渲染器不受影响
    this.$('vid-img').src = '';
    this.$('vid-empty').classList.remove('hidden');
    this.$('vid-img').classList.add('hidden');
    this.setVidStatus('unknown', '已停止观看（已释放本地连续拉图）');
    this.syncVideoUi();
  },

  reconnectVideo() {
    this.stopWatching();
    this.startWatching();
  },

  syncVideoUi() {
    this.$('vid-start').classList.toggle('hidden', this.watching);
    this.$('vid-stop').classList.toggle('hidden', !this.watching);
    this.$('vid-reconnect').classList.toggle('hidden', !this.watching);
    this.$('vid-img').classList.toggle('hidden', !this.watching);
  },

  renderVideo() {
    const caps = this.caps || DEFAULT_CAPABILITIES;
    const cam = this.media.camera;
    if (this.kind === 'demo') {
      if (this.watching) {
        this.setVidSrc(this.adapter.cameraImageUrl());
        this.$('vid-empty').classList.add('hidden');
        this.$('vid-img').classList.remove('hidden');
        this.setVidStatus('ok', '演示画面（模拟数据，非实车）');
      }
      return;
    }
    this.$('vid-metrics').textContent = '不可测（PNG 轮询，无流参数）';
    if (caps.camera === false) { this.$('vid-start').disabled = true; return; }
    this.$('vid-start').disabled = false;
    if (this.watching) {
      // 断流判定: 观看中 camera.png mtime 长时间不前进 => 已中断
      if (cam && cam.exists && cam.mtime !== this.camLastMtime) {
        this.camLastMtime = cam.mtime;
        this.camLastChangeAt = Date.now();
        this.setVidSrc(this.adapter.cameraImageUrl(Date.now()));
        this.setVidStatus('ok', '观看中（最近更新 ' + new Date(cam.mtime * 1000).toLocaleTimeString('zh-CN', { hour12: false }) + '）');
      } else if (this.camLastChangeAt && Date.now() - this.camLastChangeAt > 6000) {
        this.setVidStatus('warn', '已中断：画面超过 6 秒未更新');
      }
    }
  },
  setVidSrc(url) {
    const img = this.$('vid-img');
    if (img.dataset.src !== url) { img.dataset.src = url; img.src = url; }
  },
  setVidStatus(cls, txt) {
    const el = this.$('vid-status');
    el.className = 'vid-status ' + cls;
    el.textContent = txt;
  },

  /* ---------------- 设置 ---------------- */
  bindSettings() {
    this.$('conn-connect').addEventListener('click', () => {
      const addr = this.$('backend-addr').value.trim();
      this.baseUrl = addr;
      localStorage.setItem('hwcart_console_base', addr);
      this.saveRecent(addr);
      this.useHttp(addr);
      this.connect();
    });
    this.$('conn-disconnect').addEventListener('click', () => this.disconnect());
    this.$('conn-retry').addEventListener('click', () => this.connect());
  },

  bindDemo() {
    this.$('demo-on').addEventListener('click', () => this.confirm(
      '进入演示模式将切换到本地模拟数据（地图/相机/状态均为占位），不连接任何真实后端与控制接口。确认进入？',
      () => { this.useDemo(); this.setConn('online'); this.onStatus(); }
    ));
    this.$('demo-off').addEventListener('click', () => {
      this.useHttp(this.baseUrl);
      this.connect();
    });
  },

  bindVisibility() {
    document.addEventListener('visibilitychange', () => {
      if (document.hidden && this.watching) {
        // 切后台/锁屏: 停止连续拉图, 不自动恢复(用户回来需手动再点观看)
        this.stopWatching();
      }
    });
    window.addEventListener('beforeunload', () => this.stopPolling());
    window.addEventListener('pagehide', () => this.stopPolling());
  },

  loadRecent() {
    try { this.recent = JSON.parse(localStorage.getItem('hwcart_console_recent') || '[]'); }
    catch (e) { this.recent = []; }
  },
  saveRecent(addr) {
    if (!addr) return;
    this.recent = [addr, ...this.recent.filter(a => a !== addr)].slice(0, 6);
    localStorage.setItem('hwcart_console_recent', JSON.stringify(this.recent));
    this.renderRecent();
  },
  renderRecent() {
    const list = this.$('recent-list');
    if (!this.recent.length) { list.textContent = '（无）'; return; }
    list.innerHTML = this.recent.map(a =>
      `<div class="recent-item" data-addr="${a}">${a}<span class="del" data-del="${a}">删除</span></div>`
    ).join('');
    list.querySelectorAll('.recent-item').forEach(el => {
      el.addEventListener('click', (e) => {
        if (e.target.dataset.del) {
          this.recent = this.recent.filter(x => x !== e.target.dataset.del);
          localStorage.setItem('hwcart_console_recent', JSON.stringify(this.recent));
          this.renderRecent();
          return;
        }
        this.$('backend-addr').value = el.dataset.addr;
      });
    });
  },

  /* ---------------- 导航 ---------------- */
  bindNav() {
    document.querySelectorAll('.nav-item').forEach(btn => {
      btn.addEventListener('click', () => this.showView(btn.dataset.view));
    });
    this.$('ov-goto-settings').addEventListener('click', () => this.showView('settings'));
  },
  showView(v) {
    this.view = v;
    document.querySelectorAll('.nav-item').forEach(b => b.classList.toggle('active', b.dataset.view === v));
    document.querySelectorAll('.view').forEach(s => s.classList.add('hidden'));
    this.$('view-' + v).classList.remove('hidden');
    // 离开视频页即停止连续拉图(释放本地连接, 避免后台多路拉流)
    if (v !== 'video' && this.watching) this.stopWatching();
    // 演示模式下打开视频页自动起本地模拟画面(无真实拉流, 便于预览)
    if (v === 'video' && this.kind === 'demo' && !this.watching) this.startWatching();
  },

  /* ---------------- 工具 ---------------- */
  toggleFullscreen(el) {
    if (!document.fullscreenElement) {
      (el.requestFullscreen || el.webkitRequestFullscreen || function () {}).call(el);
    } else {
      (document.exitFullscreen || document.webkitExitFullscreen || function () {}).call(document);
    }
  },

  confirm(msg, onOk) {
    this.$('confirm-msg').textContent = msg;
    this.$('confirm-backdrop').classList.remove('hidden');
    const ok = this.$('confirm-ok'), cancel = this.$('confirm-cancel');
    const close = (doOk) => {
      this.$('confirm-backdrop').classList.add('hidden');
      ok.onclick = cancel.onclick = null;
      if (doOk) onOk();
    };
    ok.onclick = () => close(true);
    cancel.onclick = () => close(false);
  },
};

document.addEventListener('DOMContentLoaded', () => App.init());