'use strict';
/*
 * 数据适配层 —— 界面与后端地址/来源解耦。
 *
 * 界面只依赖 Backend 这个抽象接口；具体后端只有两套：
 *   HttpBackend : 指向 serve.py 这套只读本地服务(同源或跨域)；
 *   DemoBackend : 纯前端模拟数据(显式"演示模式", 不连接任何控制)。
 * 以后若接入真实 HTTP/WS 控制后端(导航/停止), 只在这里新增适配器,
 * 不改 UI 代码；控制类能力由 capabilities 声明决定按钮是否可用。
 *
 * 接口约定(README / INTERFACE_MAP.md 有完整对照):
 *   getCapabilities() -> Capabilities    后端能力声明
 *   getStatus()        -> Status         状态与媒体文件新鲜度快照
 *   mapImageUrl(ts)    -> string|null    地图图 URL(带缓存戳)
 *   cameraImageUrl(ts) -> string|null    相机图 URL
 *
 * Capabilities: { mapView, camera, navigation, multipoint, slam, softwareStop,
 *                 mapLayers, mapMetadata, videoKind }
 * Status: { serverTime, mode, media:{ map:{exists,mtime,size,genTime},
 *                                    camera:{exists,mtime,size,genTime} } }
 */

const DEFAULT_CAPABILITIES = {
  mapView: true,
  camera: true,
  navigation: false,
  multipoint: false,
  slam: false,
  softwareStop: false,
  mapLayers: false,
  mapMetadata: false,
  videoKind: 'png-poll',
};

class Backend {
  /* eslint-disable no-unused-vars */
  async getCapabilities() { throw new Error('未实现'); }
  async getStatus() { throw new Error('未实现'); }
  mapImageUrl(ts) { return null; }
  cameraImageUrl(ts) { return null; }
  /* eslint-enable no-unused-vars */
}

/* ---- HTTP 后端(默认同源; 可指跨域地址) ---- */
class HttpBackend extends Backend {
  constructor(baseUrl) {
    super();
    this.base = (baseUrl || '').replace(/\/+$/, '');
  }

  _abs(path) { return this.base + path; }

  async _json(path, timeoutMs = 4000) {
    const ctrl = new AbortController();
    const t = setTimeout(() => ctrl.abort(), timeoutMs);
    try {
      const res = await fetch(this._abs(path), {
        method: 'GET', cache: 'no-store', signal: ctrl.signal, headers: { Accept: 'application/json' },
      });
      if (!res.ok) throw new Error('HTTP ' + res.status);
      return await res.json();
    } finally {
      clearTimeout(t);
    }
  }

  async getCapabilities() {
    try {
      const c = await this._json('/api/capabilities');
      return Object.assign({}, DEFAULT_CAPABILITIES, c || {});
    } catch (e) {
      // 能力探测失败 => 保守地全关(不假装功能可用), 由上层显示"服务不可用"
      return Object.assign({}, DEFAULT_CAPABILITIES, {
        mapView: false, camera: false, navigation: false, multipoint: false,
        slam: false, softwareStop: false, mapLayers: false, mapMetadata: false,
      });
    }
  }

  async getStatus() { return await this._json('/api/status', 4000); }

  mapImageUrl(ts) { return this._abs('/media/map.png?ts=' + ts); }
  cameraImageUrl(ts) { return this._abs('/media/camera.png?ts=' + ts); }
}

/* ---- 演示后端(纯前端) ---- */
class DemoBackend extends Backend {
  async getCapabilities() {
    return Object.assign({}, DEFAULT_CAPABILITIES, {
      mapView: true, camera: true, videoKind: 'demo',
    });
  }

  async getStatus() {
    return {
      serverTime: Date.now() / 1000,
      mode: 'demo',
      media: {
        map: { exists: true, mtime: Date.now() / 1000, size: null, genTime: Date.now() / 1000 },
        camera: { exists: true, mtime: Date.now() / 1000, size: null, genTime: Date.now() / 1000 },
      },
    };
  }

  mapImageUrl() { return _demoMapDataUri(); }
  cameraImageUrl() { return _demoCameraDataUri(); }
}

/* 生成一张纯前端模拟地图(占位), 明确标注"演示数据" */
function _demoMapDataUri() {
  const w = 480, h = 480, cell = 20;
  const cv = document.createElement('canvas');
  cv.width = w; cv.height = h;
  const ctx = cv.getContext('2d');
  ctx.fillStyle = '#c8c8c8'; ctx.fillRect(0, 0, w, h);   // 未知 灰
  ctx.fillStyle = '#ffffff'; ctx.fillRect(0, 0, w, h);   // 自由 白
  ctx.strokeStyle = '#000'; ctx.lineWidth = 3;           // 墙 黑
  // 外墙
  ctx.strokeRect(5, 5, w - 10, h - 10);
  // 内部隔墙
  ctx.strokeRect(cell * 8, cell * 4, cell * 6, cell * 3);
  ctx.beginPath(); ctx.moveTo(0, cell * 12); ctx.lineTo(cell * 16, cell * 12); ctx.stroke();
  // 轨迹 蓝
  ctx.strokeStyle = '#0000ff'; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(40, h - 40);
  ctx.bezierCurveTo(120, h - 160, 260, h - 80, 320, h - 240);
  ctx.stroke();
  // 车 绿三角
  ctx.fillStyle = '#00c800';
  ctx.beginPath(); ctx.moveTo(320, h - 260); ctx.lineTo(292, h - 228); ctx.lineTo(320, h - 220); ctx.closePath(); ctx.fill();
  // 雷达 红点
  ctx.fillStyle = '#ff0000';
  for (let a = 0; a < 360; a += 8) {
    const r = 130 + (Math.sin(a * 0.11) * 25);
    ctx.fillRect(300 + Math.cos(a * Math.PI / 180) * r * 0.6, (h - 240) + Math.sin(a * Math.PI / 180) * r * 0.6, 2, 2);
  }
  // 水印
  ctx.fillStyle = 'rgba(200,150,0,0.9)';
  ctx.font = 'bold 28px sans-serif';
  ctx.fillText('演示数据 DEMO', w / 2 - 150, 40);
  return cv.toDataURL('image/png');
}

/* 生成一张纯前端模拟相机画面(占位) */
function _demoCameraDataUri() {
  const w = 320, h = 240;
  const cv = document.createElement('canvas');
  cv.width = w; cv.height = h;
  const ctx = cv.getContext('2d');
  ctx.fillStyle = '#20303f'; ctx.fillRect(0, 0, w, h);
  ctx.fillStyle = '#4a6070';
  for (let i = 0; i < 12; i++) {
    const x = (i * 53) % w, y = (i * 71) % h;
    ctx.fillRect(x, y, 40 + i * 3, 40 + i * 3);
  }
  ctx.fillStyle = 'rgba(255,255,255,0.85)';
  ctx.font = '20px sans-serif';
  ctx.fillText('演示相机画面', 20, 60);
  ctx.font = '14px sans-serif';
  ctx.fillText('DEMO (非实车)', 20, 90);
  return cv.toDataURL('image/png');
}

/* 供其它模块按名字构造后端 */
function makeBackend(kind, baseUrl) {
  if (kind === 'demo') return new DemoBackend();
  return new HttpBackend(baseUrl);
}