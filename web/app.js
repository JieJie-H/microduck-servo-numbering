// MicroDuck 舵机编号向导
// 不依赖 Three.js：用浏览器自带的 WebGL2 渲染主仓 model.json / meshes.bin。
// 所有关节在零位（2048）姿态下显示，编号标签是叠加在 3D 画面上的 HTML。

const $ = (s) => document.querySelector(s);
window.addEventListener('error', (e) => {
  const el = document.querySelector('#error');
  if (el) {
    el.hidden = false;
    el.textContent = `JS 错误：${e.message}（${e.filename}:${e.lineno}）`;
  }
});
const canvas = $('#view');
const overlay = $('#labels');
const loadingEl = $('#loading');
const errorEl = $('#error');

const SIDE_LABEL = { right: '右腿', left: '左腿', head: '颈头' };
const SIDE_RANGE = { right: '10–14', left: '20–24', head: '30–34' };

const state = {
  plan: [],
  done: new Set(),
  selected: null,
  ports: [],
  port: null,
  scanned: null,        // [{id,...}] 最近一次扫描结果
  busy: false,
  showNames: false,
  autoRotate: false,
  joints: {},           // id -> {world:[x,y,z], name}
};

let gl = null;
let duck = null;        // {items, joints}
let lastVP = null;
let dirty = true;
let tween = null;
let lastT = performance.now();
const pins = {};        // id -> element
const names = {};       // id -> element

// ---------------------------------------------------------------- 小工具

const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
const lerp = (a, b, t) => a + (b - a) * t;
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const norm = (a) => { const l = Math.hypot(...a) || 1; return [a[0] / l, a[1] / l, a[2] / l]; };
const v3 = (x, y, z) => [x, y, z];

function status(msg, isError = false) {
  $('#status-text').textContent = msg;
  $('#statusbar').classList.toggle('error', isError);
  if (!isError) $('#statusbar').classList.remove('error');
}

function showError(msg) {
  errorEl.hidden = false;
  errorEl.textContent = msg;
  status(msg, true);
  clearTimeout(showError._t);
  showError._t = setTimeout(() => { errorEl.hidden = true; }, 8000);
}

function setBusy(b, msg) {
  state.busy = b;
  document.body.classList.toggle('busy', b);
  $('#statusbar').classList.toggle('busy', b);
  if (msg) status(msg);
  updateCard();
}

async function api(path, opts = {}) {
  const timeout = opts.timeout || 60000;
  delete opts.timeout;
  const ctrl = new AbortController();
  const timer = setTimeout(() => ctrl.abort(), timeout);
  try {
    const r = await fetch(path, {
      headers: { 'Content-Type': 'application/json' },
      signal: ctrl.signal,
      ...opts,
    });
    let data;
    try { data = await r.json(); } catch { throw new Error(`HTTP ${r.status}`); }
    if (!r.ok && data.error !== 'already_done') throw new Error(data.error || `HTTP ${r.status}`);
    return data;
  } catch (e) {
    if (e.name === 'AbortError') throw new Error(`请求超时（${Math.round(timeout / 1000)} s）`);
    throw e;
  } finally {
    clearTimeout(timer);
  }
}

// ---------------------------------------------------------------- 矩阵

function mat4Mul(a, b) {
  const o = new Float32Array(16);
  for (let c = 0; c < 4; c++) {
    for (let r = 0; r < 4; r++) {
      o[c * 4 + r] = a[r] * b[c * 4] + a[4 + r] * b[c * 4 + 1]
                   + a[8 + r] * b[c * 4 + 2] + a[12 + r] * b[c * 4 + 3];
    }
  }
  return o;
}

function mat4FromQuatPos(q, p) {
  let [w, x, y, z] = q;
  const n = Math.hypot(w, x, y, z) || 1;
  w /= n; x /= n; y /= n; z /= n;
  const xx = x * x, yy = y * y, zz = z * z;
  const xy = x * y, xz = x * z, yz = y * z;
  const wx = w * x, wy = w * y, wz = w * z;
  const m = new Float32Array(16);
  m[0] = 1 - 2 * (yy + zz); m[1] = 2 * (xy + wz); m[2] = 2 * (xz - wy);
  m[4] = 2 * (xy - wz); m[5] = 1 - 2 * (xx + zz); m[6] = 2 * (yz + wx);
  m[8] = 2 * (xz + wy); m[9] = 2 * (yz - wx); m[10] = 1 - 2 * (xx + yy);
  m[12] = p[0]; m[13] = p[1]; m[14] = p[2]; m[15] = 1;
  return m;
}

function mat4LookAt(eye, center, up) {
  const f = norm(sub(center, eye));
  const s = norm(cross(f, up));
  const u = cross(s, f);
  const m = new Float32Array(16);
  m[0] = s[0]; m[1] = u[0]; m[2] = -f[0];
  m[4] = s[1]; m[5] = u[1]; m[6] = -f[1];
  m[8] = s[2]; m[9] = u[2]; m[10] = -f[2];
  m[12] = -dot(s, eye); m[13] = -dot(u, eye); m[14] = dot(f, eye); m[15] = 1;
  return m;
}

function mat4Perspective(fovy, aspect, near, far) {
  const f = 1 / Math.tan(fovy / 2);
  const m = new Float32Array(16);
  m[0] = f / aspect;
  m[5] = f;
  m[10] = (far + near) / (near - far);
  m[11] = -1;
  m[14] = (2 * far * near) / (near - far);
  return m;
}

function transformPoint(m, p) {
  return [
    m[0] * p[0] + m[4] * p[1] + m[8] * p[2] + m[12],
    m[1] * p[0] + m[5] * p[1] + m[9] * p[2] + m[13],
    m[2] * p[0] + m[6] * p[1] + m[10] * p[2] + m[14],
  ];
}

// ---------------------------------------------------------------- WebGL

const MESH_VS = `#version 300 es
in vec3 aPos;
uniform mat4 uModel;
uniform mat4 uViewProj;
out vec3 vWorld;
void main() {
  vec4 w = uModel * vec4(aPos, 1.0);
  vWorld = w.xyz;
  gl_Position = uViewProj * w;
}`;

const MESH_FS = `#version 300 es
precision highp float;
in vec3 vWorld;
uniform vec4 uColor;
uniform vec3 uLight;
out vec4 outColor;
void main() {
  vec3 n = normalize(cross(dFdx(vWorld), dFdy(vWorld)));
  if (!gl_FrontFacing) n = -n;
  vec3 l = normalize(uLight);
  float key = max(dot(n, l), 0.0);
  float fill = max(dot(n, normalize(vec3(-0.45, 0.55, 0.35))), 0.0);
  vec3 col = uColor.rgb * (0.30 + 0.72 * key + 0.18 * fill);
  col += vec3(0.035, 0.05, 0.042) * (0.5 + 0.5 * n.z);
  outColor = vec4(pow(col, vec3(1.0 / 1.9)), uColor.a);
}`;

const LINE_VS = `#version 300 es
in vec3 aPos;
uniform mat4 uViewProj;
void main() { gl_Position = uViewProj * vec4(aPos, 1.0); }`;

const LINE_FS = `#version 300 es
precision highp float;
uniform vec4 uColor;
out vec4 outColor;
void main() { outColor = uColor; }`;

function compile(type, src) {
  const s = gl.createShader(type);
  gl.shaderSource(s, src);
  gl.compileShader(s);
  if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) {
    throw new Error('着色器编译失败：' + gl.getShaderInfoLog(s));
  }
  return s;
}

function program(vs, fs) {
  const p = gl.createProgram();
  gl.attachShader(p, compile(gl.VERTEX_SHADER, vs));
  gl.attachShader(p, compile(gl.FRAGMENT_SHADER, fs));
  gl.linkProgram(p);
  if (!gl.getProgramParameter(p, gl.LINK_STATUS)) {
    throw new Error('着色器链接失败：' + gl.getProgramInfoLog(p));
  }
  return p;
}

let meshProg, lineProg, meshLoc, lineLoc, gridBuf;

function initGL() {
  gl = canvas.getContext('webgl2', { antialias: true, alpha: false, powerPreference: 'high-performance' });
  if (!gl) {
    throw new Error('这个浏览器没有 WebGL2。换 Chrome / Edge，或打开硬件加速。');
  }
  meshProg = program(MESH_VS, MESH_FS);
  lineProg = program(LINE_VS, LINE_FS);
  meshLoc = {
    aPos: gl.getAttribLocation(meshProg, 'aPos'),
    uModel: gl.getUniformLocation(meshProg, 'uModel'),
    uViewProj: gl.getUniformLocation(meshProg, 'uViewProj'),
    uColor: gl.getUniformLocation(meshProg, 'uColor'),
    uLight: gl.getUniformLocation(meshProg, 'uLight'),
  };
  lineLoc = {
    aPos: gl.getAttribLocation(lineProg, 'aPos'),
    uViewProj: gl.getUniformLocation(lineProg, 'uViewProj'),
    uColor: gl.getUniformLocation(lineProg, 'uColor'),
  };
  gridBuf = gl.createBuffer();
  const verts = [];
  const n = 10, step = 0.02, ext = n * step;
  for (let i = -n; i <= n; i++) {
    const v = i * step;
    verts.push(-ext, v, -0.0006, ext, v, -0.0006);
    verts.push(v, -ext, -0.0006, v, ext, -0.0006);
  }
  gl.bindBuffer(gl.ARRAY_BUFFER, gridBuf);
  gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(verts), gl.STATIC_DRAW);
}

function uploadMesh(pos, idx, indexBytes) {
  const vbo = gl.createBuffer();
  gl.bindBuffer(gl.ARRAY_BUFFER, vbo);
  gl.bufferData(gl.ARRAY_BUFFER, pos, gl.STATIC_DRAW);
  const ibo = gl.createBuffer();
  gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, ibo);
  gl.bufferData(gl.ELEMENT_ARRAY_BUFFER, idx, gl.STATIC_DRAW);
  return { vbo, ibo, count: idx.length, type: indexBytes === 2 ? gl.UNSIGNED_SHORT : gl.UNSIGNED_INT };
}

// ---------------------------------------------------------------- 相机

const cam = {
  target: [0, 0, 0.10],
  yaw: -0.72,
  pitch: 0.22,
  dist: 0.55,
  home: { target: [0, 0, 0.10], yaw: -0.72, pitch: 0.22, dist: 0.55 },
};

function cameraEye() {
  const cp = Math.cos(cam.pitch), sp = Math.sin(cam.pitch);
  return [
    cam.target[0] + cam.dist * cp * Math.cos(cam.yaw),
    cam.target[1] + cam.dist * cp * Math.sin(cam.yaw),
    cam.target[2] + cam.dist * sp,
  ];
}

function computeVP() {
  const w = canvas.clientWidth || 1, h = canvas.clientHeight || 1;
  const view = mat4LookAt(cameraEye(), cam.target, [0, 0, 1]);
  const proj = mat4Perspective(40 * Math.PI / 180, w / h, 0.01, 10);
  return mat4Mul(proj, view);
}

function projectPoint(p) {
  const m = lastVP;
  if (!m) return null;
  const x = p[0], y = p[1], z = p[2];
  const cx = m[0] * x + m[4] * y + m[8] * z + m[12];
  const cy = m[1] * x + m[5] * y + m[9] * z + m[13];
  const cw = m[3] * x + m[7] * y + m[11] * z + m[15];
  if (cw <= 0.0001) return null;
  const nx = cx / cw, ny = cy / cw;
  return {
    x: (nx * 0.5 + 0.5) * (canvas.clientWidth || 1),
    y: (-ny * 0.5 + 0.5) * (canvas.clientHeight || 1),
  };
}

// ---------------------------------------------------------------- 模型

async function loadDuck() {
  const [mj, bin] = await Promise.all([
    fetch('model/model.json').then(r => r.json()),
    fetch('model/meshes.bin').then(r => r.arrayBuffer()),
  ]);
  const meshes = {};
  for (const [name, m] of Object.entries(mj.meshes)) {
    const q = new Int16Array(bin, m.voff, m.nverts * 3);
    const pos = new Float32Array(m.nverts * 3);
    for (let i = 0; i < m.nverts; i++) {
      for (let k = 0; k < 3; k++) {
        const span = (m.bbox_max[k] - m.bbox_min[k]) || 1;
        pos[i * 3 + k] = m.bbox_min[k] + (q[i * 3 + k] + 32768) / 65535 * span;
      }
    }
    const idx = m.index_bytes === 2
      ? new Uint16Array(bin, m.ioff, m.ntris * 3)
      : new Uint32Array(bin, m.ioff, m.ntris * 3);
    meshes[name] = uploadMesh(pos, idx, m.index_bytes);
  }

  const bodyWorld = [];
  const worldOf = (i) => {
    if (bodyWorld[i]) return bodyWorld[i];
    const b = mj.bodies[i];
    const local = mat4FromQuatPos(b.quat, b.pos);
    bodyWorld[i] = b.parent < 0 ? local : mat4Mul(worldOf(b.parent), local);
    return bodyWorld[i];
  };

  const items = [];
  const joints = {};
  mj.bodies.forEach((b, i) => {
    const bw = worldOf(i);
    for (const g of b.geoms) {
      const gm = mat4Mul(bw, mat4FromQuatPos(g.quat, g.pos));
      items.push({
        mesh: meshes[g.mesh],
        model: gm,
        color: new Float32Array(g.rgba),
        transparent: g.rgba[3] < 0.999,
      });
    }
    if (b.joint && b.joint.id != null) {
      let jw = [bw[12], bw[13], bw[14]];
      // 嘴的关节在铰链上，画在铰链上会跑到头壳后上方；用下颌网格中心更好认
      if (b.joint.id === 34) {
        for (const g of b.geoms) {
          if (g.mesh === 'jaw') {
            const mb = mj.meshes.jaw;
            const c = [
              (mb.bbox_min[0] + mb.bbox_max[0]) / 2,
              (mb.bbox_min[1] + mb.bbox_max[1]) / 2,
              (mb.bbox_min[2] + mb.bbox_max[2]) / 2,
            ];
            const gm = mat4Mul(bw, mat4FromQuatPos(g.quat, g.pos));
            jw = transformPoint(gm, c);
          }
        }
      }
      joints[b.joint.id] = {
        id: b.joint.id,
        name: b.joint.name,
        world: jw,
      };
    }
  });
  return { items, joints };
}

// ---------------------------------------------------------------- 渲染

function resizeCanvas() {
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  const w = Math.max(1, canvas.clientWidth);
  const h = Math.max(1, canvas.clientHeight);
  const cw = Math.round(w * dpr), ch = Math.round(h * dpr);
  if (canvas.width !== cw || canvas.height !== ch) {
    canvas.width = cw;
    canvas.height = ch;
    dirty = true;
  }
  return { w: cw, h: ch };
}

function drawMeshItem(it) {
  gl.uniformMatrix4fv(meshLoc.uModel, false, it.model);
  gl.uniform4fv(meshLoc.uColor, it.color);
  gl.bindBuffer(gl.ARRAY_BUFFER, it.mesh.vbo);
  gl.enableVertexAttribArray(meshLoc.aPos);
  gl.vertexAttribPointer(meshLoc.aPos, 3, gl.FLOAT, false, 0, 0);
  gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, it.mesh.ibo);
  gl.drawElements(gl.TRIANGLES, it.mesh.count, it.mesh.type, 0);
}

function render() {
  if (!gl || !duck) return;
  const { w, h } = resizeCanvas();
  gl.viewport(0, 0, w, h);
  gl.clearColor(0.035, 0.050, 0.043, 1);
  gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
  gl.enable(gl.DEPTH_TEST);
  gl.disable(gl.CULL_FACE);
  gl.enable(gl.BLEND);
  gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);

  lastVP = computeVP();

  // 地面网格
  gl.useProgram(lineProg);
  gl.uniformMatrix4fv(lineLoc.uViewProj, false, lastVP);
  gl.uniform4f(lineLoc.uColor, 0.10, 0.17, 0.135, 1);
  gl.bindBuffer(gl.ARRAY_BUFFER, gridBuf);
  gl.enableVertexAttribArray(lineLoc.aPos);
  gl.vertexAttribPointer(lineLoc.aPos, 3, gl.FLOAT, false, 0, 0);
  gl.drawArrays(gl.LINES, 0, 21 * 4);

  // 舵机 / 外壳
  gl.useProgram(meshProg);
  gl.uniformMatrix4fv(meshLoc.uViewProj, false, lastVP);
  gl.uniform3f(meshLoc.uLight, 0.45, -0.55, 0.82);
  const opaque = duck.items.filter(it => !it.transparent);
  const trans = duck.items.filter(it => it.transparent);
  for (const it of opaque) drawMeshItem(it);
  if (trans.length) {
    gl.depthMask(false);
    for (const it of trans) drawMeshItem(it);
    gl.depthMask(true);
  }
}

function updateOverlay() {
  if (!lastVP) lastVP = computeVP();
  for (const j of state.plan) {
    const pin = pins[j.id];
    const name = names[j.id];
    const joint = duck && duck.joints[j.id];
    if (!joint) {
      if (pin) pin.style.display = 'none';
      if (name) name.style.display = 'none';
      continue;
    }
    const p = projectPoint(joint.world);
    if (!p) {
      if (pin) pin.style.display = 'none';
      if (name) name.style.display = 'none';
      continue;
    }
    if (pin) {
      pin.style.display = 'grid';
      pin.style.left = `${p.x}px`;
      pin.style.top = `${p.y}px`;
    }
    if (name) {
      name.style.display = state.showNames ? 'block' : 'none';
      name.style.left = `${p.x}px`;
      name.style.top = `${p.y}px`;
    }
  }
}

// ---------------------------------------------------------------- UI

function buildsGroups() {
  const groupsEl = $('#groups');
  groupsEl.innerHTML = '';
  for (const side of ['right', 'left', 'head']) {
    const items = state.plan.filter(j => j.side === side).sort((a, b) => a.id - b.id);
    if (!items.length) continue;
    const box = document.createElement('div');
    box.className = 'group';
    box.innerHTML = `<div class="group-title"><b>${SIDE_LABEL[side]}</b><span>${SIDE_RANGE[side]}</span></div>`;
    const rows = document.createElement('div');
    rows.className = 'rows';
    for (const j of items) {
      const row = document.createElement('div');
      row.className = 'row';
      row.dataset.id = j.id;
      row.innerHTML = `
        <div class="id-badge">${j.id}</div>
        <div class="row-main"><div class="row-name">${j.cn}</div><div class="row-key">${j.key}</div></div>
        <div class="row-state"><span class="dot"></span><span class="st">待编号</span></div>`;
      row.addEventListener('click', () => select(j.id));
      rows.appendChild(row);
    }
    box.appendChild(rows);
    groupsEl.appendChild(box);
  }
}

function buildPins() {
  overlay.innerHTML = '';
  for (const j of state.plan) {
    const pin = document.createElement('div');
    pin.className = 'pin';
    pin.textContent = j.id;
    pin.title = `${j.id} · ${j.cn}`;
    pin.dataset.id = j.id;
    pin.addEventListener('click', (e) => { e.stopPropagation(); select(j.id); });
    overlay.appendChild(pin);
    pins[j.id] = pin;

    const name = document.createElement('div');
    name.className = 'joint-name';
    name.textContent = `${j.id} ${j.cn}`;
    name.dataset.id = j.id;
    overlay.appendChild(name);
    names[j.id] = name;
  }
}

function updateRows() {
  document.querySelectorAll('.row').forEach(row => {
    const id = +row.dataset.id;
    const online = state.scanned && state.scanned.some(s => s.id === id);
    row.classList.toggle('done', state.done.has(id));
    row.classList.toggle('selected', state.selected === id);
    row.classList.toggle('online', !!online && !state.done.has(id));
    const st = row.querySelector('.st');
    st.textContent = state.done.has(id) ? '已编号' : online ? '在线' : '待编号';
  });
}

function updatePins() {
  for (const j of state.plan) {
    const pin = pins[j.id];
    if (!pin) continue;
    pin.classList.toggle('done', state.done.has(j.id));
    pin.classList.toggle('selected', state.selected === j.id);
    const name = names[j.id];
    if (name) {
      name.classList.toggle('selected', state.selected === j.id);
      name.classList.toggle('done', state.done.has(j.id));
    }
  }
}

function updateProgress() {
  const n = state.done.size;
  const total = state.plan.length || 15;
  $('#progress-text').textContent = `${n} / ${total}`;
  $('#progress-bar').style.width = `${Math.min(100, n / total * 100)}%`;
}

function updateCard() {
  const j = state.plan.find(x => x.id === state.selected);
  if (!j) return;
  $('#card-id').textContent = j.id;
  $('#card-side').textContent = SIDE_LABEL[j.side];
  $('#card-name').textContent = j.cn;
  $('#card-key').textContent = j.key;
  $('#card-location').textContent = j.location;
  $('#card-hint').textContent = j.hint;

  const scanned = state.scanned;
  const one = scanned && scanned.length === 1 ? scanned[0] : null;
  const same = one && one.id === j.id;
  const btn = $('#btn-assign');
  btn.disabled = state.busy || !one;
  btn.textContent = (state.done.has(j.id) || same)
    ? `重写 ID ${j.id}` : `写入 ID ${j.id}`;

  $('#check-done').checked = state.done.has(j.id);
  $('#btn-scan').disabled = state.busy;
  $('#btn-verify').disabled = state.busy;
  $('#btn-next').disabled = state.busy;
  $('#btn-ports').disabled = state.busy;
}

function renderScan(result) {
  const box = $('#scan-info');
  const st = $('#scan-status');
  if (!result.ok) {
    st.textContent = result.statusText || '没有应答';
    st.className = 'warn';
    box.innerHTML = `<div class="bad">${result.error || '没有找到舵机'}</div>`
      + (result.hint ? `<ul style="margin:6px 0 0;padding-left:18px;color:var(--dim)">${result.hint.map(h => `<li>${h}</li>`).join('')}</ul>` : '');
    return;
  }
  const servos = result.servos || [];
  if (servos.length === 0) {
    st.textContent = '没有应答';
    box.innerHTML = '<div class="bad">没有找到舵机。</div>';
    return;
  }
  if (servos.length > 1) {
    st.textContent = `找到 ${servos.length} 颗`;
    st.className = 'warn';
    box.innerHTML = `<div class="warn">总线上有 ${servos.length} 颗：${servos.map(s => s.id).join('、')}。改 ID 时只能接一颗。</div>`;
    return;
  }
  const s = servos[0];
  const target = state.selected;
  st.textContent = `找到 ID ${s.id}`;
  st.className = s.id === target ? 'ok' : 'muted';
  const bits = [];
  bits.push(`<div>当前 ID：<b>${s.id}</b>　型号：<b>${s.model || '?'}</b>　固件：${s.fw || '?'}</div>`);
  if (s.pos != null) bits.push(`<div>位置：${s.pos}　电压：${s.volt != null ? s.volt.toFixed(1) + ' V' : '?'}　温度：${s.temp != null ? s.temp + ' °C' : '?'}</div>`);
  if (s.unload != null) bits.push(`<div>保护：过压 <b>${s.unload & 1 ? '开' : '关'}</b>　过流 <b>${s.unload & 8 ? '开' : '关'}</b>　状态位：0x${(s.status ?? 0).toString(16).padStart(2, '0')}</div>`);
  if (s.id === target) {
    bits.push('<div class="ok">已经是目标 ID。断电重启后再扫描一次确认。</div>');
  } else {
    if (state.done.has(s.id)) {
      bits.push(`<div class="warn">注意：这颗现在已经是台账里完成的 ID ${s.id}（${jointName(s.id)}）；改它会让那个关节缺一颗。</div>`);
    }
    bits.push(`<div class="warn">目标 ID ${target}：点「写入 ID ${target}」把这一颗改过去。</div>`);
  }
  box.innerHTML = bits.join('');
}

function jointName(id) {
  const j = state.plan.find(x => x.id === id);
  return j ? j.cn : '';
}

function logAssign(msg, cls = '') {
  const el = $('#assign-log');
  el.innerHTML = `<span class="${cls}">${msg}</span>`;
}

function select(id, focus = true) {
  const changed = state.selected != null && state.selected !== id;
  state.selected = id;
  if (changed && state.scanned) {
    state.scanned = null;
    renderScan({ ok: false, statusText: '待重新扫描', error: '换了关节，请重新扫描当前这一颗舵机。' });
  }
  updateRows();
  updatePins();
  updateCard();
  if (focus) focusJoint(id);
  dirty = true;
}

function focusJoint(id) {
  const j = duck && duck.joints[id];
  if (!j) return;
  tween = {
    from: { target: [...cam.target], dist: cam.dist },
    to: { target: [...j.world], dist: clamp(cam.dist, 0.36, 0.52) },
    t0: performance.now(),
    dur: 420,
  };
}

function resetView() {
  tween = {
    from: { target: [...cam.target], dist: cam.dist },
    to: { target: [...cam.home.target], dist: cam.home.dist },
    t0: performance.now(),
    dur: 420,
  };
  cam.yaw = cam.home.yaw;
  cam.pitch = cam.home.pitch;
  dirty = true;
}

function nextPending() {
  return state.plan.find(j => !state.done.has(j.id));
}

function updateAll() {
  updateRows();
  updatePins();
  updateProgress();
  updateCard();
}

// ---------------------------------------------------------------- 交互

function bindUI() {
  $('#btn-next').addEventListener('click', () => {
    const j = nextPending();
    if (j) select(j.id);
    else status('15 个关节都已经编号了。');
  });
  $('#btn-reset').addEventListener('click', resetView);
  $('#btn-rotate').addEventListener('click', () => {
    state.autoRotate = !state.autoRotate;
    $('#btn-rotate').textContent = state.autoRotate ? '停止旋转' : '自动旋转';
    dirty = true;
  });
  $('#btn-labels').addEventListener('click', () => {
    state.showNames = !state.showNames;
    $('#btn-labels').textContent = state.showNames ? '隐藏关节名' : '显示关节名';
    dirty = true;
  });
  $('#btn-ports').addEventListener('click', loadPorts);
  $('#port').addEventListener('change', (e) => {
    state.port = e.target.value;
    localStorage.setItem('md-port', state.port || '');
    status(`串口：${state.port || '未选择'}`);
  });
  $('#btn-scan').addEventListener('click', doScan);
  $('#btn-assign').addEventListener('click', () => doAssign(false));
  $('#btn-verify').addEventListener('click', doVerify);
  $('#check-done').addEventListener('change', async (e) => {
    const id = state.selected;
    const done = e.target.checked;
    try {
      const r = await api('/api/mark', { method: 'POST', body: JSON.stringify({ target: id, done }) });
      state.done = new Set(r.state.done);
      updateAll();
      status(done ? `已手动标记 ID ${id} 完成` : `已取消 ID ${id} 的完成标记`);
    } catch (err) {
      e.target.checked = !done;
      showError(err.message);
    }
  });

  window.addEventListener('keydown', (e) => {
    if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;
    if (e.key !== 'ArrowDown' && e.key !== 'ArrowUp') return;
    e.preventDefault();
    const list = state.plan.map(j => j.id);
    const i = list.indexOf(state.selected);
    const n = e.key === 'ArrowDown' ? Math.min(list.length - 1, i + 1) : Math.max(0, i - 1);
    select(list[n]);
  });

  // 轨道
  let drag = null;
  canvas.addEventListener('pointerdown', (e) => {
    drag = { x: e.clientX, y: e.clientY };
    canvas.setPointerCapture(e.pointerId);
  });
  canvas.addEventListener('pointermove', (e) => {
    if (!drag) return;
    const dx = e.clientX - drag.x, dy = e.clientY - drag.y;
    drag = { x: e.clientX, y: e.clientY };
    cam.yaw -= dx * 0.008;
    cam.pitch = clamp(cam.pitch + dy * 0.006, -1.15, 1.15);
    dirty = true;
  });
  canvas.addEventListener('pointerup', () => { drag = null; });
  canvas.addEventListener('pointercancel', () => { drag = null; });
  canvas.addEventListener('wheel', (e) => {
    e.preventDefault();
    cam.dist = clamp(cam.dist * Math.exp(e.deltaY * 0.0012), 0.16, 1.15);
    dirty = true;
  }, { passive: false });
  canvas.addEventListener('contextmenu', (e) => e.preventDefault());
  window.addEventListener('resize', () => { dirty = true; });
}

// ---------------------------------------------------------------- 串口 / 动作

async function loadPorts() {
  try {
    const r = await api('/api/ports');
    state.ports = r.ports || [];
    const sel = $('#port');
    sel.innerHTML = '';
    for (const p of state.ports) {
      const o = document.createElement('option');
      o.value = p.device;
      o.textContent = `${p.device} · ${p.description}`;
      sel.appendChild(o);
    }
    const saved = localStorage.getItem('md-port');
    const pick = (saved && state.ports.some(p => p.device === saved))
      ? saved : (state.ports[0] && state.ports[0].device) || '';
    state.port = pick || null;
    sel.value = pick;
    if (!state.ports.length) status('没有发现串口：检查 URT-2 的 USB 和 CH343 驱动。', true);
    else status(`串口已就绪：${pick}`);
  } catch (e) {
    showError('列串口失败：' + e.message);
  }
}

async function doScan() {
  if (state.busy) return;
  setBusy(true, '扫描中');
  logAssign('');
  try {
    const r = await api('/api/scan', { method: 'POST', body: JSON.stringify({ port: state.port }) });
    state.scanned = r.ok ? r.servos : null;
    renderScan(r);
    updateRows();
    updateCard();
    status(r.ok ? `扫描完成：${r.servos.length} 颗` : '扫描没有应答');
  } catch (e) {
    state.scanned = null;
    showError('扫描失败：' + e.message);
  } finally {
    setBusy(false);
  }
}

async function doAssign(force) {
  const j = state.plan.find(x => x.id === state.selected);
  if (!j) return;
  if (state.busy) return;
  if (!state.scanned || state.scanned.length !== 1) {
    status('先扫描，确认总线上只有一颗。', true);
    return;
  }
  const cur = state.scanned[0].id;
  const text = `把当前 ID ${cur} 改成 ID ${j.id}（${j.cn}）？\n\n`
    + '会写 EEPROM、开过压/过流，然后 0x08 重启复核。只接这一颗。';
  if (!force && !confirm(text)) return;
  setBusy(true, '写入中');
  logAssign('');
  try {
    const r = await api('/api/assign', {
      method: 'POST',
      body: JSON.stringify({ port: state.port, target: j.id, joint: j.cn, protect: true, force }),
    });
    if (!r.ok && r.error === 'already_done') {
      setBusy(false);
      if (confirm(`台账里 ID ${j.id} 已经标记完成。仍要重写吗？`)) return doAssign(true);
      return;
    }
    if (!r.ok) {
      logAssign(`失败：${r.error || r.message || '未知错误'}`, 'bad');
      status('写入失败：' + (r.error || r.message), true);
      return;
    }
    state.done.add(j.id);
    updateAll();
    state.scanned = [{ ...state.scanned[0], id: j.id, id_reg: j.id }];
    renderScan({ ok: true, servos: state.scanned });
    logAssign(`成功：${r.old} → ${r.target}（${j.cn}）`
      + (r.protect && r.protect.ok ? '；过压/过流已开' : ''), 'ok');
    status('写入成功。断电 → 重新上电 → 再扫描一次确认。');
  } catch (e) {
    logAssign('失败：' + e.message, 'bad');
    showError('写入失败：' + e.message);
  } finally {
    setBusy(false);
  }
}

async function doVerify() {
  if (state.busy) return;
  setBusy(true, '总核对中');
  logAssign('');
  try {
    const r = await api('/api/verify', { method: 'POST', body: JSON.stringify({ port: state.port }) });
    if (r.pass) {
      logAssign(`总核对通过：${r.found.length} 颗，ID 齐全。`, 'ok');
      status('总核对通过。');
    } else {
      const miss = r.missing.length ? `缺 ${r.missing.join('、')}` : '不缺';
      const extra = r.extra.length ? `；多 ${r.extra.join('、')}` : '';
      logAssign(`总核对未通过：${miss}${extra}`, 'bad');
      status('总核对未通过。', true);
    }
  } catch (e) {
    showError('总核对失败：' + e.message);
  } finally {
    setBusy(false);
  }
}

// ---------------------------------------------------------------- 主循环

function tick(t) {
  const dt = Math.min(60, t - lastT);
  lastT = t;
  if (state.autoRotate) {
    cam.yaw += dt * 0.00022;
    dirty = true;
  }
  if (tween) {
    const u = clamp((t - tween.t0) / tween.dur, 0, 1);
    const e = 1 - Math.pow(1 - u, 3);
    cam.target = [
      lerp(tween.from.target[0], tween.to.target[0], e),
      lerp(tween.from.target[1], tween.to.target[1], e),
      lerp(tween.from.target[2], tween.to.target[2], e),
    ];
    cam.dist = lerp(tween.from.dist, tween.to.dist, e);
    if (u >= 1) tween = null;
    dirty = true;
  }
  if (dirty) {
    try {
      render();
      updateOverlay();
    } catch (e) {
      showError('3D 渲染失败：' + (e && e.message ? e.message : e));
      console.error(e);
      return;
    }
    dirty = false;
  }
  requestAnimationFrame(tick);
}

// ---------------------------------------------------------------- 启动

async function init() {
  bindUI();
  try {
    const cfg = await api('/api/config');
    state.plan = cfg.plan || [];
    const st = await api('/api/state');
    state.done = new Set(st.done || []);
    buildsGroups();
    buildPins();
    await loadPorts();
    const first = nextPending() || state.plan[0];
    state.selected = first ? first.id : null;
    updateAll();

    initGL();
    duck = await loadDuck();
    state.joints = duck.joints;
    window.__debug = {
      get gl() { return gl; },
      get duck() { return duck; },
      get cam() { return cam; },
      get vp() { return lastVP; },
      render, updateOverlay, computeVP,
    };
    loadingEl.classList.add('hidden');
    if (state.selected != null) {
      // 首次不拉近，先看全身
      updateOverlay();
    }
    dirty = true;
    requestAnimationFrame(tick);
    status('就绪。接一颗舵机到 URT-2，选关节后点「扫描当前舵机」。');
  } catch (e) {
    loadingEl.classList.add('hidden');
    showError('启动失败：' + e.message);
    console.error(e);
  }
}

init();
