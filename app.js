(function () {
  'use strict';
  const C = window.MapCore, $ = id => document.getElementById(id);
  const canvas = $('mapCanvas'), wrap = $('canvasWrap'), ctx = canvas.getContext('2d');
  const defaults = () => ({ resolution: .05, origin: [0, 0, 0], negate: 0, occupied_thresh: .65, free_thresh: .196, mode: 'trinary', image: '' });
  const state = { image: null, rgba: null, width: 0, height: 0, meta: defaults(), yamlName: '', imageName: '', source: 'manual', demo: false,
    poses: { pickup: null, dropoff: null }, selected: 'pickup', tool: 'select', view: { x: 0, y: 0, scale: 1 }, history: [], drag: null, space: false };
  let toastTimer, cw = 1, ch = 1, dragDepth = 0, labels = [];
  const clone = value => JSON.parse(JSON.stringify(value));
  const names = { pickup: '取货交接位', dropoff: '入库终点' }, colors = { pickup: '#2563eb', dropoff: '#e77b31' };
  function toast(message) { $('toast').textContent = message; $('toast').hidden = false; clearTimeout(toastTimer); toastTimer = setTimeout(() => $('toast').hidden = true, 5200); }
  function snapshot() { state.history.push(clone(state.poses)); if (state.history.length > 40) state.history.shift(); }
  function update() { inspector(); draw(); }
  function inspector() {
    $('mapName').textContent = state.yamlName || state.imageName || '尚未加载地图';
    $('mapHint').textContent = state.image ? (state.demo ? '内置示例，仅供体验；请加载现场地图后选点。' : state.source === 'manual' ? '使用手动地图参数，请核对分辨率和原点。' : '已读取地图参数，可离线选择车辆位姿。') : state.yamlName ? '请加载对应图片：' + state.meta.image : '一起选择 Nav2 地图 YAML 和对应图片，也可以拖入页面。';
    $('dimensions').textContent = state.image ? `${state.width} × ${state.height} px` : '—';
    $('resolutionLabel').textContent = state.image || state.yamlName ? state.meta.resolution + ' m/px' : '—';
    $('originLabel').textContent = state.image || state.yamlName ? state.meta.origin.map(v => Number(v.toFixed(4))).join(', ') : '—';
    $('mapBadge').hidden = !state.image; $('mapBadgeText').textContent = state.demo ? '示例地图 · 非现场数据' : 'map · ' + (state.imageName || '工程地图');
    $('emptyState').hidden = !!state.image;
    $('statusHint').textContent = !state.image ? '未加载地图' : state.demo ? '示例数据' : '箭头 = 车辆 +X';
    $('projectBtn').disabled = !state.image;
    $('originBtn').disabled = !state.image;
    if (!state.image) $('originStatus').textContent = 'map 原点为 (0, 0)，不同于 YAML 的图像原点。';
    $('zoomLabel').textContent = Math.round(state.view.scale * 100) + '%';
    $('undoBtn').disabled = !state.history.length;
    for (const key of ['pickup', 'dropoff']) {
      const p = state.poses[key], tab = $(key + 'Tab');
      tab.classList.toggle('selected', state.selected === key); tab.setAttribute('aria-pressed', String(state.selected === key));
      $(key + 'Summary').textContent = p ? `${p.x.toFixed(2)}, ${p.y.toFixed(2)} · ${(p.yaw * 180 / Math.PI).toFixed(1)}°` : key === 'pickup' ? '未设置 · 车辆对叉前的位置' : '未设置 · UP 状态 base_link';
      $(key + 'Check').textContent = p ? '✓' : '○'; $(key + 'Check').style.color = p ? colors[key] : '';
    }
    const p = state.poses[state.selected];
    $('selectedTitle').textContent = names[state.selected];
    $('referenceLabel').textContent = state.selected === 'pickup' ? '车辆对叉前的交接位姿' : '最终 UP 状态 base_link 位姿';
    for (const id of ['poseX', 'poseY', 'poseDeg']) $(id).disabled = !p;
    $('poseX').value = p ? p.x.toFixed(4) : ''; $('poseY').value = p ? p.y.toFixed(4) : ''; $('poseDeg').value = p ? (p.yaw * 180 / Math.PI).toFixed(2) : '';
    $('clearPoint').disabled = !p; $('copyPose').disabled = !p; $('copyTuple').disabled = !p; $('copyTupleDeg').disabled = !p;
    document.querySelectorAll('[data-angle]').forEach(b => b.disabled = !p);
    const q = p && C.quaternion(p.yaw);
    $('yawRad').textContent = p ? p.yaw.toFixed(9) : '—'; $('quatZ').textContent = q ? q.z.toFixed(8) : '—'; $('quatW').textContent = q ? q.w.toFixed(8) : '—';
    $('exportTask').disabled = !state.poses.pickup || !state.poses.dropoff || !state.image;
    $('exportGoal').disabled = !state.poses.dropoff || !state.image;
    $('exportHint').textContent = state.demo ? '当前为示例地图，导出坐标不能直接用于现场。' : state.poses.pickup && state.poses.dropoff ? '完整任务包含两点；入库目标仅包含终点。' : '设置两个点后可导出完整任务。';
    const goal = state.poses.dropoff, show = goal && $('showStaging').checked;
    $('candidateInfo').hidden = !show;
    if (show) {
      const [a, b] = C.stagingCandidates(goal, stagingDistance());
      $('candidateInfo').textContent = `交接候选（仅几何显示）\nx ${a.x.toFixed(3)} m · y ${a.y.toFixed(3)} m\n候选 A：${(a.yaw * 180 / Math.PI).toFixed(1)}°\nyaw = ${a.yaw.toFixed(9)} rad\n候选 B：${(b.yaw * 180 / Math.PI).toFixed(1)}°\nyaw = ${b.yaw.toFixed(9)} rad`;
      $('candidateInfo').style.whiteSpace = 'pre-line';
    }
    canvas.style.cursor = state.tool === 'pan' || state.space ? 'grab' : 'crosshair';
  }
  function syncMetaInputs() { $('metaResolution').value = state.meta.resolution; $('metaX').value = state.meta.origin[0]; $('metaY').value = state.meta.origin[1]; $('metaYaw').value = state.meta.origin[2]; }
  function stagingDistance() { const v = Number($('stagingDistance').value); return Number.isFinite(v) && v >= .1 && v <= 10 ? v : 1.7; }
  function local(event) { const r = canvas.getBoundingClientRect(); return { x: event.clientX - r.left, y: event.clientY - r.top }; }
  function pixel(screen) { return { u: (screen.x - state.view.x) / state.view.scale, v: (screen.y - state.view.y) / state.view.scale }; }
  function world(screen) { const p = pixel(screen); return C.pixelToWorld(p.u, p.v, state.height, state.meta); }
  function screen(p) { const v = C.worldToPixel(p.x, p.y, state.height, state.meta); return { x: state.view.x + v.u * state.view.scale, y: state.view.y + v.v * state.view.scale }; }
  function fit() { if (!state.image) return; const scale = Math.min((cw - 90) / state.width, (ch - 90) / state.height); state.view = { scale: Math.max(.005, scale), x: (cw - state.width * scale) / 2, y: (ch - state.height * scale) / 2 }; update(); }
  function zoom(factor, anchor = { x: cw / 2, y: ch / 2 }) { if (!state.image) return; const old = state.view.scale, next = Math.max(.003, Math.min(60, old * factor)); state.view.x = anchor.x - (anchor.x - state.view.x) * next / old; state.view.y = anchor.y - (anchor.y - state.view.y) * next / old; state.view.scale = next; update(); }
  function arrow(p, yaw, color, label, dashed = false, selected = false) {
    const a = screen(p), theta = yaw - state.meta.origin[2], dx = Math.cos(theta), dy = -Math.sin(theta), length = selected ? 65 : 49;
    if (a.x < -100 || a.y < -100 || a.x > cw + 100 || a.y > ch + 100) return;
    ctx.save(); ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = selected ? 3 : 2; ctx.setLineDash(dashed ? [5, 4] : []);
    if (selected) { ctx.beginPath(); ctx.arc(a.x, a.y, 15, 0, Math.PI * 2); ctx.fillStyle = color + '20'; ctx.fill(); ctx.fillStyle = color; }
    ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(a.x + dx * length, a.y + dy * length); ctx.stroke(); ctx.setLineDash([]);
    const tip = { x: a.x + dx * length, y: a.y + dy * length };
    ctx.beginPath(); ctx.moveTo(tip.x, tip.y); ctx.lineTo(tip.x - dx * 12 - dy * 5, tip.y - dy * 12 + dx * 5); ctx.lineTo(tip.x - dx * 12 + dy * 5, tip.y - dy * 12 - dx * 5); ctx.closePath(); ctx.fill();
    ctx.beginPath(); ctx.arc(a.x, a.y, 5, 0, 2 * Math.PI); ctx.fill(); ctx.strokeStyle = '#fff'; ctx.lineWidth = 2; ctx.stroke();
    if (label) labels.push({ x: a.x, y: a.y, label, color, priority: dashed ? 1 : 0 });
    ctx.restore();
  }
  function drawLabels() {
    const placed = [], markers = labels.map(p => ({ x: p.x - 9, y: p.y - 9, w: 18, h: 18 }));
    const overlaps = (a, b) => a.x < b.x + b.w + 3 && a.x + a.w + 3 > b.x && a.y < b.y + b.h + 3 && a.y + a.h + 3 > b.y;
    ctx.save(); ctx.font = '600 11px "Microsoft YaHei",sans-serif';
    for (const p of labels.sort((a, b) => a.priority - b.priority)) {
      const w = ctx.measureText(p.label).width + 14, h = 23;
      const offsets = [[12, 13], [12, -35], [-w - 12, -35], [-w - 12, 13], [12, 42], [-w - 12, 42], [12, -65]];
      let box;
      for (const [dx, dy] of offsets) {
        const a = { x: Math.max(4, Math.min(cw - w - 4, p.x + dx)), y: Math.max(4, Math.min(ch - h - 4, p.y + dy)), w, h };
        if (!placed.some(b => overlaps(a, b)) && !markers.some(b => overlaps(a, b))) { box = a; break; }
      }
      box = box || { x: Math.max(4, Math.min(cw - w - 4, p.x + 12)), y: Math.max(4, Math.min(ch - h - 4, p.y + 42)), w, h };
      ctx.fillStyle = '#fffffff0'; ctx.fillRect(box.x, box.y, w, h); ctx.fillStyle = p.color; ctx.fillText(p.label, box.x + 7, box.y + 15); placed.push(box);
    }
    ctx.restore();
  }
  function draw() {
    labels = [];
    ctx.clearRect(0, 0, cw, ch); ctx.fillStyle = '#edf1f5'; ctx.fillRect(0, 0, cw, ch);
    ctx.fillStyle = '#dce3ec'; for (let x = 12; x < cw; x += 24) for (let y = 12; y < ch; y += 24) ctx.fillRect(x, y, 1, 1);
    if (!state.image) return;
    const v = state.view; ctx.save(); ctx.shadowColor = '#273e5b22'; ctx.shadowBlur = 18; ctx.shadowOffsetY = 3; ctx.fillStyle = '#fff'; ctx.fillRect(v.x, v.y, state.width * v.scale, state.height * v.scale); ctx.restore();
    ctx.imageSmoothingEnabled = false; ctx.drawImage(state.image, v.x, v.y, state.width * v.scale, state.height * v.scale);
    ctx.strokeStyle = '#ccd5df'; ctx.lineWidth = 1; ctx.strokeRect(v.x, v.y, state.width * v.scale, state.height * v.scale);
    drawMapAxes();
    const goal = state.poses.dropoff;
    if (goal && $('showStaging').checked) {
      const [p, other] = C.stagingCandidates(goal, stagingDistance()), a = screen(goal), b = screen(p);
      ctx.save(); ctx.strokeStyle = '#c99c71'; ctx.lineWidth = 1.5; ctx.setLineDash([5, 5]); ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke(); ctx.restore();
      arrow(p, p.yaw, '#ae977e', '交接候选 ±90°', true);
      arrow(other, other.yaw, '#ae977e', '', true);
    }
    for (const key of ['pickup', 'dropoff']) if (state.poses[key]) arrow(state.poses[key], state.poses[key].yaw, colors[key], names[key], false, state.selected === key);
    drawLabels(); drawCompass(); drawRuler();
  }
  function drawMapAxes() {
    if (!$('showAxes').checked) { $('originStatus').textContent = 'map 原点为 (0, 0)；坐标轴显示已关闭。'; return; }
    const o = screen({ x: 0, y: 0 }), phi = state.meta.origin[2], v = state.view;
    const directions = [{ x: Math.cos(phi), y: Math.sin(phi), color: '#cb4d52', label: '+X', axis: 'x' },
      { x: Math.sin(phi), y: -Math.cos(phi), color: '#208b67', label: '+Y', axis: 'y' }];
    const uv = C.worldToPixel(0, 0, state.height, state.meta), inMap = uv.u >= 0 && uv.u <= state.width && uv.v >= 0 && uv.v <= state.height;
    const onScreen = o.x >= 0 && o.x <= cw && o.y >= 0 && o.y <= ch;
    $('originStatus').textContent = !inMap ? 'map 原点 (0, 0) 在图像范围外；可点击“定位原点”。' : !onScreen ? 'map 原点 (0, 0) 在当前视图外；可点击“定位原点”。' : 'map 原点 (0, 0) · 红色 +X / 绿色 +Y';
    ctx.save(); ctx.beginPath(); ctx.rect(v.x, v.y, state.width * v.scale, state.height * v.scale); ctx.clip();
    const reach = Math.hypot(cw, ch) + Math.hypot(o.x, o.y) + 100;
    const corners = [{ x: 0, y: 0 }, { x: cw, y: 0 }, { x: 0, y: ch }, { x: cw, y: ch }].map(world);
    const raw = 95 * state.meta.resolution / v.scale, power = Math.pow(10, Math.floor(Math.log10(raw))), n = raw / power;
    const step = (n <= 1 ? 1 : n <= 2 ? 2 : n <= 5 ? 5 : 10) * power;
    for (const d of directions) {
      ctx.strokeStyle = d.color + 'aa'; ctx.lineWidth = 1.1; ctx.setLineDash([7, 5]);
      ctx.beginPath(); ctx.moveTo(o.x - d.x * reach, o.y - d.y * reach); ctx.lineTo(o.x + d.x * reach, o.y + d.y * reach); ctx.stroke(); ctx.setLineDash([]);
      const min = Math.min(...corners.map(p => p[d.axis])), max = Math.max(...corners.map(p => p[d.axis]));
      ctx.font = '10px Consolas,monospace'; ctx.textAlign = 'center'; ctx.fillStyle = d.color;
      for (let t = Math.ceil(min / step) * step, i = 0; t <= max && i < 250; t += step, i++) {
        if (Math.abs(t) < step * 1e-6) continue;
        const p = screen(d.axis === 'x' ? { x: t, y: 0 } : { x: 0, y: t });
        ctx.beginPath(); ctx.moveTo(p.x - d.y * 4, p.y + d.x * 4); ctx.lineTo(p.x + d.y * 4, p.y - d.x * 4); ctx.stroke();
        const label = Number(t.toPrecision(6)) + ' m', x = p.x + d.y * 13, y = p.y - d.x * 13;
        const w = ctx.measureText(label).width + 4; ctx.fillStyle = '#ffffffd9'; ctx.fillRect(x - w / 2, y - 9, w, 12); ctx.fillStyle = d.color; ctx.fillText(label, x, y);
      }
    }
    ctx.restore();
    if (onScreen) {
      ctx.save(); ctx.font = '600 11px "Microsoft YaHei",sans-serif';
      for (const d of directions) {
        const tx = o.x + d.x * 65, ty = o.y + d.y * 65; ctx.strokeStyle = ctx.fillStyle = d.color; ctx.lineWidth = 2.5;
        ctx.beginPath(); ctx.moveTo(o.x, o.y); ctx.lineTo(tx, ty); ctx.stroke();
        ctx.beginPath(); ctx.moveTo(tx, ty); ctx.lineTo(tx - d.x * 10 + d.y * 4, ty - d.y * 10 - d.x * 4); ctx.lineTo(tx - d.x * 10 - d.y * 4, ty - d.y * 10 + d.x * 4); ctx.closePath(); ctx.fill();
        ctx.fillText(d.label, tx + d.x * 10 - 6, ty + d.y * 10 + 4);
      }
      ctx.fillStyle = '#fff'; ctx.strokeStyle = '#344b68'; ctx.lineWidth = 2; ctx.beginPath(); ctx.arc(o.x, o.y, 5, 0, 2 * Math.PI); ctx.fill(); ctx.stroke();
      ctx.fillStyle = '#ffffffed'; ctx.fillRect(o.x + 10, o.y + 9, 110, 24); ctx.fillStyle = '#344b68'; ctx.fillText('map 原点 (0, 0)', o.x + 16, o.y + 25); ctx.restore();
    }
    // YAML origin is the image's bottom-left boundary, not map (0, 0).
    const imageOrigin = { x: v.x, y: v.y + state.height * v.scale };
    if (imageOrigin.x >= 0 && imageOrigin.x < cw && imageOrigin.y >= 0 && imageOrigin.y < ch && Math.hypot(imageOrigin.x - o.x, imageOrigin.y - o.y) > 20) {
      ctx.save(); ctx.strokeStyle = '#8797aa'; ctx.fillStyle = '#fff'; ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.moveTo(imageOrigin.x, imageOrigin.y - 5); ctx.lineTo(imageOrigin.x + 5, imageOrigin.y); ctx.lineTo(imageOrigin.x, imageOrigin.y + 5); ctx.lineTo(imageOrigin.x - 5, imageOrigin.y); ctx.closePath(); ctx.fill(); ctx.stroke();
      const text = `图像原点 (${state.meta.origin[0]}, ${state.meta.origin[1]})`; ctx.font = '10px sans-serif';
      ctx.fillStyle = '#ffffffee'; ctx.fillRect(imageOrigin.x + 8, imageOrigin.y - 20, ctx.measureText(text).width + 10, 18); ctx.fillStyle = '#75869c'; ctx.fillText(text, imageOrigin.x + 13, imageOrigin.y - 7); ctx.restore();
    }
  }
  function drawCompass() {
    const x = 40, y = ch - 46, theta = -state.meta.origin[2];
    ctx.save(); ctx.fillStyle = '#ffffffdd'; ctx.fillRect(10, ch - 100, 100, 89); ctx.font = '10px sans-serif';
    for (const [a, color, label] of [[theta, '#db635e', 'X'], [theta + Math.PI / 2, '#47a18a', 'Y']]) {
      const dx = Math.cos(a), dy = -Math.sin(a); ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(x + dx * 30, y + dy * 30); ctx.stroke(); ctx.fillText(label, x + dx * 40 - 3, y + dy * 40 + 3);
    }
    ctx.fillStyle = '#8c9aae'; ctx.fillText('map 坐标轴', 18, ch - 18); ctx.restore();
  }
  function drawRuler() {
    const ppm = state.view.scale / state.meta.resolution, raw = 110 / ppm, power = Math.pow(10, Math.floor(Math.log10(raw))), n = raw / power;
    const meters = (n >= 5 ? 5 : n >= 2 ? 2 : 1) * power, length = meters * ppm, x = cw - 30 - length, y = ch - 28;
    ctx.save(); ctx.strokeStyle = '#597089'; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(x, y - 5); ctx.lineTo(x, y); ctx.lineTo(x + length, y); ctx.lineTo(x + length, y - 5); ctx.stroke(); ctx.fillStyle = '#597089'; ctx.textAlign = 'center'; ctx.font = '10px sans-serif'; ctx.fillText(Number(meters.toPrecision(3)) + ' m', x + length / 2, y - 11); ctx.restore();
  }
  function resize() { const r = wrap.getBoundingClientRect(); cw = r.width; ch = r.height; const dpr = window.devicePixelRatio || 1; canvas.width = Math.round(cw * dpr); canvas.height = Math.round(ch * dpr); ctx.setTransform(dpr, 0, 0, dpr, 0, 0); draw(); }
  function showCursor(pos) {
    if (!state.image) return;
    const p = world(pos), px = pixel(pos), u = Math.floor(px.u), v = Math.floor(px.v);
    $('cursorPosition').textContent = `map  x ${p.x.toFixed(3)} m    y ${p.y.toFixed(3)} m`;
    if (u < 0 || v < 0 || u >= state.width || v >= state.height) { $('cellStatus').textContent = '地图外'; return; }
    const at = (v * state.width + u) * 4, rgba = state.rgba;
    let occupancy = (rgba[at] + rgba[at + 1] + rgba[at + 2]) / (3 * 255); if (!state.meta.negate) occupancy = 1 - occupancy;
    const label = state.meta.mode !== 'trinary' ? '灰度预览' : rgba[at + 3] < 255 ? '未知' : occupancy > state.meta.occupied_thresh ? '占用' : occupancy < state.meta.free_thresh ? '自由' : '未知';
    $('cellStatus').textContent = `${label} · 像素 ${u}, ${v}`;
  }
  canvas.addEventListener('contextmenu', e => e.preventDefault());
  canvas.addEventListener('pointerdown', e => {
    if (!state.image) return; const a = local(e); wrap.focus(); canvas.setPointerCapture(e.pointerId);
    if (e.button === 1 || e.button === 2 || state.space || state.tool === 'pan') { state.drag = { pan: true, start: a, view: { ...state.view } }; return; }
    if (e.button !== 0) return; const px = pixel(a); if (px.u < 0 || px.v < 0 || px.u > state.width || px.v > state.height) return;
    snapshot(); const p = world(a), old = state.poses[state.selected]; state.poses[state.selected] = { x: p.x, y: p.y, yaw: old ? old.yaw : 0 }; state.drag = { pan: false, start: a, point: p }; update();
  });
  canvas.addEventListener('pointermove', e => {
    const p = local(e); showCursor(p); const d = state.drag; if (!d) return;
    if (d.pan) { state.view.x = d.view.x + p.x - d.start.x; state.view.y = d.view.y + p.y - d.start.y; draw(); }
    else if (Math.hypot(p.x - d.start.x, p.y - d.start.y) > 6) { const target = world(p); state.poses[state.selected].yaw = Math.atan2(target.y - d.point.y, target.x - d.point.x); update(); }
  });
  function endDrag() { state.drag = null; }
  canvas.addEventListener('pointerup', endDrag); canvas.addEventListener('pointercancel', endDrag); canvas.addEventListener('lostpointercapture', endDrag);
  canvas.addEventListener('wheel', e => { e.preventDefault(); zoom(Math.exp(-e.deltaY * .0015), local(e)); }, { passive: false });
  document.addEventListener('keydown', e => { if (['INPUT', 'TEXTAREA'].includes(document.activeElement.tagName)) return; if (e.code === 'Space') { e.preventDefault(); state.space = true; inspector(); } if ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === 'z') { e.preventDefault(); undo(); } });
  document.addEventListener('keyup', e => { if (e.code === 'Space') { state.space = false; inspector(); } });
  window.addEventListener('blur', () => { state.space = false; endDrag(); });
  function undo() { if (!state.history.length) return; state.poses = state.history.pop(); update(); }
  $('undoBtn').onclick = undo; $('fitBtn').onclick = fit; $('zoomIn').onclick = () => zoom(1.3); $('zoomOut').onclick = () => zoom(1 / 1.3);
  $('showAxes').onchange = draw;
  $('originBtn').onclick = () => { if (!state.image) return; $('showAxes').checked = true; const p = C.worldToPixel(0, 0, state.height, state.meta); state.view.x = cw / 2 - p.u * state.view.scale; state.view.y = ch / 2 - p.v * state.view.scale; update(); };
  for (const key of ['pickup', 'dropoff']) $(key + 'Tab').onclick = () => { state.selected = key; update(); };
  for (const tool of ['select', 'pan']) $(tool + 'Tool').onclick = () => { state.tool = tool; $('selectTool').classList.toggle('active', tool === 'select'); $('panTool').classList.toggle('active', tool === 'pan'); inspector(); };
  for (const id of ['poseX', 'poseY', 'poseDeg']) $(id).onchange = () => {
    const p = state.poses[state.selected]; if (!p) return;
    const value = Number($(id).value); if (!$(id).value.trim() || !Number.isFinite(value)) { toast('请输入有限数值。'); inspector(); return; }
    snapshot(); if (id === 'poseDeg') p.yaw = C.wrap(value * Math.PI / 180); else p[id === 'poseX' ? 'x' : 'y'] = value; update();
  };
  document.querySelectorAll('[data-angle]').forEach(b => b.onclick = () => { const p = state.poses[state.selected]; if (!p) return; snapshot(); p.yaw = C.wrap(Number(b.dataset.angle) * Math.PI / 180); update(); });
  $('clearPoint').onclick = () => { snapshot(); state.poses[state.selected] = null; update(); };
  $('showStaging').onchange = update; $('stagingDistance').onchange = () => { if (!Number.isFinite(Number($('stagingDistance').value)) || Number($('stagingDistance').value) < .1 || Number($('stagingDistance').value) > 10) { $('stagingDistance').value = '1.7'; toast('交接距离范围为 0.1～10 米。'); } update(); };
  function setMeta(meta) {
    if (state.image) for (const key of ['pickup', 'dropoff']) { const p = state.poses[key]; if (!p) continue; const old = C.worldToPixel(p.x, p.y, state.height, state.meta), next = C.pixelToWorld(old.u, old.v, state.height, meta); state.poses[key] = { ...next, yaw: C.wrap(p.yaw + meta.origin[2] - state.meta.origin[2]) }; }
    state.meta = meta; state.history = []; syncMetaInputs(); update();
  }
  $('applyMeta').onclick = () => { try {
    const values = ['metaResolution', 'metaX', 'metaY', 'metaYaw'].map(id => { if (!$(id).value.trim()) throw Error('地图参数不能为空。'); return Number($(id).value); });
    setMeta(C.validateMeta({ ...state.meta, resolution: values[0], origin: values.slice(1) })); state.source = 'manual'; state.demo = false; update(); toast('地图参数已更新，已有选点按图像位置重新换算。');
  } catch (e) { toast(e.message); } };
  async function decodeImage(file) {
    const off = document.createElement('canvas');
    if (/\.pgm$/i.test(file.name)) {
      const p = C.parsePgm(await file.arrayBuffer()); off.width = p.width; off.height = p.height; const context = off.getContext('2d'), image = context.createImageData(p.width, p.height);
      for (let i = 0; i < p.gray.length; i++) { image.data[4 * i] = image.data[4 * i + 1] = image.data[4 * i + 2] = p.gray[i]; image.data[4 * i + 3] = 255; } context.putImageData(image, 0, 0);
    } else {
      const url = URL.createObjectURL(file);
      try { const img = await imageFromUrl(url); if (img.width * img.height > 64000000) throw Error('地图图片超过 6400 万像素。'); off.width = img.width; off.height = img.height; off.getContext('2d').drawImage(img, 0, 0); }
      finally { URL.revokeObjectURL(url); }
    }
    return off;
  }
  function imageFromUrl(url) { return new Promise((resolve, reject) => { const img = new Image(); img.onload = () => resolve(img); img.onerror = () => reject(Error('图片无法读取，请选择 PGM、PNG 或 JPG。')); img.src = url; }); }
  function install(image, meta, options = {}) {
    state.image = image; state.width = image.width; state.height = image.height; state.rgba = image.getContext('2d').getImageData(0, 0, image.width, image.height).data;
    state.meta = meta; state.imageName = options.imageName || ''; state.yamlName = options.yamlName || ''; state.source = options.source || 'yaml'; state.demo = !!options.demo;
    state.poses = options.poses || { pickup: null, dropoff: null }; state.history = []; state.drag = null; syncMetaInputs(); fit(); update();
  }
  const basename = name => name.replace(/\\/g, '/').split('/').pop().toLowerCase();
  async function loadFiles(files) {
    try {
      files = [...files]; if (!files.length) return;
      if (files.some(f => f.size > 150000000)) throw Error('单个文件不能超过 150 MB。');
      const projects = files.filter(f => /\.json$/i.test(f.name));
      if (projects.length) {
        if (files.length !== 1) throw Error('恢复工程时，请只选择一个工程 JSON 文件。');
        const data = JSON.parse(await projects[0].text());
        if (data.format !== 'forklift-map-points-v1' || !/^data:image\/png;base64,/.test(data.imageData || '')) throw Error('不是本工具保存的地图工程。');
        const meta = C.validateMeta(data.meta);
        for (const key of ['pickup', 'dropoff']) if (data.poses[key] !== null && !C.validPose(data.poses[key])) throw Error('工程中的点位无效。');
        const img = await imageFromUrl(data.imageData); if (img.width * img.height > 64000000) throw Error('工程地图过大。'); const off = document.createElement('canvas'); off.width = img.width; off.height = img.height; off.getContext('2d').drawImage(img, 0, 0);
        install(off, meta, { ...data, poses: data.poses }); $('stagingDistance').value = Number.isFinite(data.stagingDistance) && data.stagingDistance >= .1 && data.stagingDistance <= 10 ? data.stagingDistance : 1.7; update(); toast('地图和选点已恢复。'); return;
      }
      const yamls = files.filter(f => /\.ya?ml$/i.test(f.name)); if (yamls.length > 1) throw Error('请一次只加载一张地图的 YAML。');
      const yaml = yamls[0], meta = yaml ? C.parseYaml(await yaml.text()) : state.meta;
      const images = files.filter(f => /\.(pgm|png|jpe?g|bmp|webp)$/i.test(f.name));
      let file = meta.image ? images.find(f => basename(f.name) === basename(meta.image)) : null;
      if (!file && images.length === 1) file = images[0];
      if (!file && images.length > 1) throw Error('多张图片中未找到 YAML 对应图片，请选择准确的文件。');
      if (file) {
        const useYaml = !!yaml || !!state.yamlName && basename(meta.image) === basename(file.name);
        const image = await decodeImage(file); install(image, useYaml ? meta : defaults(), { imageName: file.name, yamlName: yaml ? yaml.name : useYaml ? state.yamlName : '', source: useYaml ? 'yaml' : 'manual' });
        if (yaml && meta.image && basename(meta.image) !== basename(file.name)) toast('所选图片名称与 YAML 的 image 不一致，请核对是否为同一张地图。');
        else toast(useYaml ? '地图已加载。按住左键拖动，设置位置和朝向。' : '图片已加载。请在左侧核对并填写地图分辨率和原点。');
      } else if (yaml) {
        if (state.image && basename(state.imageName) === basename(meta.image)) { setMeta(meta); state.yamlName = yaml.name; state.source = 'yaml'; state.demo = false; update(); }
        else { state.image = null; state.width = state.height = 0; state.rgba = null; state.history = []; state.poses = { pickup: null, dropoff: null }; state.meta = meta; state.yamlName = yaml.name; state.imageName = ''; state.source = 'yaml'; syncMetaInputs(); update(); }
        toast(state.image ? 'YAML 参数已更新，选点已按图像位置重新换算。' : 'YAML 已读取，请继续加载对应地图图片。');
      } else throw Error('请加载地图 YAML、PGM/PNG 图片或本工具的工程 JSON。');
    } catch (e) { console.error(e); toast('加载失败：' + e.message); }
  }
  $('openBtn').onclick = $('emptyOpen').onclick = () => $('fileInput').click();
  $('fileInput').onchange = async e => { await loadFiles(e.target.files); e.target.value = ''; };
  window.addEventListener('dragenter', e => { if (!e.dataTransfer.types.includes('Files')) return; e.preventDefault(); dragDepth++; $('dragOverlay').hidden = false; });
  window.addEventListener('dragover', e => { if (e.dataTransfer.types.includes('Files')) e.preventDefault(); });
  window.addEventListener('dragleave', e => { e.preventDefault(); if (--dragDepth <= 0) { dragDepth = 0; $('dragOverlay').hidden = true; } });
  window.addEventListener('drop', e => { e.preventDefault(); dragDepth = 0; $('dragOverlay').hidden = true; loadFiles(e.dataTransfer.files); });
  function download(text, name, type) { const a = document.createElement('a'), url = URL.createObjectURL(new Blob([text], { type })); a.href = url; a.download = name; a.click(); setTimeout(() => URL.revokeObjectURL(url), 10000); toast('已导出 ' + name + (state.demo ? '（示例坐标，不能用于现场）' : '')); }
  $('exportTask').onclick = () => download((state.demo ? '# 示例地图坐标，不可用于实车\n' : '') + C.taskYaml(state.poses), 'task.yaml', 'text/yaml;charset=utf-8');
  $('exportGoal').onclick = () => download((state.demo ? '# 示例地图坐标，不可用于实车\n' : '') + C.goalYaml(state.poses.dropoff, stagingDistance()), 'goal_task.yaml', 'text/yaml;charset=utf-8');
  $('projectBtn').onclick = () => download(JSON.stringify({ format: 'forklift-map-points-v1', frame: 'map', meta: state.meta, imageData: state.image.toDataURL('image/png'), imageName: state.imageName, yamlName: state.yamlName, source: state.source, demo: state.demo, poses: state.poses, stagingDistance: stagingDistance() }, null, 2), 'map_points.json', 'application/json');
  async function copyText(text, name, message) {
    try { await navigator.clipboard.writeText(text); toast(message); }
    catch (_) { const t = document.createElement('textarea'); t.value = text; document.body.appendChild(t); t.select(); const ok = document.execCommand('copy'); t.remove(); if (ok) toast(message); else download(text, name, 'text/plain'); }
  }
  $('copyTuple').onclick = () => copyText(C.poseTuple(state.poses[state.selected]), 'pose_tuple_rad.txt', '已复制 (x, y, yaw)，yaw 单位为弧度。');
  $('copyTupleDeg').onclick = () => copyText(C.poseTuple(state.poses[state.selected], 'deg'), 'pose_tuple_deg.txt', '已复制 (x, y, yaw)，yaw 单位为度。');
  $('copyPose').onclick = async () => {
    const p = state.poses[state.selected], q = C.quaternion(p.yaw), text = `{header: {frame_id: map}, pose: {position: {x: ${p.x.toFixed(6)}, y: ${p.y.toFixed(6)}, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: ${q.z.toFixed(9)}, w: ${q.w.toFixed(9)}}}}`;
    await copyText(text, 'pose.txt', '已复制 PoseStamped。');
  };
  $('demoBtn').onclick = () => {
    const off = document.createElement('canvas'); off.width = 760; off.height = 650; const g = off.getContext('2d');
    g.fillStyle = '#cdcdcd'; g.fillRect(0, 0, 760, 650); g.fillStyle = '#fbfbf8'; g.fillRect(45, 45, 670, 560); g.strokeStyle = '#262d35'; g.lineWidth = 9; g.strokeRect(45, 45, 670, 560);
    g.fillStyle = '#48515c'; for (const y of [115, 295, 475]) for (const x of [100, 500]) g.fillRect(x, y, 160, 60);
    g.fillStyle = '#fbfbf8'; g.fillRect(326, 42, 107, 10); g.fillRect(326, 600, 107, 10);
    install(off, { ...defaults(), resolution: .02, origin: [-7.6, -6.5, 0] }, { demo: true, yamlName: '示例仓库 · 非现场地图', imageName: 'demo.png' });
  };
  new ResizeObserver(resize).observe(wrap); resize(); syncMetaInputs(); inspector();
  window.MapEditor = { loadFiles, getState: () => ({ meta: clone(state.meta), poses: clone(state.poses), width: state.width, height: state.height, view: { ...state.view }, selected: state.selected, source: state.source, demo: state.demo }), worldToScreen: p => screen(p) };
})();
