/* Pure Nav2 map geometry and file readers. No ROS or network dependency. */
(function (root) {
  'use strict';
  const finite = v => typeof v === 'number' && Number.isFinite(v);
  const wrap = a => Math.atan2(Math.sin(a), Math.cos(a));
  function uncomment(line) {
    let quote = null;
    for (let i = 0; i < line.length; i++) {
      const c = line[i];
      if (quote) { if (c === quote && line[i - 1] !== '\\') quote = null; }
      else if (c === '"' || c === "'") quote = c;
      else if (c === '#') return line.slice(0, i);
    }
    return line;
  }
  function unquote(value) {
    value = value.trim();
    if (value.startsWith('"') && value.endsWith('"')) {
      try { return JSON.parse(value); } catch (_) { return value.slice(1, -1); }
    }
    if (value.startsWith("'") && value.endsWith("'")) return value.slice(1, -1).replace(/''/g, "'");
    return value;
  }
  function validateMeta(meta) {
    if (!finite(meta.resolution) || meta.resolution <= 0 || meta.resolution > 100) throw Error('resolution 必须为正数，单位为米/像素。');
    if (!Array.isArray(meta.origin) || meta.origin.length !== 3 || !meta.origin.every(finite)) throw Error('origin 必须包含三个有限数值：[x, y, yaw]，yaw 单位为弧度。');
    if (![0, 1].includes(meta.negate)) throw Error('negate 必须为 0 或 1。');
    if (!finite(meta.free_thresh) || !finite(meta.occupied_thresh) || meta.free_thresh < 0 || meta.free_thresh >= meta.occupied_thresh || meta.occupied_thresh > 1) throw Error('地图阈值需满足 0 ≤ free_thresh < occupied_thresh ≤ 1。');
    return meta;
  }
  function parseYaml(text) {
    const values = {}, lines = text.replace(/^\uFEFF/, '').split(/\r?\n/).map(uncomment);
    for (let i = 0; i < lines.length; i++) {
      const m = lines[i].match(/^\s*(image|resolution|origin|negate|occupied_thresh|free_thresh|mode)\s*:\s*(.*?)\s*$/);
      if (!m) continue;
      const key = m[1], value = m[2];
      if (key === 'origin') {
        if (value.startsWith('[') && value.endsWith(']')) values.origin = value.slice(1, -1).split(',').map(v => Number(v.trim()));
        else if (!value) {
          const parts = [];
          while (i + 1 < lines.length && /^\s*-\s+/.test(lines[i + 1])) parts.push(Number(lines[++i].replace(/^\s*-\s+/, '').trim()));
          values.origin = parts;
        } else throw Error('origin 请使用 [x, y, yaw] 或 YAML 列表。');
      } else if (key === 'image' || key === 'mode') values[key] = unquote(value);
      else if (key === 'negate' && /^(true|false)$/i.test(value)) values[key] = value.toLowerCase() === 'true' ? 1 : 0;
      else values[key] = Number(value);
    }
    return validateMeta(Object.assign({ image: '', negate: 0, occupied_thresh: 0.65, free_thresh: 0.196, mode: 'trinary' }, values));
  }
  function parsePgm(buffer) {
    const data = buffer instanceof Uint8Array ? buffer : new Uint8Array(buffer);
    let at = 0;
    const whitespace = c => c === 9 || c === 10 || c === 11 || c === 12 || c === 13 || c === 32;
    function token() {
      while (at < data.length) {
        if (whitespace(data[at])) at++;
        else if (data[at] === 35) { while (at < data.length && data[at] !== 10 && data[at] !== 13) at++; }
        else break;
      }
      const start = at;
      while (at < data.length && !whitespace(data[at]) && data[at] !== 35) at++;
      if (at === start) throw Error('PGM 文件头或像素数据不完整。');
      return String.fromCharCode(...data.subarray(start, at));
    }
    const magic = token();
    if (magic !== 'P2' && magic !== 'P5') throw Error('仅支持 PGM P2 / P5 格式。');
    const width = Number(token()), height = Number(token()), max = Number(token());
    if (!Number.isInteger(width) || !Number.isInteger(height) || width <= 0 || height <= 0 || width * height > 64000000 || !Number.isInteger(max) || max < 1 || max > 65535) throw Error('PGM 尺寸或最大灰度无效，最多支持 6400 万像素。');
    const gray = new Uint8Array(width * height);
    if (magic === 'P2') {
      for (let i = 0; i < gray.length; i++) {
        const v = Number(token());
        if (!Number.isInteger(v) || v < 0 || v > max) throw Error('PGM 灰度值超出范围。');
        gray[i] = Math.round(v / max * 255);
      }
    } else {
      // Exactly one raster separator (or CRLF). Never trim binary pixel bytes:
      // the first pixel can itself be space, newline or '#'.
      if (!whitespace(data[at])) throw Error('PGM 二进制数据缺少分隔符。');
      if (data[at++] === 13 && data[at] === 10) at++;
      const step = max < 256 ? 1 : 2;
      if (data.length - at < gray.length * step) throw Error('PGM 像素数据不完整。');
      for (let i = 0; i < gray.length; i++) {
        const v = step === 1 ? data[at++] : data[at++] * 256 + data[at++];
        if (v > max) throw Error('PGM 灰度值超出范围。');
        gray[i] = Math.round(v / max * 255);
      }
    }
    return { width, height, gray };
  }
  function pixelToWorld(u, v, height, meta) {
    const x = u * meta.resolution, y = (height - v) * meta.resolution;
    const c = Math.cos(meta.origin[2]), s = Math.sin(meta.origin[2]);
    return { x: meta.origin[0] + c * x - s * y, y: meta.origin[1] + s * x + c * y };
  }
  function worldToPixel(x, y, height, meta) {
    const dx = x - meta.origin[0], dy = y - meta.origin[1], c = Math.cos(meta.origin[2]), s = Math.sin(meta.origin[2]);
    return { u: (c * dx + s * dy) / meta.resolution, v: height - (-s * dx + c * dy) / meta.resolution };
  }
  function quaternion(yaw) { return { x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) }; }
  function validPose(p) { return p && [p.x, p.y, p.yaw].every(finite); }
  function poseTuple(p, unit = 'rad') {
    if (!validPose(p)) throw Error('请先设置有效位姿。');
    if (unit !== 'rad' && unit !== 'deg') throw Error('角度单位必须为 rad 或 deg。');
    const yaw = wrap(p.yaw) * (unit === 'deg' ? 180 / Math.PI : 1);
    return `(${p.x.toFixed(6)}, ${p.y.toFixed(6)}, ${yaw.toFixed(unit === 'deg' ? 6 : 9)})`;
  }
  function stagingCandidates(p, distance) {
    if (!validPose(p) || !finite(distance) || distance <= 0) throw Error('无效的交接候选参数。');
    const x = p.x + distance * Math.cos(p.yaw), y = p.y + distance * Math.sin(p.yaw);
    return [-1, 1].map(sign => ({ x, y, yaw: wrap(p.yaw + sign * Math.PI / 2) }));
  }
  function goalYaml(p, distance) {
    if (!validPose(p)) throw Error('请先设置入库终点。');
    return '# 最终 UP 状态 base_link 位姿；yaw 单位：弧度\nframe: map\ngoal: {x: ' + p.x.toFixed(6) + ', y: ' + p.y.toFixed(6) + ', yaw: ' + wrap(p.yaw).toFixed(9) + '}\nstaging_distance: ' + Number(distance).toFixed(3) + '\n';
  }
  function taskYaml(poses) {
    if (!validPose(poses.pickup) || !validPose(poses.dropoff)) throw Error('请先设置取货交接位和入库终点。');
    const row = p => '{x: ' + p.x.toFixed(6) + ', y: ' + p.y.toFixed(6) + ', yaw: ' + wrap(p.yaw).toFixed(9) + '}';
    return '# 车辆位姿；配合 task_chain.yaml 的 vehicle 参考模式使用\n# pickup：车辆取货交接位；dropoff：入库最终 UP 状态 base_link 位姿\nframe: map\npickup: ' + row(poses.pickup) + '\ndropoff: ' + row(poses.dropoff) + '\ntimeout: 1800.0\n';
  }
  const api = { wrap, parseYaml, validateMeta, parsePgm, pixelToWorld, worldToPixel, quaternion, validPose, poseTuple, stagingCandidates, goalYaml, taskYaml };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else root.MapCore = api;
})(typeof globalThis !== 'undefined' ? globalThis : this);
