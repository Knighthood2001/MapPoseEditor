const assert = require('assert');
const C = require('./map-core.js');
let count = 0;
function test(name, fn) { fn(); count++; console.log('PASS ' + name); }
function near(a, b) { assert.ok(Math.abs(a - b) < 1e-9, `${a} != ${b}`); }
const base = { resolution: .05, origin: [-15.75, -14.648, 0], negate: 0, free_thresh: .196, occupied_thresh: .65 };
test('Nav2 block origin and defaults', () => {
  const m = C.parseYaml('image: map.pgm\nresolution: 0.05\norigin:\n  - -15.75\n  - -14.648\n  - 0\n');
  assert.deepStrictEqual(m.origin, base.origin); assert.equal(m.mode, 'trinary');
});
test('flow origin, scientific notation, quoted filename with hash', () => {
  const m = C.parseYaml('image: "test#map.pgm" # comment\nresolution: 5e-2\norigin: [1, -2, 1.5707963267948966]');
  assert.equal(m.image, 'test#map.pgm'); near(m.resolution, .05); near(m.origin[2], Math.PI / 2);
});
test('image bottom left is map origin', () => { const p = C.pixelToWorld(0, 500, 500, base); near(p.x, -15.75); near(p.y, -14.648); });
test('pixel centers use half-cell geometry and inverted rows', () => {
  const p = C.pixelToWorld(.5, .5, 500, base); near(p.x, -15.725); near(p.y, 10.327);
});
test('origin yaw rotates the image into map', () => {
  const meta = { ...base, resolution: 1, origin: [10, 20, Math.PI / 2] };
  const p = C.pixelToWorld(2, 7, 10, meta); near(p.x, 7); near(p.y, 22);
});
test('coordinate round trips at arbitrary origin yaw', () => {
  for (const yaw of [0, .4, -2, Math.PI / 2]) for (let i = 0; i < 100; i++) {
    const m = { ...base, origin: [-17, 12, yaw] }, p = C.pixelToWorld(i * 3.1, i * .7, 600, m), uv = C.worldToPixel(p.x, p.y, 600, m);
    near(uv.u, i * 3.1); near(uv.v, i * .7);
  }
});
test('P2 gray scale and comments', () => {
  const p = C.parsePgm(Buffer.from('P2\n# sample\n2 2\n15\n0 15 # row\n7 8\n'));
  assert.equal(p.width, 2); assert.deepStrictEqual([...p.gray], [0, 255, 119, 136]);
});
test('P5 preserves first raster bytes that look like whitespace or comment', () => {
  const p = C.parsePgm(Buffer.concat([Buffer.from('P5\n3 1\n255\n'), Buffer.from([10, 32, 35])]));
  assert.deepStrictEqual([...p.gray], [10, 32, 35]);
});
test('P5 CRLF header', () => {
  const p = C.parsePgm(Buffer.concat([Buffer.from('P5\r\n2 1\r\n255\r\n'), Buffer.from([0, 255])]));
  assert.deepStrictEqual([...p.gray], [0, 255]);
});
test('P5 16 bit big-endian pixels', () => {
  const p = C.parsePgm(Buffer.concat([Buffer.from('P5\n3 1\n65535\n'), Buffer.from([0, 0, 128, 0, 255, 255])]));
  assert.deepStrictEqual([...p.gray], [0, 128, 255]);
});
test('truncated and invalid PGM rejected', () => {
  assert.throws(() => C.parsePgm(Buffer.from('P5\n2 2\n255\n\0')));
  assert.throws(() => C.parsePgm(Buffer.from('P2\n-1 2\n255\n0 0')));
});
test('invalid geometry rejected', () => {
  assert.throws(() => C.parseYaml('resolution: 0\norigin: [0,0,0]'));
  assert.throws(() => C.parseYaml('resolution: .05\norigin: [0,0]'));
  assert.throws(() => C.parseYaml('resolution: .05\norigin: [0,0,NaN]'));
});
test('90 degree quaternion', () => { const q = C.quaternion(Math.PI / 2); near(q.z, Math.SQRT1_2); near(q.w, Math.SQRT1_2); });
test('task export keeps map vehicle coordinates and yaw radians', () => {
  const text = C.taskYaml({ pickup: { x: 1, y: 2, yaw: Math.PI }, dropoff: { x: 3.7, y: -.5, yaw: Math.PI / 2 } });
  assert.ok(text.includes('yaw: 1.570796327')); assert.ok(text.includes('frame: map')); assert.ok(text.includes('timeout: 1800.0'));
});
test('missing point cannot export a complete task', () => assert.throws(() => C.taskYaml({ pickup: null, dropoff: null })));
test('goal export has staging distance and normalized angle', () => {
  const text = C.goalYaml({ x: 1, y: 2, yaw: 2 * Math.PI }, 1.7); assert.ok(text.includes('staging_distance: 1.700'));
});
test('pose tuple radians and degrees are explicit conversions', () => {
  const p = { x: -10.183, y: 1.94, yaw: Math.PI / 2 };
  assert.equal(C.poseTuple(p), '(-10.183000, 1.940000, 1.570796327)');
  assert.equal(C.poseTuple(p, 'deg'), '(-10.183000, 1.940000, 90.000000)');
  assert.throws(() => C.poseTuple(p, 'unknown'));
});
test('staging candidates provide correct absolute map yaw radians', () => {
  const [a, b] = C.stagingCandidates({ x: -10.183, y: .24, yaw: Math.PI / 2 }, 1.7);
  near(a.x, -10.183); near(a.y, 1.94); near(a.yaw, 0); near(b.yaw, Math.PI);
  assert.deepStrictEqual([a.x, a.y], [b.x, b.y]);
});
test('map zero differs from translated/rotated YAML origin', () => {
  for (const phi of [0, Math.PI / 2, -.7]) {
    const m = { ...base, origin: [-3, 7, phi] }, uv = C.worldToPixel(0, 0, 500, m);
    const p = C.pixelToWorld(uv.u, uv.v, 500, m); near(p.x, 0); near(p.y, 0);
    const imageOrigin = C.pixelToWorld(0, 500, 500, m); near(imageOrigin.x, -3); near(imageOrigin.y, 7);
  }
});
console.log(`${count} core tests passed.`);
