'use strict';
const assert = require('node:assert/strict');
const fs = require('node:fs');
const test = require('node:test');
const path = require('node:path');
const html = fs.readFileSync(path.join(__dirname, '../windows/dashboard/index.html'), 'utf8');
const parserCode = html.match(/\/\/ BEGIN HW11_TELEMETRY_PARSER([\s\S]*?)\/\/ END HW11_TELEMETRY_PARSER/)[1];
const {parse} = new Function(parserCode + '; return Hw11Telemetry;')();
const header = '# hw11-telemetry-v1\nt_ms,x,y,z,vx,vy,speed,dir,state\n';
const row1 = '1000,1.25,-2.5,10,3,4,5,0.5,1';
const row2 = '1100,1.55,-2.5,10,3,4,5,0.5,1';
const valid = header + row1 + '\n' + row2 + '\n';

test('reads telemetry fields and converts milliseconds to seconds', () => {
  assert.deepEqual(parse(valid)[0],
    {t:1, x:1.25, y:-2.5, z:10, vx:3, vy:4, speed:5, dir:0.5, state:1});
  assert.equal(parse(valid)[1].t, 1.1);
  assert.equal(parse('\uFEFF' + valid.replace(/\n/g, '\r\n')).length, 2);
});
test('rejects offline JSON and unsupported schemas', () => {
  assert.throws(() => parse('{"steps":[{"position":{"x":1,"y":2}}]}'), /CSV/);
  assert.throws(() => parse(valid.replace('v1', 'v2')), /CSV/);
});
test('rejects missing, non-finite, truncated and non-monotonic samples', () => {
  for (const bad of [row2.replace('1.55', ''), row2.replace('1.55', 'NaN'),
    row2.replace('1.55', '1e999'), row2.replace(',1', ''),
    row2.replace('1100', '1000'), row2.replace('1100', '900'),
    row2.replace('1100', '1100.5')]) {
    assert.throws(() => parse(header + row1 + '\n' + bad));
  }
  assert.throws(() => parse(header + row1), /2 до 5000/);
  assert.throws(() => parse(header + Array(5001).fill(row1).join('\n')), /2 до 5000/);
});
test('reads actual output produced by the C++ logger when supplied', {
  skip: !process.env.HW11_LOG_FIXTURE
}, () => {
  const points = parse(fs.readFileSync(process.env.HW11_LOG_FIXTURE, 'utf8'));
  assert.equal(points.length, 2);
  assert.equal(points[0].x, 1.25);
  assert.equal(points[1].t, 1.1);
});
