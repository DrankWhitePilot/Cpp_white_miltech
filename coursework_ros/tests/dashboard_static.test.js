'use strict';

const fs = require('node:fs');
const path = require('node:path');
const test = require('node:test');
const assert = require('node:assert/strict');

const root = path.resolve(__dirname, '..');
const windows = path.join(root, 'windows');
const html = fs.readFileSync(path.join(windows, 'dashboard', 'index.html'), 'utf8');

test('dashboard exposes the ten scenario choices and five parameter sets', () => {
  const scenarios = [
    'base_circles', 'heavy_ammo', 'lissajous_eight', 'extreme', 'ellipses',
    'cardioids_epitrochoids', 'flowers', 'lissajous_complex',
    'gliding_ammo', 'fast_drone_slow_targets'
  ];
  const ammo = ['VOG-17', 'M67', 'RKG-3', 'GLIDING-VOG', 'GLIDING-RKG'];
  for (const id of scenarios) assert.match(html, new RegExp(`value="${id}"`));
  for (const id of ammo) assert.match(html, new RegExp(`value="${id}"`));
});

test('dashboard contains the live view, controls and local JSON replay', () => {
  for (const id of ['airsimFrame', 'restart', 'stop', 'close', 'simulationFile', 'playJson']) {
    assert.match(html, new RegExp(`id="${id}"`));
  }
  for (const endpoint of ['/api/camera', '/api/restart', '/api/stop', '/api/close']) {
    assert.ok(html.includes(endpoint), `missing endpoint ${endpoint}`);
  }
});

test('removed experimental controls do not return', () => {
  assert.ok(!html.includes('hw11TelemetryFile'));
  assert.ok(!html.includes('compareRun'));
  assert.ok(!html.includes('cameraDistance'));
  assert.ok(!html.includes('ПОРІВНЯТИ З AIRSIM'));
});

test('camera bridge captures the native AirSim window and is portable', () => {
  const camera = fs.readFileSync(path.join(windows, 'CAMERA_FEED.ps1'), 'utf8');
  const start = fs.readFileSync(path.join(windows, 'START_COURSEWORK.ps1'), 'utf8');
  const restart = fs.readFileSync(path.join(windows, 'RESTART_DEMO.ps1'), 'utf8');
  const server = fs.readFileSync(path.join(windows, 'DASHBOARD_SERVER.ps1'), 'utf8');
  assert.match(camera, /PrintWindow/);
  assert.match(camera, /source = 'native-window'/);
  assert.match(start, /Join-Path \$PSScriptRoot 'CAMERA_FEED\.ps1'/);
  assert.match(restart, /Join-Path \$PSScriptRoot 'CAMERA_FEED\.ps1'/);
  assert.match(server, /Join-Path \$PSScriptRoot 'web\\live'/);
  assert.ok(!start.includes('work\\flight_review_project'));
  assert.ok(!restart.includes('work\\flight_review_project'));
});
