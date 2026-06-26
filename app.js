let btDevice = null, btChar = null, streaming = false, camRotation = 0, autoMode = false;

function toggleAutonomous() {
  autoMode = !autoMode;
  const btn = document.getElementById('autoBtn');
  const card = document.querySelector('.auto-card');
  const status = document.getElementById('autoStatus');
  const sub = document.getElementById('autoSub');
  const dpadBtns = document.querySelectorAll('.dpad-btn');
  const actionBtns = document.querySelectorAll('.action-btn');

  if (autoMode) {
    sendCmd('A');
    btn.classList.add('active');
    card.classList.add('active');
    status.textContent = 'ON';
    status.classList.add('on');
    sub.textContent = 'Robot is navigating autonomously';
    dpadBtns.forEach(b => { b.style.opacity = '0.3'; b.style.pointerEvents = 'none'; });
    actionBtns.forEach(b => { b.style.opacity = '0.3'; b.style.pointerEvents = 'none'; });
    logSerial('AUTONOMOUS MODE ON — obstacle avoidance active');
  } else {
    sendCmd('M');
    btn.classList.remove('active');
    card.classList.remove('active');
    status.textContent = 'OFF';
    status.classList.remove('on');
    sub.textContent = 'Tap to enable autonomous walking';
    dpadBtns.forEach(b => { b.style.opacity = '1'; b.style.pointerEvents = 'auto'; });
    actionBtns.forEach(b => { b.style.opacity = '1'; b.style.pointerEvents = 'auto'; });
    logSerial('MANUAL MODE — controls re-enabled');
  }
}

function switchTab(name) {
  document.querySelectorAll('.tab').forEach(t => t.classList.toggle('active', t.dataset.tab === name));
  document.querySelectorAll('.tab-content').forEach(c => c.classList.toggle('active', c.id === 'tab-' + name));
}

function sendCmd(cmd) {
  logSerial('> ' + cmd);
  if (btChar) {
    const enc = new TextEncoder();
    btChar.writeValue(enc.encode(cmd)).catch(e => logSerial('ERR: ' + e.message));
  }
}

function updateSpeed(val) {
  document.getElementById('speedVal').textContent = val + '%';
  sendCmd('V' + val);
}

async function toggleBluetooth() {
  const btn = document.getElementById('btBtn');
  const dot = document.getElementById('statusDot');
  const stxt = document.getElementById('statusText');
  const info = document.getElementById('btInfo');

  if (btDevice && btDevice.gatt.connected) {
    btDevice.gatt.disconnect();
    btDevice = null; btChar = null;
    btn.innerHTML = '<span class="material-icons-round">bluetooth_searching</span>Connect HC-05';
    btn.classList.remove('connected-btn');
    dot.classList.remove('connected');
    stxt.textContent = 'Disconnected';
    info.textContent = 'Not connected';
    logSerial('Bluetooth disconnected');
    return;
  }

  try {
    logSerial('Searching for Bluetooth devices...');
    btDevice = await navigator.bluetooth.requestDevice({
      filters: [{ services: ['0000ffe0-0000-1000-8000-00805f9b34fb'] }],
      optionalServices: ['0000ffe0-0000-1000-8000-00805f9b34fb']
    });

    logSerial('Connecting to ' + btDevice.name + '...');
    const server = await btDevice.gatt.connect();
    const service = await server.getPrimaryService('0000ffe0-0000-1000-8000-00805f9b34fb');
    btChar = await service.getCharacteristic('0000ffe1-0000-1000-8000-00805f9b34fb');

    await btChar.startNotifications();
    btChar.addEventListener('characteristicvaluechanged', e => {
      const dec = new TextDecoder();
      logSerial('< ' + dec.decode(e.target.value));
    });

    btn.innerHTML = '<span class="material-icons-round">bluetooth_connected</span>Disconnect';
    btn.classList.add('connected-btn');
    dot.classList.add('connected');
    stxt.textContent = 'Connected';
    info.textContent = 'Connected to ' + btDevice.name;
    logSerial('Connected to ' + btDevice.name);

    btDevice.addEventListener('gattserverdisconnected', () => {
      btDevice = null; btChar = null;
      btn.innerHTML = '<span class="material-icons-round">bluetooth_searching</span>Connect HC-05';
      btn.classList.remove('connected-btn');
      dot.classList.remove('connected');
      stxt.textContent = 'Disconnected';
      info.textContent = 'Not connected';
      logSerial('Device disconnected');
    });
  } catch (e) {
    logSerial('BT Error: ' + e.message);
    info.textContent = 'Error: ' + e.message;
  }
}

function toggleStream() {
  const btn = document.getElementById('camConnectBtn');
  const feed = document.getElementById('camFeed');
  const ph = document.getElementById('camPlaceholder');
  const url = document.getElementById('camUrl').value.trim();

  if (streaming) {
    feed.src = '';
    feed.style.display = 'none';
    ph.style.display = 'flex';
    btn.innerHTML = '<span class="material-icons-round">play_arrow</span>';
    btn.classList.remove('streaming');
    streaming = false;
    logSerial('Camera stream stopped');
    return;
  }

  if (!url) {
    logSerial('Please enter a camera URL');
    document.getElementById('camUrl').focus();
    return;
  }

  logSerial('Connecting to camera: ' + url);
  feed.src = url;
  feed.style.display = 'block';
  ph.style.display = 'none';
  btn.innerHTML = '<span class="material-icons-round">stop</span>';
  btn.classList.add('streaming');
  streaming = true;

  feed.onerror = () => {
    logSerial('Camera feed error - check URL');
  };
}

function toggleFullscreen() {
  const vp = document.getElementById('camViewport');
  if (!document.fullscreenElement) {
    vp.requestFullscreen().catch(() => {});
  } else {
    document.exitFullscreen();
  }
}

function captureSnapshot() {
  const feed = document.getElementById('camFeed');
  if (!streaming || !feed.src) { logSerial('No active stream to capture'); return; }
  const canvas = document.getElementById('snapCanvas');
  canvas.width = feed.naturalWidth || 640;
  canvas.height = feed.naturalHeight || 480;
  const ctx = canvas.getContext('2d');
  try {
    ctx.drawImage(feed, 0, 0);
    const link = document.createElement('a');
    link.download = 'spider_cam_' + Date.now() + '.png';
    link.href = canvas.toDataURL('image/png');
    link.click();
    logSerial('Snapshot saved');
  } catch (e) {
    logSerial('Snapshot failed (CORS): ' + e.message);
  }
}

function rotateStream() {
  camRotation = (camRotation + 90) % 360;
  document.getElementById('camFeed').style.transform = 'rotate(' + camRotation + 'deg)';
  logSerial('Stream rotated to ' + camRotation + '°');
}

function logSerial(msg) {
  const box = document.getElementById('serialBox');
  const ts = new Date().toLocaleTimeString('en-US', { hour12: false });
  box.innerHTML += '<div>[' + ts + '] ' + msg + '</div>';
  box.scrollTop = box.scrollHeight;
}

function sendSerial() {
  const input = document.getElementById('serialInput');
  const val = input.value.trim();
  if (!val) return;
  sendCmd(val);
  input.value = '';
}

document.addEventListener('keydown', e => {
  const map = { ArrowUp: 'F', ArrowDown: 'B', ArrowLeft: 'L', ArrowRight: 'R', ' ': 'S' };
  if (map[e.key] && !e.repeat) sendCmd(map[e.key]);
});

document.addEventListener('keyup', e => {
  const map = { ArrowUp: true, ArrowDown: true, ArrowLeft: true, ArrowRight: true };
  if (map[e.key]) sendCmd('S');
});

logSerial('Spider Robot Controller ready');
logSerial('Connect via Bluetooth or use keyboard arrows');
