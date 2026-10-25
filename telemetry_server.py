import socket
import threading
import json
import time
from flask import Flask, render_template, jsonify, request
from flask_socketio import SocketIO
import os
import re

app = Flask(__name__)
# Standard threading mode for reliability
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

UDP_IP = "0.0.0.0" 
UDP_PORT = 3333

last_sent_ts = -1.0
data_history = []
LOG_DIR = "saved_logs"
if not os.path.exists(LOG_DIR):
    os.makedirs(LOG_DIR)

def save_internal_log():
    global data_history
    if not data_history:
        return
    try:
        filename = f"log_{int(time.time())}.json"
        path = os.path.join(LOG_DIR, filename)
        # Use a copy to avoid thread race conditions during json dump
        history_snapshot = list(data_history)
        with open(path, 'w') as f:
            json.dump(history_snapshot, f, indent=2)
        print(f"--- AUTO-SAVED SESSION: {filename} ({len(history_snapshot)} points) ---")
        data_history.clear() # Clear after saving to prepare for next run
    except Exception as e:
        print(f"AUTO-SAVE ERROR: {e}")

def udp_listener():
    global last_sent_ts
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind((UDP_IP, UDP_PORT))
    except Exception as e:
        print(f"UDP ERROR: {e}")
        return
        
    print(f"UDP LISTENING ON {UDP_PORT}...")
    pattern = re.compile(r"\[\s*(?P<ts>[\d\.]+)\s*s\].*?\[M:(?P<mode>\d+)\s+W:(?P<wing>\d+)\].*?\[S:(?P<s_bits>[01]{5})\s+L:(?P<l_bits>[01]{2})\].*?\[State:(?P<state>\d+)\].*?Pwr:\s*(?P<pwr>-?\d+).*?Yaw:\s*(?P<yaw>[\d\.-]+).*?Ax:\s*(?P<ax>[\d\.-]+)G.*?Impact:(?P<impact>[^|]+)(?:.*?PID:(?P<pid>[\d\.,\-]+))?(?:.*?SE:(?P<se>[\d\.]+))?")        
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            line = data.decode('utf-8', errors='ignore').strip()
            match = pattern.search(line)
            if match:
                ts_val = float(match.group('ts'))
                
                # Check for Start condition (ts=0 when we weren't at 0)
                if ts_val == 0.0 and last_sent_ts > 0.1:
                    print("--> SESSION STOP/RESET DETECTED")
                    save_internal_log() # Save the session that just finished
                    data_history.clear() 
                    socketio.emit('telemetry_reset', {"ts": 0})
                
                # Capture history only if ts > 0 (active session)
                if ts_val > 0.0:
                    payload = {
                        "ts": ts_val,
                        "mode": int(match.group('mode')),
                        "wing": int(match.group('wing')),
                        "sensors": match.group('s_bits'),
                        "lines": match.group('l_bits'),
                        "state": int(match.group('state')),
                        "pwr": int(match.group('pwr')),
                        "yaw": float(match.group('yaw')),
                        "ax": float(match.group('ax')),
                        "impact": match.group('impact').strip(),
                        "se": float(match.group('se')) if match.group('se') else 0.0

                    }
                    data_history.append(payload)
                    socketio.emit('telemetry_update', payload)

                last_sent_ts = ts_val
        except Exception as e:
            print(f"UDP PAYLOAD ERROR: {e}")

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/ota', methods=['POST'])
def ota_upload():
    if 'firmware' not in request.files:
        return jsonify({"error": "no file"}), 400
    f = request.files['firmware']
    data = f.read()
    try:
        import requests as req_lib
        resp = req_lib.post(
            'http://192.168.4.1:8080/ota',
            data=data,
            headers={'Content-Type': 'application/octet-stream'},
            timeout=60
        )
        return jsonify({"status": "success", "response": resp.text})
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/save_log', methods=['POST'])
def save_log():
    save_internal_log()
    return jsonify({"status": "success"})

@app.route('/list_logs')
def list_logs():
    return jsonify(sorted(os.listdir(LOG_DIR), reverse=True))

@app.route('/load_log/<filename>')
def load_log(filename):
    path = os.path.join(LOG_DIR, filename)
    if os.path.exists(path):
        with open(path, 'r') as f:
            return jsonify(json.load(f))
    return jsonify({"error": "file not found"}), 404

@app.route('/update_pid', methods=['POST'])
def update_pid():
    data = request.json
    try:
        # Format string exactly as expected by C sscanf
        msg = f"PID:{data['kps']},{data['kis']},{data['kds']},{data['kpt']},{data['kit']},{data['kdt']}"
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # 192.168.4.1 is the default IP of the ESP32 when acting as a SoftAP
        sock.sendto(msg.encode('utf-8'), ('192.168.4.1', 3334))
        return jsonify({"status": "success", "msg": msg})
    except Exception as e:
        return jsonify({"status": "error", "error": str(e)}), 400

if __name__ == '__main__':
    t = threading.Thread(target=udp_listener, daemon=True)
    t.start()
    print("DASHBOARD STARTING AT http://localhost:5000")
    socketio.run(app, host='0.0.0.0', port=5000, log_output=False)
