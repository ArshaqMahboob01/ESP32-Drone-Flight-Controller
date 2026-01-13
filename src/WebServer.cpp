#include "WebServer.h"

TuningWebServer::TuningWebServer(uint16_t port) 
    : _server(port), _ws("/ws"), _isAP(false), _lastBroadcast(0),
      _rollPID(nullptr), _pitchPID(nullptr), _yawPID(nullptr), _settings(nullptr),
      _roll(0), _pitch(0), _yaw(0), _rollRate(0), _pitchRate(0), _yawRate(0),
      _loopTimeUs(0), _armed(false), _altitude(0), _heading(0), _temperature(0) {}

void TuningWebServer::begin(const char* ssid, const char* password) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    
    Serial.print("Connecting to WiFi");
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected!");
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());
        _isAP = false;
        
        if (MDNS.begin("drone")) {
            Serial.println("MDNS responder started: http://drone.local");
        }
    } else {
        Serial.println("\nWiFi connection failed, starting AP mode fallback...");
        beginAP("DroneConfig");
    }
    
    setupRoutes();
    _server.begin();
}

void TuningWebServer::beginAP(const char* ssid, const char* password) {
    WiFi.mode(WIFI_AP);
    if (password) {
        WiFi.softAP(ssid, password);
    } else {
        WiFi.softAP(ssid);
    }
    
    Serial.println("AP Mode started!");
    Serial.print("SSID: ");
    Serial.println(ssid);
    Serial.print("IP: ");
    Serial.println(WiFi.softAPIP());
    
    if (MDNS.begin("drone")) {
        Serial.println("MDNS responder started: http://drone.local");
    }
    
    _isAP = true;
    
    setupRoutes();
    _server.begin();
}

String TuningWebServer::getIP() const {
    if (_isAP) {
        return WiFi.softAPIP().toString();
    }
    return WiFi.localIP().toString();
}

void TuningWebServer::setPIDControllers(PIDController* roll, PIDController* pitch, PIDController* yaw) {
    _rollPID = roll;
    _pitchPID = pitch;
    _yawPID = yaw;
}

void TuningWebServer::updateTelemetry(float roll, float pitch, float yaw,
                                       float rollRate, float pitchRate, float yawRate,
                                       float loopTimeUs, bool armed) {
    _roll = roll;
    _pitch = pitch;
    _yaw = yaw;
    _rollRate = rollRate;
    _pitchRate = pitchRate;
    _yawRate = yawRate;
    _loopTimeUs = loopTimeUs;
    _armed = armed;
    
    // Broadcast at ~20Hz
    if (millis() - _lastBroadcast > 50) {
        broadcastTelemetry();
        _lastBroadcast = millis();
    }
}

void TuningWebServer::updateExtendedTelemetry(float altitude, float heading, float temperature) {
    _altitude = altitude;
    _heading = heading;
    _temperature = temperature;
}

void TuningWebServer::broadcastTelemetry() {
    if (_ws.count() == 0) return;
    
    StaticJsonDocument<384> doc;
    doc["roll"] = _roll;
    doc["pitch"] = _pitch;
    doc["yaw"] = _yaw;
    doc["rollRate"] = _rollRate;
    doc["pitchRate"] = _pitchRate;
    doc["yawRate"] = _yawRate;
    doc["loopUs"] = _loopTimeUs;
    doc["armed"] = _armed;
    doc["altitude"] = _altitude;
    doc["heading"] = _heading;
    doc["temp"] = _temperature;
    
    String json;
    serializeJson(doc, json);
    _ws.textAll(json);
}

void TuningWebServer::setupRoutes() {
    // WebSocket handler
    _ws.onEvent([this](AsyncWebSocket* server, AsyncWebSocketClient* client,
                       AwsEventType type, void* arg, uint8_t* data, size_t len) {
        handleWebSocketEvent(server, client, type, arg, data, len);
    });
    _server.addHandler(&_ws);
    
    // Main page
    _server.on("/", HTTP_GET, [this](AsyncWebServerRequest* request) {
        request->send(200, "text/html", generateHTML());
    });
    
    // Get current PID values
    _server.on("/pid", HTTP_GET, [this](AsyncWebServerRequest* request) {
        StaticJsonDocument<512> doc;
        
        if (_rollPID) {
            doc["roll"]["kp"] = _rollPID->getP();
            doc["roll"]["ki"] = _rollPID->getI();
            doc["roll"]["kd"] = _rollPID->getD();
        }
        if (_pitchPID) {
            doc["pitch"]["kp"] = _pitchPID->getP();
            doc["pitch"]["ki"] = _pitchPID->getI();
            doc["pitch"]["kd"] = _pitchPID->getD();
        }
        if (_yawPID) {
            doc["yaw"]["kp"] = _yawPID->getP();
            doc["yaw"]["ki"] = _yawPID->getI();
            doc["yaw"]["kd"] = _yawPID->getD();
        }
        
        String json;
        serializeJson(doc, json);
        request->send(200, "application/json", json);
    });
    
    // Set PID values
    _server.on("/pid", HTTP_POST, [](AsyncWebServerRequest* request) {},
        nullptr,
        [this](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
            StaticJsonDocument<256> doc;
            DeserializationError error = deserializeJson(doc, data, len);
            
            if (error) {
                request->send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
                return;
            }
            
            const char* axis = doc["axis"];
            float kp = doc["kp"];
            float ki = doc["ki"];
            float kd = doc["kd"];
            float imax = doc["imax"] | 50.0f;
            
            PIDController* pid = nullptr;
            if (strcmp(axis, "roll") == 0) pid = _rollPID;
            else if (strcmp(axis, "pitch") == 0) pid = _pitchPID;
            else if (strcmp(axis, "yaw") == 0) pid = _yawPID;
            
            if (pid) {
                pid->setGains(kp, ki, kd);
                pid->setIntegralMax(imax);
                request->send(200, "application/json", "{\"status\":\"ok\"}");
            } else {
                request->send(400, "application/json", "{\"error\":\"Invalid axis\"}");
            }
        });
        
    // Save PID values to flash
    _server.on("/save", HTTP_POST, [this](AsyncWebServerRequest* request) {
        if (_settings && _rollPID && _pitchPID && _yawPID) {
            _settings->saveAll(*_rollPID, *_pitchPID, *_yawPID);
            request->send(200, "application/json", "{\"status\":\"ok\"}");
        } else {
            request->send(500, "application/json", "{\"error\":\"SettingsManager not initialized\"}");
        }
    });
}

void TuningWebServer::handleWebSocketEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                                           AwsEventType type, void* arg, uint8_t* data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        Serial.printf("WebSocket client #%u connected\n", client->id());
    } else if (type == WS_EVT_DISCONNECT) {
        Serial.printf("WebSocket client #%u disconnected\n", client->id());
    }
}

String TuningWebServer::generateHTML() {
    return R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Drone PID Tuning</title>
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            background: linear-gradient(135deg, #1a1a2e 0%, #16213e 100%);
            color: #fff;
            min-height: 100vh;
            padding: 20px;
        }
        h1 { text-align: center; margin-bottom: 20px; color: #00d9ff; }
        .container { max-width: 900px; margin: 0 auto; }
        
        .telemetry {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(120px, 1fr));
            gap: 12px;
            margin-bottom: 25px;
        }
        .telemetry-card {
            background: rgba(255,255,255,0.1);
            border-radius: 12px;
            padding: 12px;
            text-align: center;
            backdrop-filter: blur(10px);
        }
        .telemetry-card .label { font-size: 11px; opacity: 0.7; text-transform: uppercase; }
        .telemetry-card .value { font-size: 24px; font-weight: bold; color: #00d9ff; }
        .telemetry-card .unit { font-size: 11px; opacity: 0.5; }
        .telemetry-card.alt .value { color: #44ff88; }
        .telemetry-card.temp .value { color: #ffaa44; }
        
        .status { text-align: center; margin-bottom: 15px; font-size: 18px; font-weight: bold; }
        .status.armed { color: #ff4444; }
        .status.disarmed { color: #44ff44; }
        
        .pid-section {
            background: rgba(255,255,255,0.05);
            border-radius: 12px;
            padding: 15px;
            margin-bottom: 15px;
        }
        .pid-section h2 { margin-bottom: 12px; font-size: 16px; color: #00d9ff; }
        .pid-grid {
            display: grid;
            grid-template-columns: repeat(4, 1fr);
            gap: 8px;
        }
        .pid-input label { font-size: 11px; opacity: 0.7; }
        .pid-input input {
            width: 100%;
            background: rgba(0,0,0,0.3);
            border: 1px solid rgba(255,255,255,0.2);
            border-radius: 6px;
            padding: 8px;
            color: #fff;
            font-size: 14px;
        }
        .pid-input input:focus { outline: none; border-color: #00d9ff; }
        
        button {
            background: linear-gradient(135deg, #00d9ff, #0066ff);
            border: none;
            border-radius: 6px;
            padding: 10px 20px;
            color: #fff;
            font-size: 13px;
            cursor: pointer;
            margin-top: 10px;
            transition: transform 0.1s;
        }
        button:hover { opacity: 0.9; transform: scale(1.02); }
        button:active { transform: scale(0.98); }
        button.save { background: linear-gradient(135deg, #44ff88, #00aa44); margin-top: 20px; width: 100%; font-weight: bold; }
        
        .loop-time {
            position: fixed;
            bottom: 15px;
            right: 15px;
            background: rgba(0,0,0,0.6);
            padding: 8px 12px;
            border-radius: 6px;
            font-size: 11px;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>🚁 Drone PID Tuning</h1>
        
        <div id="status" class="status disarmed">DISARMED</div>
        
        <div class="telemetry">
            <div class="telemetry-card">
                <div class="label">Roll</div>
                <div class="value" id="roll">0.0</div>
                <div class="unit">deg</div>
            </div>
            <div class="telemetry-card">
                <div class="label">Pitch</div>
                <div class="value" id="pitch">0.0</div>
                <div class="unit">deg</div>
            </div>
            <div class="telemetry-card">
                <div class="label">Yaw</div>
                <div class="value" id="yaw">0.0</div>
                <div class="unit">deg</div>
            </div>
            <div class="telemetry-card">
                <div class="label">Heading</div>
                <div class="value" id="heading">0</div>
                <div class="unit">deg</div>
            </div>
            <div class="telemetry-card alt">
                <div class="label">Altitude</div>
                <div class="value" id="altitude">0.0</div>
                <div class="unit">m</div>
            </div>
            <div class="telemetry-card temp">
                <div class="label">Temp</div>
                <div class="value" id="temp">0.0</div>
                <div class="unit">°C</div>
            </div>
        </div>
        
        <div class="pid-section">
            <h2>Roll PID</h2>
            <div class="pid-grid">
                <div class="pid-input"><label>Kp</label><input type="number" id="roll_kp" step="0.1" value="1.0"></div>
                <div class="pid-input"><label>Ki</label><input type="number" id="roll_ki" step="0.01" value="0.0"></div>
                <div class="pid-input"><label>Kd</label><input type="number" id="roll_kd" step="0.01" value="0.0"></div>
                <div class="pid-input"><label>I-Max</label><input type="number" id="roll_imax" step="1" value="50"></div>
            </div>
            <button onclick="updatePID('roll')">Apply</button>
        </div>
        
        <div class="pid-section">
            <h2>Pitch PID</h2>
            <div class="pid-grid">
                <div class="pid-input"><label>Kp</label><input type="number" id="pitch_kp" step="0.1" value="1.0"></div>
                <div class="pid-input"><label>Ki</label><input type="number" id="pitch_ki" step="0.01" value="0.0"></div>
                <div class="pid-input"><label>Kd</label><input type="number" id="pitch_kd" step="0.01" value="0.0"></div>
                <div class="pid-input"><label>I-Max</label><input type="number" id="pitch_imax" step="1" value="50"></div>
            </div>
            <button onclick="updatePID('pitch')">Apply</button>
        </div>
        
        <div class="pid-section">
            <h2>Yaw PID</h2>
            <div class="pid-grid">
                <div class="pid-input"><label>Kp</label><input type="number" id="yaw_kp" step="0.1" value="2.0"></div>
                <div class="pid-input"><label>Ki</label><input type="number" id="yaw_ki" step="0.01" value="0.0"></div>
                <div class="pid-input"><label>Kd</label><input type="number" id="yaw_kd" step="0.01" value="0.0"></div>
                <div class="pid-input"><label>I-Max</label><input type="number" id="yaw_imax" step="1" value="50"></div>
            </div>
            <button onclick="updatePID('yaw')">Apply</button>
        </div>
        
        <button class="save" onclick="saveSettings()">💾 Save PID Settings to Flash</button>
    </div>
    
    <div class="loop-time">Loop: <span id="loopTime">0</span> µs</div>
    
    <script>
        const ws = new WebSocket(`ws://${window.location.host}/ws`);
        
        ws.onmessage = (event) => {
            const d = JSON.parse(event.data);
            document.getElementById('roll').textContent = d.roll.toFixed(1);
            document.getElementById('pitch').textContent = d.pitch.toFixed(1);
            document.getElementById('yaw').textContent = d.yaw.toFixed(1);
            document.getElementById('heading').textContent = Math.round(d.heading);
            document.getElementById('altitude').textContent = d.altitude.toFixed(1);
            document.getElementById('temp').textContent = d.temp.toFixed(1);
            document.getElementById('loopTime').textContent = Math.round(d.loopUs);
            
            const s = document.getElementById('status');
            s.textContent = d.armed ? 'ARMED' : 'DISARMED';
            s.className = 'status ' + (d.armed ? 'armed' : 'disarmed');
        };
        
        async function updatePID(axis) {
            const kp = parseFloat(document.getElementById(`${axis}_kp`).value);
            const ki = parseFloat(document.getElementById(`${axis}_ki`).value);
            const kd = parseFloat(document.getElementById(`${axis}_kd`).value);
            const imax = parseFloat(document.getElementById(`${axis}_imax`).value);
            
            const r = await fetch('/pid', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({ axis, kp, ki, kd, imax })
            });
            if ((await r.json()).status === 'ok') alert(`${axis.toUpperCase()} PID updated!`);
        }
        
        async function saveSettings() {
            const r = await fetch('/save', { method: 'POST' });
            if ((await r.json()).status === 'ok') alert('Settings saved to flash memory!');
            else alert('Error saving settings');
        }
    </script>
</body>
</html>
)rawliteral";
}
