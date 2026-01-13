#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include "PIDController.h"

class TuningWebServer {
public:
    TuningWebServer(uint16_t port = 80);
    
    // Initialize WiFi and start server
    void begin(const char* ssid, const char* password);
    void beginAP(const char* ssid, const char* password = nullptr);
    
    // Set references to PID controllers for tuning
    void setPIDControllers(PIDController* roll, PIDController* pitch, PIDController* yaw);
    void setSettingsManager(SettingsManager* settings) { _settings = settings; }
    
    // Update telemetry data (call from main loop at ~10-50Hz)
    void updateTelemetry(float roll, float pitch, float yaw,
                         float rollRate, float pitchRate, float yawRate,
                         float loopTimeUs, bool armed);
    
    // Update extended telemetry (altitude, heading, temperature)
    void updateExtendedTelemetry(float altitude, float heading, float temperature);
    
    // Check if connected
    bool isConnected() const { return WiFi.status() == WL_CONNECTED || _isAP; }
    String getIP() const;

private:
    AsyncWebServer _server;
    AsyncWebSocket _ws;
    
    PIDController* _rollPID;
    PIDController* _pitchPID;
    PIDController* _yawPID;
    SettingsManager* _settings;
    
    bool _isAP;
    unsigned long _lastBroadcast;
    
    // Telemetry data
    float _roll, _pitch, _yaw;
    float _rollRate, _pitchRate, _yawRate;
    float _loopTimeUs;
    bool _armed;
    
    // Extended telemetry
    float _altitude, _heading, _temperature;
    
    void setupRoutes();
    void handleWebSocketEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                              AwsEventType type, void* arg, uint8_t* data, size_t len);
    String generateHTML();
    void broadcastTelemetry();
};

#endif // WEB_SERVER_H
