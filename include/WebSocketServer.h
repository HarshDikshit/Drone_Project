#ifndef WEBSOCKET_SERVER_H
#define WEBSOCKET_SERVER_H

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSocketsServer.h>

class WebSocketServer {
private:
    AsyncWebServer server;
    WebSocketsServer webSocket;
    void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);

public:
    WebSocketServer();
    void begin();
    void sendData(const char* data);
};

#endif