#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <HardwareSerial.h>

// --- Configuration ---
const char* ssid = "********";
const char* password = "************";
const int wsPort = 8080;

// STM32 Serial 통신 설정 (UART2)
#define STM32_RX_PIN 16 
#define STM32_TX_PIN 17 
const long STM32_BAUD_RATE = 115200; 

// --- Global Objects ---
WebSocketsServer webSocket = WebSocketsServer(wsPort);
HardwareSerial STM32_Serial(2);

// Keep-Alive 타이머 (수동 Ping 로직 제거를 위해 주석 처리하거나 제거 가능)
// unsigned long lastPingTime = 0;
// const unsigned long PING_INTERVAL = 15000; // 15초마다 ping

// 버퍼를 사용하여 STM32 응답을 한 번에 읽음
String stm32_response_buffer = ""; 

// WebSocket 이벤트 핸들러 함수
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED:
            Serial.printf("[%u] Disconnected!\n", num);
            break;
            
        case WStype_CONNECTED: {
            IPAddress ip = webSocket.remoteIP(num);
            Serial.printf("[%u] WebSocket Connected from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
            webSocket.sendTXT(num, "Server Ready.");
        }
            break;
            
        case WStype_TEXT: {
            String commandString = String((char*)payload);
            commandString.trim();
            Serial.printf("[%u] Received: %s\n", num, commandString.c_str());

            if (commandString.length() > 0) {
                // STM32로 전송 시, '\n' 대신 '!'와 같은 명확한 종결 문자 사용 권장
                STM32_Serial.print(commandString);
                STM32_Serial.print('\n'); // 기존 코드와 동일하게 '\n' 사용
                Serial.printf("Forwarded to STM32: %s\n", commandString.c_str());
            }
        }
            break;
            
        case WStype_PING:
            // Serial.printf("[%u] Received Ping\n", num); // 너무 자주 출력되면 성능 저하 유발
            break;
            
        case WStype_PONG:
            // Serial.printf("[%u] Received Pong\n", num); // 너무 자주 출력되면 성능 저하 유발
            break;
            
        default:
            break;
    }
}

void setup_wifi() {
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);
    
    // Wi-Fi 자동 재연결 활성화
    WiFi.setAutoReconnect(true);
    WiFi.persistent(true);

    int countdown = 30;
    while (WiFi.status() != WL_CONNECTED && countdown > 0) {
        delay(1000);
        Serial.print(".");
        countdown--;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected successfully.");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\nWiFi connection failed.");
    }
}

void setup() {
    Serial.begin(115200);
    delay(100);

    STM32_Serial.begin(STM32_BAUD_RATE, SERIAL_8N1, STM32_RX_PIN, STM32_TX_PIN);
    Serial.println("Serial2 initialized for STM32 communication.");

    setup_wifi();

    // WebSocket 서버 설정
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
    
    // 🚨 Heartbeat 설정 수정 (Ping 15초, Pong 타임아웃 5초, 3회 실패 시 연결 종료)
    // 이전 설정(3000ms)보다 타임아웃을 여유 있게 늘려 끊김 방지
    webSocket.enableHeartbeat(15000, 5000, 3);
    
    Serial.print("WebSocket Server started on port: ");
    Serial.println(wsPort);
}

void loop() {
    // Wi-Fi 연결 상태 체크 및 재연결 (OK)
    if (WiFi.status() != WL_CONNECTED) {
        // 너무 잦은 재연결 시도를 막기 위해 짧은 딜레이 추가
        delay(1000); 
        Serial.println("WiFi disconnected! Reconnecting...");
        setup_wifi();
    }
    
    // WebSocket 서버 루프 (필수)
    webSocket.loop();
    
    // 🚫 수동 Ping 로직 제거 (Heartbeat 기능이 이미 활성화되어 있어 중복 및 충돌 가능성 제거)
    
    // STM32 응답 처리 (개선된 방식)
    while (STM32_Serial.available()) {
        char incomingChar = STM32_Serial.read();
        
        if (incomingChar == '\n') {
            // 줄바꿈 문자를 만나면 한 줄의 응답이 끝난 것으로 간주
            stm32_response_buffer.trim();

            if (stm32_response_buffer.length() > 0) {
                Serial.print("STM32 Response: ");
                Serial.println(stm32_response_buffer);
                // 모든 연결된 클라이언트에게 응답 브로드캐스트
                webSocket.broadcastTXT(stm32_response_buffer);
            }
            stm32_response_buffer = ""; // 버퍼 초기화
        } else {
            // 문자열 버퍼에 문자 추가
            stm32_response_buffer += incomingChar;
        }
    }

    delay(1); // 루프 지연 시간 유지
}