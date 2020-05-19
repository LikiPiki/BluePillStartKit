#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>

#define LEN_BYTES 6
#define SERVER_IP "192.168.1.226";
#define SEC 1000
#define DELAY_SECONDS 60

#define USE_SERIAL Serial

float T = 0;
float V = 0;
float H1 = 0;
float H2 = 0;

char charByte;
char comingNum[3];

// For spliting IP adress to bytes
int firstByte = 0;
int secondByte = 0;

void sendToServer() {
  WiFiClient client;
  HTTPClient http;
  
  http.begin(client, "http://192.168.1.219:3333"); //HTTP
  http.addHeader("Content-Type", "application/json");
  String tempstr = String("{\"T\":") +  T + String(", \"H1\": ") + H1 + String(", \"H2\": ") + H2 + String(", \"V\": ") + V + String("}");
  int httpCode = http.POST(tempstr);
}

float readData() {
  float result = 0;
  uint8_t i =  0;
  for(; i < 3; i++) {
    comingNum[i] = Serial.read();
  }
  result = atoi(comingNum);
  comingNum[0] = '0';
  for(i = 1; i < 3; i++) {
    comingNum[i] = Serial.read();
  }
  result += (float) (atoi(comingNum)) / 100;
  return result;
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  WiFi.begin("airCube", "andrey99");

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
  }
  delay(1000);

  IPAddress ip = WiFi.localIP();
  char msg[3] = {'I', char(ip[2]), char(ip[3])};
  Serial.write(msg, 3);
  digitalWrite(LED_BUILTIN, 1);  
}


void loop() {
  while(Serial.available() >= LEN_BYTES) {
        charByte = Serial.read();
        if (charByte == 'T') {
          T = readData();
        }
        if (charByte == 'H') {
          H1 = readData();
        }
        if (charByte == 'h') {
          H2 = readData();
        }
        if (charByte == 'V') {
          V = readData();
        }
        if (charByte == 'O') {
          while(Serial.available()) {
            Serial.read();
          }
          digitalWrite(LED_BUILTIN, 0);  
          sendToServer();
          digitalWrite(LED_BUILTIN, 1);  
        }
    }
    
    delay(SEC * DELAY_SECONDS);
    Serial.print("GET");
    delay(SEC);
}
