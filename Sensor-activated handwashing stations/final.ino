#include <WiFi.h>
#include <PubSubClient.h>
#include <HCSR04.h>

// 超聲波感測器定義
#define CURRENT_TEMPERATURE 26.0
#define PIN_TRIG  16
#define PIN_ECHO  18
#define PIN_TEMP  17
#define PIN_BUZZER  26
#define DISTANCE_FAR  50  // 將距離閾值設置得遠一些
#define DISTANCE_NEAR 10   // cm

//水庫定義(ml)
unsigned int reservoir = 5000;
bool touch=0;
// 水流感測器定義
const int flowPin = 34;  // 流量感測器信號接腳
volatile int pulseCount;  // 中斷服務程序中會改變的變數
float flowRate;
unsigned long flowMilliLitres;
unsigned int totalMilliLitres;
unsigned long lastFlowRateCheck = 0;

// 創建超聲波距離感測器對象
UltraSonicDistanceSensor distanceSensor(PIN_TRIG, PIN_ECHO);
// WiFi AP ssid / password
static const char* ssid = "iot2.4g";  // Change this to your WiFi SSID
static const char* password = "12345678";  // Change this to your WiFi password

// MQTT Broker info
static char *server = "140.127.196.39";
int port = 18883;


// MQTT topics
#define TOPIC_INFO  "nine/info"
#define TOPIC_TOTL  "nine/total"
#define TOPIC_RESE  "nine/reservoir"
#define TOPIC_BTON  "nine/touch"

// MQTT Client info
static char *client_id = "nine_11"; 
const char* mqtt_user = "iot2024";
const char* mqtt_password = "isuCSIE2024#";
// Clients for MQTT
WiFiClient wifiClient;
PubSubClient client(wifiClient);

// 流量感測器中斷服務程序
void IRAM_ATTR pulseCounter() {
  pulseCount++;
}

void callback(char* topic, byte* payload, unsigned int length) {
  // 輸出收到的消息到串口終端
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i=0; i<length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  if(!strcmp(topic, TOPIC_BTON)){
     touch=1;
      }

}

void reconnect() {
  // 持續嘗試連接到MQTT broker，直到成功
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // 嘗試連接
    if (client.connect(client_id,mqtt_user,mqtt_password)) {
     // Serial.println("connected");
     client.subscribe(TOPIC_BTON);
      // 一旦連接，發布一個公告
      //client.publish(TOPIC_INFO, "sensor node ready ...");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // 等待5秒後重試
      delay(5000);
    }
  }
}

void setup() {
  // 初始化串行端口
  Serial.begin(115200);

  // 初始化超聲波感測器
  pinMode(PIN_TEMP, OUTPUT);
  
  // 設置水流感測器的引腳模式和中斷
  pinMode(flowPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(flowPin), pulseCounter, FALLING);

  // 初始化WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // 設置MQTT客戶端
  client.setServer(server, port); // 設置伺服器地址和端口
  client.setCallback(callback); // 設置回調函數

  // 初始化計時變量
  lastFlowRateCheck = millis();
}

void loop() {
  unsigned long current_time;

  // 檢查MQTT broker連接狀態
  // 如果斷開連接，重新連接到broker
  if (!client.connected()) {
    reconnect();
  }

  if ((millis() - lastFlowRateCheck) > 1000) {  
    // 超聲波感測器功能
    double distance = distanceSensor.measureDistanceCm(CURRENT_TEMPERATURE);
    //Serial.print(distance);
    if (distance >= 0) {
      

      if (distance > DISTANCE_FAR) {
      digitalWrite(PIN_TEMP, LOW);
    } else if (distance < DISTANCE_NEAR) {
      digitalWrite(PIN_TEMP, HIGH);

      // 水流感測器功能
      detachInterrupt(digitalPinToInterrupt(flowPin));
      
      flowRate = (pulseCount /98)+11;  

      totalMilliLitres += flowRate;
      
      unsigned int frac = (int(flowRate * 10) % 10);
      
      Serial.print("流量: ");
      Serial.print(int(flowRate));
      Serial.print(".");
      Serial.print(frac, DEC);
      Serial.print("L/min\t總流量: ");
      Serial.print(totalMilliLitres);
      Serial.println("mL");

      float temp = float(totalMilliLitres);
      Serial.print("總毫升數: ");
      Serial.println(temp);
      char buf[10];
      
      sprintf(buf, "%.2f", temp);
      client.publish(TOPIC_TOTL, buf);

      // 水庫-總流量=目前水量

      /*if (reservoir > 0 && reservoir <= 5000) */
        Serial.println("water");
        Serial.print("目前水庫水量: ");
        Serial.println(5000-temp);
        float dd=5000-temp;
        sprintf(buf, "%.2f", dd);
        client.publish(TOPIC_RESE, buf);
        if (touch){
          touch=0;
          totalMilliLitres=0;
          temp=totalMilliLitres;
          dd=5000;
          sprintf(buf, "%.2f", dd);
          client.publish(TOPIC_RESE, buf);
          sprintf(buf, "%.2f", temp);
          client.publish(TOPIC_TOTL, buf);

      }
      

      
      pulseCount = 0;
      attachInterrupt(digitalPinToInterrupt(flowPin), pulseCounter, FALLING);
      lastFlowRateCheck = millis();
     }
    }
  }
  // 保持MQTT處理
  client.loop();
}
