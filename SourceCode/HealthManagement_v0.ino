/*********************
 * Date: Dec 4th, 2022
 * Studnets: Bhaskar et al
 * RTS Assignment-1
 ***********************/
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>
/* macros */
#define LED_GPIO 2
#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif

AsyncWebServer server(80);

const char* ssid = "CB";          
const char* password = "87654321"; 

static SemaphoreHandle_t mutex; //Object instantiation
unsigned int tempValue, BPMValue; //global read only
int AnalogPinBPM=34, AnalogPinTemp=35; // pins
/*==================================================================*/

void message(uint8_t *data, size_t len){ // Hardware ISR() hight priority by default
  WebSerial.print("Hardware Interrupt>>");
  String Data = "";
  for(int i=0; i < len; i++){
    Data += char(data[i]);
  }
  WebSerial.println(Data);
  if (Data == "EMERGENCY"){
    digitalWrite(LED_GPIO, HIGH);
  }
  if (Data=="NORMAL"){
    digitalWrite(LED_GPIO, LOW);
  }
}

void TaskBPM(void *parameters) // This task - BPM value is Published via WiFi
{

  while (1)
  {
    if (xSemaphoreTake(mutex, 0) == pdTRUE) { // lock the critical section/resource if section is available
    
    if(WiFi.status() != WL_CONNECTED){Serial.println("Wifi disconnected>");}
    else
      {
        WebSerial.print("B"); vTaskDelay(500 / portTICK_PERIOD_MS);
        WebSerial.print("P"); vTaskDelay(500 / portTICK_PERIOD_MS);
        WebSerial.print("M:"); vTaskDelay(500 / portTICK_PERIOD_MS);
        WebSerial.println(BPMValue); vTaskDelay(500 / portTICK_PERIOD_MS);

        xSemaphoreGive(mutex); // Give mutex after critical section

        vTaskDelay(1000 / portTICK_PERIOD_MS); // non-blocking delay (holds the task till 500ms)
      }
    }
  else  vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void TaskTEMP(void *parameters) // This task - Reads Temperature -> Publish via WiFi
{
  while (1)
  {
    if (xSemaphoreTake(mutex, 0) == pdTRUE) { // lock the critical section/resource if section is available
    
    if(WiFi.status() != WL_CONNECTED){Serial.println("Wifi disconnected<");}
    else
      {
        WebSerial.print("T"); vTaskDelay(500 / portTICK_PERIOD_MS);
        WebSerial.print("E"); vTaskDelay(500 / portTICK_PERIOD_MS);
        WebSerial.print("MP:"); vTaskDelay(500 / portTICK_PERIOD_MS);
        WebSerial.print(tempValue); vTaskDelay(500 / portTICK_PERIOD_MS);
        WebSerial.println("°C"); vTaskDelay(500 / portTICK_PERIOD_MS);

        xSemaphoreGive(mutex); // Give mutex after critical section

        vTaskDelay(1000 / portTICK_PERIOD_MS); // non-blocking delay (holds the task till 500ms)
      }
    }
  else  vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void TaskBPM_withoutMuTex(void *parameters) // This task - Reads BPM -> Publish via WiFi
{

  while (1)
  {
    if (true) { // critical section/resource accessed
    
    if(WiFi.status() != WL_CONNECTED){Serial.println("Wifi disconnected>");}
    else
      {
        WebSerial.print("B"); vTaskDelay(500 / portTICK_PERIOD_MS);
        WebSerial.print("P"); vTaskDelay(500 / portTICK_PERIOD_MS);
        WebSerial.print("M:"); vTaskDelay(500 / portTICK_PERIOD_MS);
        WebSerial.println(BPMValue); vTaskDelay(500 / portTICK_PERIOD_MS);

        vTaskDelay(1000 / portTICK_PERIOD_MS); // non-blocking delay (holds the task till 500ms)
      }
    }
  else  vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void TaskTEMP_withoutMuTex(void *parameters) // This task - Reads Temperature -> Publish via WiFi
{
  while (1)
  {
    if (true) { // the critical section/resource
    
    if(WiFi.status() != WL_CONNECTED){Serial.println("Wifi disconnected<");}
    else
      {
        WebSerial.print("T"); vTaskDelay(500 / portTICK_PERIOD_MS);
        WebSerial.print("E"); vTaskDelay(500 / portTICK_PERIOD_MS);
        WebSerial.print("MP:"); vTaskDelay(500 / portTICK_PERIOD_MS);
        WebSerial.print(tempValue); vTaskDelay(500 / portTICK_PERIOD_MS);
        WebSerial.println("°C"); vTaskDelay(500 / portTICK_PERIOD_MS);

        vTaskDelay(1000 / portTICK_PERIOD_MS); // non-blocking delay (holds the task till 500ms)
      }
    }
  else  vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void SensingTask(void *parameter)
{ 
  tempValue=0; BPMValue=0; //global read only
  while(1){
    BPMValue=analogRead(AnalogPinBPM);
    tempValue=analogRead(AnalogPinTemp);
    vTaskDelay(2000 / portTICK_PERIOD_MS); // delay
  }

}

/******************************************************/

void setup() {
  Serial.begin(115200);
  pinMode(LED_GPIO, OUTPUT);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while(WiFi.status() != WL_CONNECTED){Serial.println("Wifi not connected yet!");};

  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  if(WiFi.status() == WL_CONNECTED){
  WebSerial.begin(&server);
  WebSerial.msgCallback(message);
  server.begin();
  Serial.println("Server began. ok.");
  }
  
  // Create mutex before starting tasks
  mutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(SensingTask,
                      "SensorTask",
                      5120,
                      NULL,
                      2,
                      NULL,
                      app_cpu);

/*************************************************************/
  xTaskCreatePinnedToCore(TaskBPM,
                      "ReadBPM",
                      5120,
                      NULL,
                      2,
                      NULL,
                      app_cpu);

  xTaskCreatePinnedToCore(TaskTEMP,
                          "Read Temperature and publish in server",
                          5120,
                          NULL,
                          2,
                          NULL,
                          app_cpu);  
/*************************************************************
  xTaskCreatePinnedToCore(TaskBPM_withoutMuTex,
                      "ReadBPM",
                      5120,
                      NULL,
                      2,
                      NULL,
                      app_cpu);

  xTaskCreatePinnedToCore(TaskTEMP_withoutMuTex,
                          "Read Temperature and publish in server",
                          5120,
                          NULL,
                          2,
                          NULL,
                          app_cpu);  
/*************************************************************/
}

char info[200]="This is basic patient Health information logging service. It will publish patient logs here. That will be sent to FTP server for database logging service.";

void loop() { //Priotity 1 task (lowest priority task)
  for(int i=0;i<200 && info[i]!='\0';i++)
  {
    Serial.print(info[i]);
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
    Serial.println(" ");
}
