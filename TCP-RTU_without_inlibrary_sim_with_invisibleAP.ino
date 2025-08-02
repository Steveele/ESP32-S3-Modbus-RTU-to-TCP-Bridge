/*
  ModbusRTU ESP8266/ESP32
  ModbusTCP to ModbusRTU bridge 
*/
#ifdef ESP8266
 #include <ESP8266WiFi.h>
#else //ESP32
 #include <WiFi.h>
#endif
#include <ModbusTCP.h>
#include <ModbusRTU.h>
//#include <SoftwareSerial.h>
//SoftwareSerial S(13, 15);


constexpr uint8_t PIN_RX    = 9;   // RO → ESP
constexpr uint8_t PIN_TX    = 8;   // DI ← ESP
constexpr uint8_t PIN_DE_RE = 7;    // DE+RE

/*****  Wi‑Fi: Soft‑AP  *****/
const char AP_SSID[]     = "ModbusLink";   // choose any name
const char AP_PASS[]     = "Bridge123";    // WPA2‑PSK
const uint8_t CHANNEL    = 6;              // pick a quiet 2.4 GHz channel
const bool    HIDDEN     = true; 

ModbusRTU rtu;
ModbusTCP tcp;

IPAddress srcIp;


uint16_t transRunning = 0;  // Currently executed ModbusTCP transaction
uint8_t slaveRunning = 0;   // Current request slave
 
bool cbTcpTrans(Modbus::ResultCode event, uint16_t transactionId, void* data) { // Modbus Transaction callback
  if (event != Modbus::EX_SUCCESS)                  // If transaction got an error
    Serial.printf("Modbus result: %02X, Mem: %d\n", event, ESP.getFreeHeap());  // Display Modbus error code (222527)
  if (event == Modbus::EX_TIMEOUT) {    // If Transaction timeout took place
    tcp.disconnect(tcp.eventSource());          // Close connection
    transRunning = 0;
    slaveRunning = 0;
  }
  return true;
}

bool cbRtuTrans(Modbus::ResultCode event, uint16_t transactionId, void* data) {
    if (event != Modbus::EX_SUCCESS)                  // If transaction got an error
      Serial.printf("Modbus result: %02X, Mem: %d\n", event, ESP.getFreeHeap());  // Display Modbus error code (222527)
    return true;
}


// Callback receives raw data 
Modbus::ResultCode cbTcpRaw(uint8_t* data, uint8_t len, void* custom) {
  auto src = (Modbus::frame_arg_t*) custom;
  
  Serial.print("TCP IP in - ");
  Serial.print(IPAddress(src->ipaddr));
  Serial.printf(" Fn: %02X, len: %d \n\r", data[0], len);

  if (transRunning) { // Note that we can't process new requests from TCP-side while waiting for responce from RTU-side.
    tcp.setTransactionId(src->transactionId); // Set transaction id as per incoming request
    tcp.errorResponce(IPAddress(src->ipaddr), (Modbus::FunctionCode)data[0], Modbus::EX_SLAVE_DEVICE_BUSY);
    return Modbus::EX_SLAVE_DEVICE_BUSY;
  }
Serial.println("→ sending RTU frame to UART");
  rtu.rawRequest(src->unitId, data, len, cbRtuTrans);
  
  if (!src->unitId) { // If broadcast request (no responce from slave is expected)
    tcp.setTransactionId(src->transactionId); // Set transaction id as per incoming request
    tcp.errorResponce(IPAddress(src->ipaddr), (Modbus::FunctionCode)data[0], Modbus::EX_ACKNOWLEDGE);

    transRunning = 0;
    slaveRunning = 0;
    return Modbus::EX_ACKNOWLEDGE;
  }
  
  srcIp = IPAddress(src->ipaddr);
  
  slaveRunning = src->unitId;
  
  transRunning = src->transactionId;
  
  return Modbus::EX_SUCCESS;  
  
}


// Callback receives raw data from ModbusTCP and sends it on behalf of slave (slaveRunning) to master
Modbus::ResultCode cbRtuRaw(uint8_t* data, uint8_t len, void* custom) {
  auto src = (Modbus::frame_arg_t*) custom;
  if (!transRunning) // Unexpected incoming data
      return Modbus::EX_PASSTHROUGH;
  tcp.setTransactionId(transRunning); // Set transaction id as per incoming request
  uint16_t succeed = tcp.rawResponce(srcIp, data, len, slaveRunning);
  if (!succeed){
    Serial.print("TCP IP out - failed");
  }
  Serial.printf("RTU Slave: %d, Fn: %02X, len: %d, ", src->slaveId, data[0], len);
  Serial.print("Response TCP IP: ");
  Serial.println(srcIp);
  Serial.println("← reply received from UART");

  
  transRunning = 0;
  slaveRunning = 0;
  return Modbus::EX_PASSTHROUGH;
}


void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_AP);
WiFi.softAP(AP_SSID, AP_PASS, CHANNEL, HIDDEN);
/* give the AP a fixed IP so ESP‑A can find it */
IPAddress apIP(192,168,10,1);
IPAddress net(255,255,255,0);
WiFi.softAPConfig(apIP, apIP, net);
Serial.println();
Serial.print("Soft‑AP up, IP = ");
Serial.println(WiFi.softAPIP());     // should print 192.168.10.1
    
  tcp.server(); // Initialize ModbusTCP to pracess as server
  tcp.onRaw(cbTcpRaw); // Assign raw data processing callback
  
  Serial1.begin(9600, SERIAL_8N1, PIN_RX, PIN_TX);
  rtu.begin(&Serial1, PIN_DE_RE);   //   <-- direction pin declared!
  rtu.master();
  rtu.onRaw(cbRtuRaw); // Assign raw data processing callback
}

void loop() { 
  
  rtu.task();
  tcp.task();
  yield();
}
