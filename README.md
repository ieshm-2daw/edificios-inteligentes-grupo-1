**Lista de dispositivos**

**-Placa ESP32**
![placa ESP32](https://tienda.bricogeek.com/img/p/5/8/3/3/5833.jpg)
Se trata de un microcontrolador que integra tecnologías WiFi y Bluetooth, que le proporcionan conectividad con internet u otros dispositivos.



**Descarga y configuracion de Arduino IDE**
- Nos dirigimos a https://www.arduino.cc/en/software
- Descargamos la version linux ZIP
- Descomprimimos y ejecutamos el archivo "arduino-ide"
- Una vez abierto el IDE en la esquina superior izquirda pulsamos "Select other board and port"
- Buscamos y seleccionamos ESP32 Dev Module y a la derecha nos aparecera el puesto al que tenemos conectada la ESP32. Tras seleccionar le damos a "OK"



**Primeras pruebas de codigo**

El relé se enciende y se apaga con intervalo de un segundo

```cpp

#define SWITCH_BUILTIN 4 //definimos el pin donde esta conectado el rele

void setup() {
  pinMode(SWITCH_BUILTIN, OUTPUT); //se configura el pin
}

void loop() {
  digitalWrite(SWITCH_BUILTIN, HIGH); //se enciende el rele
  deplay(1000); // un segundo de espera
  digitalWrite(SWITCH_BUILTIN, LOW); //se apaga el rele
  deplay(1000); //un segundo de espera
}

```

Imprimir por consola el nivel de luminosidad captado por el sensor BH1750

```cpp

#include <BH1750.h>
#inlcude <Wire.h>

BH1750 sensor:

void setup() {
  Wire.begin();
  Serial.begin(9600);
}

void loop() {
  unsigned int lux = sensor.readLightLevel();
  Serial.print("Nivel: ");
  Serial.print(lux);
  Serial.println(" lx");
  deplay(1000);
}

```

Conectar ESP32 al WiFi

```cpp

#include "WiFi.h"
#define SECRET_SSID "2DAW_IoT"
#define SECRET_PSW "Somos2DAW"

const char* ssid = SECRET_SSID;
const char* password = SECRET_PSW;

void setup() {
 Serial.begin(115200);
 WiFi.begin(ssid, password);
 while (WiFi.status() != WL_CONNECTED) {
   delay(500);
   Serial.println("Conectando con la wifi...");
 }
 Serial.println("Conectado con el WiFi network");
}

void loop() {}

```

Codigo necesario para que se suscriba a mqtt

```cpp

#include <WiFi.h>
#include <PubSubClient.h>

// Configuración WiFi
const char* ssid = "2DAW_IoT";
const char* password = "Somos2DAW";

// Configuración MQTT
const char* mqtt_server = "192.168.100.101"; // Dirección del broker MQTT
const int mqtt_port = 1883;                   // Puerto del broker MQTT
const char* mqtt_topic = "g1/rele";     // Tema MQTT para control del relé

const char* mqtt_username = "mqtt"; // Sustituye con tu nombre de usuario MQTT
const char* mqtt_password = "mqtt"; // Sustituye con tu contraseña MQTT
// Pin del relé
const int relePin = 21;

WiFiClient espClient;
PubSubClient client(espClient);

// Función para conectar al WiFi
void setupWiFi() {
 delay(10);
 Serial.println("Conectando a WiFi...");
 WiFi.begin(ssid, password);
 while (WiFi.status() != WL_CONNECTED) {
   delay(500);
   Serial.print(".");
 }
 Serial.println("\nConectado a WiFi");
}

// Callback para gestionar los mensajes MQTT recibidos
void callback(char* topic, byte* payload, unsigned int length) {
 Serial.print("Mensaje recibido en el tema: ");
 Serial.println(topic);

 String message = "";
 for (int i = 0; i < length; i++) {
   message += (char)payload[i];
 }

 Serial.print("Contenido del mensaje: ");
 Serial.println(message);

 // Controlar el relé según el mensaje recibido
 if (message == "ON") {
   digitalWrite(relePin, HIGH); // Encender relé
   Serial.println("Relé ENCENDIDO");
 } else if (message == "OFF") {
   digitalWrite(relePin, LOW); // Apagar relé
   Serial.println("Relé APAGADO");
 }
}

// Función para conectar al broker MQTT
void reconnectMQTT() {
 while (!client.connected()) {
   Serial.print("Conectando al broker MQTT...");
   if (client.connect("ESP32Client", mqtt_username, mqtt_password)) {
     Serial.println("Conectado a MQTT");
     client.subscribe(mqtt_topic);
   } else {
     Serial.print("Falló la conexión. Código de error: ");
     Serial.print(client.state());
     Serial.println(". Intentando de nuevo en 5 segundos...");
     delay(5000);
   }
 }
}

void setup() {
 Serial.begin(115200);

 // Configuración del pin del relé como salida
 pinMode(relePin, OUTPUT);
 digitalWrite(relePin, LOW); // Inicialmente apagado

 // Conectar a WiFi
 setupWiFi();

 // Configurar el cliente MQTT
 client.setServer(mqtt_server, mqtt_port);
 client.setCallback(callback);
}

void loop() {
 if (!client.connected()) {
   reconnectMQTT();
 }
 client.loop();
}

```

Encender el rele segun la cantidad de luz detectada

```cpp

#include <BH1750.h>
#include <Wire.h>

 BH1750 sensor;

#define SWITCH_BUILTIN 13

void setup() {
 // put your setup code here, to run once:
 Wire.begin();
 sensor.begin();
 Serial.begin(115200);

 pinMode(SWITCH_BUILTIN, OUTPUT);  // SE CONFIGURA EL PIN
}

void loop() {
 // put your main code here, to run repeatedly:
 unsigned int lux = sensor.readLightLevel();
 Serial.print("Nivel: ");
 Serial.print(lux);
 Serial.println(" lx");
 delay(1000);

 if (lux > 10) {
   digitalWrite(SWITCH_BUILTIN, HIGH);
 }
 else if(lux < 8) {
   digitalWrite(SWITCH_BUILTIN, LOW);
 }
}

```
