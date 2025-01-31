**Objetivo en el proyecto**
Nuestro objetivo es utilizar dos ESP32 con un sensor de luminosidad en cada uno, estas placas se colocaran en dos ubicaciones de la clase para medir el nivel de luz en cada punto, estas dos placas harán un promedio para decidir si encender las luces o mantenerlas apagadas, Ejp: Si el promedio de ambas placas supera los 100 lux la luz se mantendrá apagada, en cambio si es menor a 100lux se encenderan las luces, todo esto se medirá cada 60 segundos, esto ayudará a que haya una visión clara en el aula.

**¿Como lo vamos a hacer?**
Tendremos dos ESP32 y cada uno tendrá un medidor de lúmenes cada uno, el primer sensor (que será el proincipal) mostrará la cantidad de lúmenes que recibe su sensor, y estará escuchando conectada mediante MQTT al sensor secundario, esperando a recibir datos para mostrarlos con los datos del primer sensor. El segundo sensor recibirá los lúmenes y los enviará a la ESP32 principal mediante MQTT.

Para que se haga más efectivo, mediremos el promedio de ambos sensores y si la cantidad de lúmenes supera una cierta cantidad, encendemos la luz para una mejor vision, para que esto funcione conectaremos un relé a la ESP32 principal.

**Lista de dispositivos en el kit**

**-Placa ESP32**

![placa ESP32](https://tienda.bricogeek.com/img/p/5/8/3/3/5833.jpg)

Se trata de un microcontrolador que integra tecnologías WiFi y Bluetooth, que le proporcionan conectividad con internet u otros dispositivos.


**-Placa desarrollo para ESP32**

![Placa de desarrollo](https://es.led-diode.com/Content/uploads/2022459623/2022121417582118d90af43cf848ab978cb6138d9a2ad9.jpg)

Es una tarjeta de desarrollo WIFI/Bluetooth, tiene las mismas características que otras placas ESP32 y se puede programar con microPython, LUA y Arduino IDE. Se puede alimentar mediante MicroUSB.


**-Cable alimentación USB-C USB**

![Cable de alimentacion](https://portdesigns.com/img/cms/Produits/ALIM%2065W%20USB-C%20900097B-UK/900097B-UK%20-%20PORT%20-%20UNIVERSAL%20POWER%20SUPPLY%20BUDGET%2065W%20-%20TOP%20PICTO.jpg)

Se emplea para transmitir energía o datos desde un punto a otro.


**-Sensor de gas CO2 TVOC CCS811**

![Sensor de gas CO2](https://m.media-amazon.com/images/I/51qY+fz1otL.jpg)

Mide los valores de CO2 durante largos periodos de tiempo, mediante la absorción de una luz infrarroja en una longitud de onda específica.


**-Sensor de iluminación BH1750**

![Sensor iluminacion](https://naylampmechatronics.com/170-home_default/modulo-sensor-de-luz-digital-bh1750.jpg)

El sensor de iluminación es un sensor digital de luz ambiental que mide la intensidad lumínica en lux.


**-Sensor PIR**

![Sensor PIR](https://www.prometec.net/wp-content/uploads/2015/09/HC-SR501.jpg)

El sensor pir es un dispositivo electrónico que se utiliza para detectar la presencia o ausencia en un área. 


**-Módulo RFID MRC522**

![Módulo rfid mrc522](https://www.turibot.es/images/thumbs/0012259_modulo-rfid-rc522-de-135mhz-con-tarjeta-y-etiqueta_600.jpeg)

La tecnología rfid es un sistema de identificación de productos por radiofrecuencia


**-Sensor de corriente SCT-013 30A**

![Sensor de corriente sct-013 30A](https://encrypted-tbn0.gstatic.com/images?q=tbn:ANd9GcSF8soM0R9scS4V-dF_OxEngltQ4lQzdAc-ug&s)

Los sensores de corriente detectan el campo magnético producido cuando una corriente circula a través del cable a medir. 


**-Sensor de sonido JY-038**

![Sensor de sonido jy-038](https://m.media-amazon.com/images/I/71iptWHmWpL.jpg)

El sensor de sonido transforma la intensidad del sonido en el ambiente, en señales eléctricas.


**-Sensor mov. microondas RCWll0516**

![Sensor mov. microondas rcwll0516](https://encrypted-tbn0.gstatic.com/images?q=tbn:ANd9GcRw1SPznC8rUSBemvbBsKqahQO3irkfm2pYWw&s)

El sensor mov. microondas emite señales de microondas continuas, que rebotan en los objetos en su área de detección y regresan al sensor , cuando las señales reflejadas tienen la misma frecuencia que las de salida no hay movimiento.


**-Módulo amplificación señal LM358**

![Módulo amplificación señal lm358](https://m.media-amazon.com/images/I/71uNzm6W-1L.jpg)

El funcionamiento del amplificador de señal es amplificar la señal del dispositivo de entrada para facilitar el procesamiento o detección por otros dispositivos electrónicos como los microcontroladores.


**-Cerradura solenoide electromagnética**

![Cerradura solenoide electromagnética](https://m.media-amazon.com/images/I/612l4SkFGgL.jpg)

Es un electroimán que consta de una bobina de alambre de cobre con una armadura (un lingote de metal) en el medio 


**-Módulo de relé optoacoplador 3V**

![Módulo de relé optoacoplador 3V](https://m.media-amazon.com/images/I/51KshDr22-L._AC_UF894,1000_QL80_.jpg)

Este módulo sirve para controlar todo tipo de electrodomésticos y dispositivos de carga de alta corriente como ventiladores funcionando como un interruptor activado mediante la luz emitida por un diodo red.


**-Protoboard 830 Puntos C/MB102**

![Protoboard 830 Puntos C/MB102](https://digitalcodesign.com/web/image/product.template/3442/image_1024?unique=1908a9f)

Es una herramienta simple que se usa en proyectos de robótica que permite conectar fácilmente componentes electrónicos entre sí, sin necesidad de realizar una soldadura


**-Pantalla Oled 0.96” I2C**

![Pantalla Oled 0.96” I2C](https://encrypted-tbn0.gstatic.com/images?q=tbn:ANd9GcR4VvnzCsjT4Vv3OweKAqwwUkEc0Is_SXKjDA&s)

Dispositivos electrónicos tipo led, permiten controlar cada píxel individualmente y mostrar tanto texto como gráficos por medio de comunicación I2C


**-Raspberry PI 5 4GB**

![Raspberry PI 5 4GB](https://www.kubii.com/14207-full_default/raspberry-pi-5.jpg)

Es un ordenador de placa única (SBC) de bajo costo y es una herramienta útil para la enseñanza y el desarrollo de proyectos de electrónica o programación


**-Carcasa con disipador raspberry PI 5**

![Carcasa con disipador raspberry PI 5](https://www.tiendatec.es/9675-home_default/kksb-caja-aluminio-para-raspberry-pi-5.jpg)

Esta carcasa trae un disipador el cual nos sirve para extraer el calor generado por el procesador


**-Fuente de alimentación 5.5V/5.1A USB-C**

![Fuente de alimentación 5.5V/5.1A USB-C](https://www.tiendatec.es/9826-large_default/fuente-alimentacion-usb-c-pd-gan-5-1v-5a-27w.jpg)

Dispositivo que se utiliza para convertir la corriente De la red eléctrica en una forma de energía adecuada para los componentes de un dispositivo o sistema electrónico


**-Tarjeta micro SD 32GB**

![Tarjeta microsd 32GB](https://static.fnac-static.com/multimedia/Images/ES/NR/5c/32/06/406108/1540-4/tsp20160812185213/Sandisk-MicroSD-32-GB.jpg)

Son dispositivos que nos permiten expandir el sistema de almacenamiento del dispositivo



**Descarga y configuracion de Arduino IDE**
- Nos dirigimos a https://www.arduino.cc/en/software
- Descargamos la version linux ZIP
- Descomprimimos y ejecutamos el archivo "arduino-ide"
- Una vez abierto el IDE en la esquina superior izquirda pulsamos "Select other board and port"
- Instalamos la extension esp32
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


**Automatización de la iluminación de clase**
Para automatizar la iluminacion utilizaremos 2 sensores de luz conectado a su propia ESP32 en diferentes zonas de la clase para una mayor efectividad.

La ESP32 principal recibira los datos de luz de su sensor y de un sensor secundario y dependiendo del nivel de luminosidad de las distintas zonas encendera o apagara el rele.

```ccp
#include <WiFi.h>
#include <PubSubClient.h>

//Definimos todo para conectar con MQTT

#define WIFI_SSID "2DAW_IoT"
#define WIFI_PASSWORD "Somos2DAW"
#define MQTT_SERVER "192.168.100.101"
#define MQTT_PORT 1883
#define MQTT_USER "mqtt"
#define MQTT_PASSWORD "mqtt"
#define MQTT_TOPIC "g1/rele"

//Definimos el relé
#define SWITCH_BUILTIN 13


//definimos sensor principal
#include <BH1750.h>
#include <Wire.h>
BH1750 sensor;


WiFiClient espClient;
PubSubClient client(espClient);

//El callback lo utilizaremos para recibir por MQTT los datos del sensor de la placa 2
void callback(char* topic, byte* payload, unsigned int length) {
 Serial.print("Mensaje recibido en el topic: ");
 Serial.println(topic);


 String luxStr = "";
 for (int i = 0; i < length; i++) {
   luxStr += (char)payload[i];
 }
 Serial.print("Nivel de luz placa 2: ");
 Serial.println(luxStr);


 int luxSecundaria = luxStr.toInt();
 if (luxSecundaria > 10) {
   digitalWrite(SWITCH_BUILTIN, HIGH);
 } else if (luxSecundaria < 8) {
   digitalWrite(SWITCH_BUILTIN, LOW);
 }
}

//El reconnect se utiliza para verificar si estamos conectados a MQTT y si no lo esta o da fallo lo notifique
void reconnect() {
 while (!client.connected()) {
   Serial.print("Conectando a MQTT...");
   if (client.connect("ESP32Subscriber", MQTT_USER, MQTT_PASSWORD)) {
     Serial.println("Conectado a MQTT");
     client.subscribe(MQTT_TOPIC);
   } else {
     Serial.print("Error, rc=");
     Serial.print(client.state());
     Serial.println(" Intentando de nuevo en 2 segundos...");
     delay(2000);
   }
 }
}


void setup() {
  //definimos el relé
 pinMode(SWITCH_BUILTIN, OUTPUT);


 Serial.begin(115200);

//Utilizamos el sensor
 Wire.begin();
 sensor.begin();


 // Conexión WiFi
 Serial.print("Conectando a WiFi");
 WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
 while (WiFi.status() != WL_CONNECTED) {
   delay(1000);
   Serial.print(".");
 }
 Serial.println("\nConectado a WiFi");


 // Configuración MQTT
 client.setServer(MQTT_SERVER, MQTT_PORT);
 client.setCallback(callback);


 reconnect();
}


void loop() {
 // Leer el nivel de luz de la placa principal
 float lux = sensor.readLightLevel();
 Serial.print("Nivel de luz placa principal: ");
 Serial.println(lux);
 delay(2000);


 if (!client.connected()) {
   reconnect();
 }
 client.loop();
}

```

La ESP32 secundaria recibira los datos de luz de su sensor y los enviara a la ESP32 principal que controla el rele
```ccp
#include <WiFi.h>
#include <PubSubClient.h>
#include <BH1750.h>
#include <Wire.h>


#define WIFI_SSID "2DAW_IoT"
#define WIFI_PASSWORD "Somos2DAW"
#define MQTT_SERVER "192.168.100.101"
#define MQTT_PORT 1883
#define MQTT_USER "mqtt"
#define MQTT_PASSWORD "mqtt"
#define MQTT_TOPIC "g1/rele"


BH1750 sensor;
WiFiClient espClient;
PubSubClient client(espClient);


void reconnect() {
 while (!client.connected()) {
   Serial.print("Conectando a MQTT...");
   if (client.connect("ESP32_Secundaria", MQTT_USER, MQTT_PASSWORD)) {
     Serial.println("Conectado a MQTT");
   } else {
     Serial.print("Error, rc=");
     Serial.print(client.state());
     Serial.println(" Intentando de nuevo en 2 segundos...");
     delay(2000);
   }
 }
}


void setup() {
 Serial.begin(115200);
 Wire.begin();
  if (!sensor.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
   Serial.println("Error al iniciar el sensor BH1750");
   while (1);  // Si falla, detener el programa
 }


 // Conectar WiFi
 WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
 while (WiFi.status() != WL_CONNECTED) {
   delay(1000);
   Serial.print(".");
 }
 Serial.println("\nConectado a WiFi");


 // Configurar MQTT
 client.setServer(MQTT_SERVER, MQTT_PORT);
 reconnect();
}


void loop() {
 if (!client.connected()) {
   reconnect();
 }
 client.loop();


 // Leer el nivel de luzclient.publish(MQTT_TOPIC, luxStr);
 float lux = sensor.readLightLevel();
 Serial.print("Nivel de luz placa Secundaria: ");
 Serial.println(lux);


 // Convertir el valor a string antes de enviarlo por MQTT
 char luxStr[10];
 dtostrf(lux, 6, 2, luxStr);
 client.publish(MQTT_TOPIC, luxStr);


 delay(2000);
}

```
