# CodigoElectronica

$E = \frac{1}{2}mv^2$

$$G=(N,\Sigma,P,S)$$
$$N={S,A,B,X}$$
$$\Sigma={a,b}\\newline$$
$$P= S \to  ABX\\newline$$
$$A \to  aaA | aa\\newline$$
$$B \to bbB | b$$

$$N=(Q,\Sigma,\Gamma,\delta, S,F)$$
$$Q={q}$$
$$\Sigma={a,b\right\}$$
$$\Gamma ={a,b}$$
$$S={q}$$
$$F=\phi$$

#include "Ubidots.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#define Trig_sensor_prox 2
#define out_sensor_prox 4
#define sda_giro 19
#define scl_giro 21
#define sda_lcd 22
#define scl_lcd 23
#include <LiquidCrystal_I2C.h>

// Configura la dirección I2C y el tamaño del LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);  // Cambia 0x27 si tu LCD tiene otra dirección

Adafruit_MPU6050 mpu;
bool estado = false;  // Indica si la música está reproduciendo
int volumen = 50;
bool objetoDetectado = false;                                                   // Indica si el objeto está presente (mano cerca)
bool objetoPrevio = false;                                                      // Indica el estado anterior del objeto (para detectar cambios)
const char* UBIDOTS_TOKEN = "BBUS-GPG07cdhcf3HNVryv3U1PEI1bJPDpn";              // Put here your Ubidots TOKEN
const char* WIFI_SSID = "IoT-B19";                                              // Red Wi-Fi (SSID)
const char* WIFI_PASS = "lcontrol2020*";                                        // Password Wi-Fi                                      // Put here your Wi-Fi password
const char* DEVICE_LABEL_TO_RETRIEVE_VALUES_FROM = "6717b808e2bd832efe6b0d7c";  // Replace with your device label
const char* VARIABLE_LABEL_TO_RETRIEVE_VALUES_FROM_1 = "estado";                // Replace with your variable label
const char* VARIABLE_LABEL_TO_RETRIEVE_VALUES_FROM_2 = "volumen";               // Replace with your variable label
const char* VARIABLE_LABEL_TO_RETRIEVE_VALUES_FROM_3 = "inclinacion";           // Replace with your variable label

// Create a pointer to a instance of the Ubidots class to use it globally
Ubidots* ubidots{ nullptr };

/**
 * Auxiliar Functions
 **/

// Put here your auxiliar functions

/**
 * Main Functions
 **/

void setup() {
  Serial.begin(115200);
  Ubidots::wifiConnect(WIFI_SSID, WIFI_PASS);
  ubidots = new Ubidots(UBIDOTS_TOKEN, UBI_HTTP);
  pinMode(Trig_sensor_prox, OUTPUT);
  pinMode(out_sensor_prox, INPUT);

  // Inicia la comunicación I2C y el LCD
  Wire.begin();
  lcd.begin(16,2);
  lcd.backlight(); // Enciende la luz de fondo

  // Muestra un mensaje en la pantalla
  lcd.setCursor(0, 0);
  lcd.print("hola ");

  Serial.println("Adafruit MPU6050 test!");

  // Inicializa el MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.println("");
  delay(100);
}

void mostrarPantalla(String mensaje) {
  lcd.print(mensaje);
}

void parar() {  // Función para detener la música
  estado = false;
  mostrarPantalla("Ⅱ");  // Mostrar el símbolo de pausa
}

void reproducir() {  // Función para seguir reproduciendo la música
  estado = true;
  mostrarPantalla("►");  // Mostrar el símbolo de reproducción
}

bool detectarObjeto() {
  // Limpia el pin TRIG
  digitalWrite(Trig_sensor_prox, LOW);
  delayMicroseconds(2);

  // Envía un pulso de 10 microsegundos al pin TRIG
  digitalWrite(Trig_sensor_prox, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig_sensor_prox, LOW);

  long duracion = pulseIn(out_sensor_prox, HIGH);
  long distance = (duracion * 340) / 10000 / 2;

  // Detectar si el objeto está dentro de la distancia (< 6 cm)
  if (distance < 6) {
    return true;
  } else {
    return false;
  }
}

void loop() {
  lcd.setCursor(0, 0);
  lcd.print("Hola");
  /* Obtain last value from a variable as float using HTTP */
  // estado = ubidots->get(DEVICE_LABEL_TO_RETRIEVE_VALUES_FROM, VARIABLE_LABEL_TO_RETRIEVE_VALUES_FROM_1);
  // volumen = ubidots->get(DEVICE_LABEL_TO_RETRIEVE_VALUES_FROM, VARIABLE_LABEL_TO_RETRIEVE_VALUES_FROM_2);
  // inclinacion = ubidots->get(DEVICE_LABEL_TO_RETRIEVE_VALUES_FROM, VARIABLE_LABEL_TO_RETRIEVE_VALUES_FROM_3);
  // // Agregar variables a Ubidots
  // ubidots->add(VARIABLE_LABEL_TO_RETRIEVE_VALUES_FROM_1, estado);
  // ubidots->add(VARIABLE_LABEL_TO_RETRIEVE_VALUES_FROM_2, volumen);
  // ubidots->add(VARIABLE_LABEL_TO_RETRIEVE_VALUES_FROM_3, inclinacion);
  // Enviar datos a Ubidots
  // ubidots->send(DEVICE_LABEL_TO_RETRIEVE_VALUES_FROM);
  // Leemos el valor analógico del sensor LM35, convertimos ese valor a voltaje.
  bool objetoActual = detectarObjeto();  // Detectar si la mano está sobre el sensor

  // Detectar si hubo un cambio de estado en la detección del objeto (paso de la mano)
  if (objetoActual != objetoPrevio && objetoActual == true) {
    // Si hay un cambio y ahora se detecta un objeto, alterna el estado
    if (estado) {
      parar();  // Si está reproduciendo, pausar
    } else {
      reproducir();  // Si está en pausa, reproducir
    }
  }

  // Actualizar el estado previo del objeto
  objetoPrevio = objetoActual;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.println("");
  delay(500);
}


// int detectarInclinacionX(){ //Función para detectar alguna inclinación en X (Derecha e Izquierda)
//   //Código del giroscopio para detectar las inclinaciones en X
//   if(true){ //Suponiendo una inclinación positiva, es decir, hacia la derecha, que devuelva el valor
//     return 1;
//   } else { //Suponiendo que la inclinación es negativa, es decir, hacia la izquierda, que devuelva el valor
//     return -1;
//   }
// }


// int detectarInclinacionY(){ //Función para detectar alguna inclinación en Y (Arriba y abajo)
//   //Código del giroscopio para detectar las inclinaciones en Y
//   if(true){ //Suponiendo una inclinación positiva, es decir, hacia arriba, que devuelva el valor
//     return 1;
//   } else { //Suponiendo que la inclinación es negativa, es decir, hacia abajo, que devuelva el valor
//     return -1;
//   }
// }
//   if(detectarInclinacionY()>0){ //Todo el rato confirma la inclinación en Y para ver si tenemos que subir o bajar el volumen
//     subirVolumen();
//     mostrarPantalla("+");
//   } else if (detectarInclinacionY()<0){
//     bajarVolumen();
//     mostrarPantalla("-");
//   }
//     if(detectarInclinacionY()>0){ //Todo el rato confirma la inclinación en X para ver si tenemos que pasar la canción actual
//     subirVolumen();
//     mostrarPantalla("≫");
//   } else if (detectarInclinacionY()<0){
//     bajarVolumen();
//     mostrarPantalla("≪");
//   }
// }
