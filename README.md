# 📊 Actividad 2 - Instrumentación

Este repositorio contiene el desarrollo de la **Actividad 2** de la asignatura de *Instrumentación* del Máster de la **UNIR** (Universidad Internacional de La Rioja).
El objetivo es documentar todo el proceso de diseño, implementación y prueba de un sistema de instrumentación utilizando hardware y simulación.

## 👨‍🎓 Integrantes del equipo

- Adrián García
- Carlos Martínez
- Inés Martínez
- Manuel Torres
- José Adrián Sabina
   
## 📑 Descripción del Readme

- [Introducción](#introducción)
- [Hardware](#hardware)
- [Firmware](#firmware)
- [Pruebas de simulación](#pruebas-de-simulación)


---

## 🧭 Introducción

En la actividad 2 de la asignatura Equipos e Instrumentación Electrónica vamos a implementar modulos que optimicen el control de las variables medidas en la actividad 1 (temperatura y luminosidad). 
Para el control optimizado de estos paramétros hemos implementado algoritmos, el algoritmo utilizado en el control de la temperatura es el algoritmo PID, muy usado en controles de temperatura del hogar, etc. 
Para el control de la luminosidad usaremos otro algoritmo para lograr la implantación de varios de estos algoritmos de control y no ser repetitivos. Usaremos el algoritmo de 3 posiciones con histéresis, según la luz exterior el LED que muestra la posición de la boya aumentará su brillo o disminuirá.

---

## 🔌 Hardware

En esta sección se incluirán los recursos relacionados con el hardware del proyecto:

- 📐 **Diagrama de conexiones**

AÑADIR IMAGEN DEL DIAGRAMA DE CONEXIONES CREADO CON KICAD
  
- 📋 **BOM (Bill of Materials)**: listado de todos los componentes utilizados

| Cant. | Componente                  | Descripción / Número de parte                             |
| ----- | --------------------------- | --------------------------------------------------------- |
| 1     | Arduino Uno R3              | Placa microcontroladora con ATmega328P                    |
| 1     | Pantalla LCD 16x2 con I2C   | Pantalla LCD de 2 líneas por 16 caracteres con módulo I2C |
| 1     | Sensor DHT22                | Sensor digital de temperatura y humedad                   |
| 1     | Sensor DS18B20              | Sensor digital de temperatura (OneWire)                   |
| 1     | Sensor ultrasónico HC-SR04  | Sensor de distancia por ultrasonido                       |
| 1     | LED RGB (Cátodo común)      | LED tricolor de 4 patas con cátodo común                  |
| 3     | Resistencias de 220Ω        | Para limitar corriente en cada color del LED RGB          |
| 1     | Zumbador pasivo             | Módulo buzzer sin oscilador interno                       |
| 1     | Sensor de luz (LDR)         | Sensor analógico de luz (fotoresistencia)                 |
| 1     | Potenciómetro deslizante    | Potenciómetro lineal tipo deslizable                      |
| 1     | Potenciómetro rotativo      | Potenciómetro giratorio estándar                          |
| 2     | Pulsadores                  | Botones tipo pulsador momentáneo                          |
| 2     | Resistencias de 10kΩ        | Resistencias pull-down para los botones                   |
| 1     | Servomotor SG90             | Servomotor pequeño de rotación controlada                 |
| 1     | Protoboard                  | Placa para prototipado sin soldadura                      |
| 20+   | Cables dupont (Macho-Macho) | Cables para conexiones entre componentes                  |
| 1     | Cable USB tipo B            | Cable para alimentar y programar el Arduino               |

---

## 💻 Firmware

A continuación, se documentará el código utilizado y se explicara su funcionamiento.

- 🧾 **Código comentado**<br>
El código principal ya fue comentado en la Actividad 1. A continuación, explicaremos el nuevo estado de funcionamiento que hemos añadido para el control de la temperatura y la luminosidad.
El primer estado es la implementación del algoritmo PID para el control de la temperatura. Para que quede claro, lo dividiremos en bloques:

   -  **Librerías**<br>
   ```cpp
   #include <dht.h>               // Lectura del sensor de temperatura y humedad DHT22
   #include <PID_v1.h>            // Algoritmo PID para control de temperatura
   ```
   -  **Declaración de variables**<br>
   ```cpp
   double input, output, setpoint = 20.0, simTemp;   
   // input: valor actual de temperatura que entra al PID
   // output: salida del PID (Para control de enfriar o calentar)
   // setpoint: Temperatura objetivo de funcionamiento óptimo de baterías
   // simTemp: Variable que recoge el valor del sensor DHT22, para simulacon
   
   PID pid(&input, &output, &setpoint, 2.0, 0.5, 1.5, DIRECT); 
   // Se crea el objeto PID con punteros a input, output y setpoint
   // y los parámetros KP=2.0, KI=0.5, KD=1.5 en modo DIRECTO
   
   bool flag = false; 
   // Protección ante rebotes del boton
   
   uint32_t btnTimer = 0; 
   // Temporizador para pulsaciones del boton
   
   int mode_PID = 0; 
   // Estados dentro del modo algoritmo PID para temperatura
   
   int count = 0;    
   unsigned long lastTime;
   ```
   -  **Función SETUP**<br>
   Esta función se ejecuta una única vez, en ella se incializa la consola para mostrar mensajes (11500 baudios), la pantalla LCD, se indica el direccionamiento de los pines y se hace una lectura del sensor para comprobar que funciona correctamente. Respecto al algoritmo PID, en esta función establecemos los límites de la salida y el modo de funcionamiento en AUTOMATIC.
   ```cpp
      void setup() {
      Serial.begin(11500);
      lcd.init(); 
      lcd.backlight();
      
      pinMode(BUTTON_PIN, INPUT_PULLUP);
      pinMode(HEAT_PIN, OUTPUT);
      pinMode(COOL_PIN, OUTPUT);
      
      pid.SetMode(AUTOMATIC);
      pid.SetOutputLimits(-255, 255);
      
      int chk = DHT.read22(DHTPIN);
      if (chk == DHTLIB_OK) {
      simTemp = DHT.temperature;
      }
      lastTime = millis();
      }
   ```
Dentro de la función loop(), en nuevo estado añadido para el control de la temperatura con el algoritmo PID encontramos el código adjunto. Se divide en dos estados internos, uno en el que mediante la lectura del sensor DHT22 se almacena el valor de la temperatura en una variable para posteriormente en el modo interno dos se ejecute el algoritmo PID, en función de su valor activamos con diferentes combinaciones de colores un LED RGB, si es ROJO (Resistencias térmicas ON), VERDE (Temperatura en el rango óptimo) y AZUL (Ventilador de enfriamiento ON).

Para mayor detalle de cada línea de código, leer comentarios.
   ```cpp
      // --- MODOS INTERNOS ALGORITMO PID ---
  if (mode_PID == 0) {
    int chk = DHT.read22(DHTPIN);
    if (chk == DHTLIB_OK) {
      simTemp = DHT.temperature; // Lectura del sensor DHT22
    }
   //Pantalla LCD, limpiza y visualización
    lcd.setCursor(0, 0); 
    lcd.print("TEMP AMB        ");
    lcd.setCursor(0, 1); 
    lcd.print(simTemp, 1); 
    lcd.print(" C        ");
    analogWrite(HEAT_PIN, 0);
    analogWrite(COOL_PIN, 0);
  } else {
    input = simTemp;
    pid.Compute(); //Calcula el output del algoritmo PID
/*La siguiente función imita una correción del algoritmo PID pero de forma simulada, al principio normalizamos la salida a un valor de ±1, se multiplica por 5 para hacer que los cambios por cada interacción del bucle sean de 5ºC. Luego se imita un aumento o decremento de la temperatura proporcional a 0.1ºC. */
    simTemp += (output / 255.0) * 5.0 - 0.1 * (simTemp - setpoint);

    // Calcula si estamos alejados del rango 20 °C (±1 °C) 
    double delta = simTemp - setpoint;  
    if (delta > 1.0) {
      // Temperatura muy alta -> Azul
      analogWrite(red_led, 0);
      analogWrite(green_led, 0);
      analogWrite(blue_led, 255);
    } else if (delta < -1.0) {
      // Temperatura muy baja -> Rojo
      analogWrite(red_led, 255);
      analogWrite(green_led, 0);
      analogWrite(blue_led, 0);
    } else {
      // Temperatura cercana -> Verde
      analogWrite(red_led, 0);
      analogWrite(green_led, 255);
      analogWrite(blue_led, 0);
    }
    lcd.setCursor(0, 0); 
    lcd.print("MODO PID        ");
    lcd.setCursor(0, 1); 
    lcd.print(simTemp, 2); 
    lcd.print(" C ->20C ");

  delay(50);
}
   ```
---

## 🧪 Pruebas de simulación

Simulaciones realizadas para verificar el comportamiento del sistema:

- 🔍 **Pruebas del funcionamiento en WOKWI**
  
IMAGENES DE WOKWI FUNCIONANDO  

AQUI VA EL CASO 1 DE LOS .gif, SE ADJUNTA ARRASTRANDO EL ARCHIVO DESDE EL EXPLORADOR DE WINDOWS HASTA AQUI. ASI SE CARGA. NO ME  DEJA PORQUE INDRA LO TIENE BLOQUEADO PROBAR CON OTRO ORDENADOR.

El siguiente ejemplo ilustra cómo el algoritmo PID hace que la temperatura descienda y se ajuste al valor que queremos de temperatura en las baterías, siendo este de 20 °C. A medida que se aproxima a 20ºC se observa como la "correción" es más lenta y no van tán rápido.

El video es un fragmento donde se observan dos elementos clave: el primero es la pantalla LCD, que a diferencia del primer video cambia su interfaz y muestra cómo se autoajusta la temperatura gracias al algoritmo PID; y el LED RGB, que según la temperatura que haya en ese momento, cumple una función u otra. Es decir, si el LED está en AZUL, la temperatura está por encima de 20 °C (±1 °C) y se activa el ventilador; si está en el rango de 20 °C (±1 °C), se pondrá en VERDE; y si está por debajo, se activan las resistencias y se pone en ROJO.

![CASO2_PID](https://github.com/user-attachments/assets/42fd210c-40b3-461a-b503-dda0467b948f)

---
