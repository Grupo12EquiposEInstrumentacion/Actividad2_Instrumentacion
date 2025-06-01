# 📊 Actividad 2 - Instrumentación

Este repositorio contiene el desarrollo de la **Actividad 2** de la asignatura de *Equipos e Instrumentación Electrónica* del Máster de la **UNIR** (Universidad Internacional de La Rioja).
El objetivo es documentar todo el proceso de diseño, implementación y prueba de un sistema de instrumentación utilizando hardware y simulación.

## 👨‍🎓 Integrantes del equipo

- Adrián García Alonso 
- Carlos Martínez Rodríguez
- Inés Martínez Romero
- Manuel Torres Pineda 
- José Adrián Sabina Pamo
   
## 📑 Descripción del Readme

- [Introducción](#introducción)
- [Hardware](#hardware)
- [Firmware](#firmware)
- [Pruebas de simulación](#pruebas-de-simulación)


---

## 🧭 Introducción

En la actividad 2 de la asignatura Equipos e Instrumentación Electrónica vamos a implementar modulos que optimicen el control de las variables medidas en la actividad 1, exactamente sobre las variables de luminosidad y temperatura. 
Para el control optimizado de estos paramétros hemos implementado algoritmos, el algoritmo utilizado en el control de la temperatura es el algoritmo PID, muy usado en controles de temperatura del hogar, etc. 
Para el control de la luminosidad hemos implementado un servo y una cadena de LEDs, estos elementos quieren imitar la realidad pero de forma simulada en WOKWI, el servo hace referencia al motor de una placa solar que sigue la dirección del sol desde que amanece hasta que anochece, según como se encuentre la luz estará en una posición u otra, luego la cadena de LEDs hacen referencia a la luz que puede tener la boya integrada que según vaya cayendo la noche más se ilumina.

---

## 🔌 Hardware

En esta sección se incluirán los recursos relacionados con el hardware del proyecto:

- 📐 **Diagrama de conexiones**

El sistema completo HARDWARE se ve como en la imagen adjunta:
![Captura de pantalla 2025-05-31 171320](https://github.com/user-attachments/assets/40dfb17d-a249-4767-ade1-f7b67af29273)

  
- 📋 **BOM (Bill of Materials)**: listado de todos los componentes utilizados

| Cant. | Componente                         | Descripción / Número de parte                               |
| ----- | ---------------------------------- | ----------------------------------------------------------- |
| 1     | Arduino Uno                        | Placa microcontroladora                                     |
| 1     | Pantalla LCD 20x4 con I2C          | Pantalla LCD de 4 líneas por 20 caracteres con interfaz I2C |
| 1     | Sensor DHT22                       | Sensor digital de temperatura y humedad                     |
| 1     | Sensor ultrasónico HC-SR04         | Sensor de distancia por ultrasonido                         |
| 1     | LED RGB (Cátodo común)             | LED tricolor de 4 patas con cátodo común                    |
| 1     | Zumbador pasivo                    | Módulo buzzer sin oscilador interno                         |
| 2     | Sensor de luz (LDR)                | Sensor analógico de luz (fotoresistencia)                   |
| 1     | Potenciómetro deslizante           | Potenciómetro lineal tipo deslizable                        |
| 1     | Potenciómetro rotativo             | Potenciómetro giratorio estándar                            |
| 2     | Pulsadores                         | Botones tipo pulsador momentáneo                            |
| 1     | Servomotor SG90                    | Servomotor pequeño de rotación controlada                   |
| 1     | Registro de desplazamiento 74HC595 | Circuito integrado para expansión de salidas                |
| 8     | LEDs blancos                       | Diodos emisores de luz color blanco                         |
| 8     | Resistencias de 220Ω               | Limitadores de corriente para los LEDs                      |
| 1     | Protoboard mini                    | Placa para prototipado sin soldadura (versión pequeña)      |
| 20+   | Cables Macho-Macho                 | Cables para conexiones entre componentes                    |
| 1     | Cable USB                          | Cable para alimentar y programar el Arduino                 |

---

## 💻 Firmware

A continuación, se documentará el código utilizado y se explicara su funcionamiento.

- 🧾 **Código comentado**<br>
El código principal ya fue comentado en la Actividad 1, a pesar de haber reutilizado las partes del código que realizaban la medición del viento, oleaje y activación de las resistencias térmicas, lo hemos organizado mucho mejor para un código más limpio y escalable. Hemos implementado los estados de cada modo de funcionamiento mediante un Switch Case que llama a la función y ejecuta su código. Esto trae mejoras en cuanto a la correción de errores, porque es mucho más localizable donde esta cada fallo.
A continuación, explicaremos el nuevo estado de funcionamiento que hemos añadido para el control de la temperatura y la luminosidad.
El primer estado es la implementación del algoritmo PID para el control de la temperatura. Para que quede claro, lo dividiremos en bloques:

**Cabe recalcar que para no hacer la documentación muy extensa hablaremos únicamente de las nuevas funciones. Y las declaraciones de variables importantes.**

**ALGORITMO PID PARA TEMPERATURA**

   -  **Librerías**<br>
   ```cpp
   #include <dht.h>               // Lectura del sensor de temperatura y humedad DHT22
   #include <PID_v1.h>            // Algoritmo PID para control de temperatura
   #include <LiquidCrystal_I2C.h>   //Librería para el manejo de la pantalla LCD
   #include <Servo.h>               //Librería para el control del Servo
   ```
   -  **Declaración de variables**<br>
   ```cpp
   double input, output, setpoint = 25.0, simTemp;   
   // input: valor actual de temperatura que entra al PID
   // output: salida del PID (Para control de enfriar o calentar)
   // setpoint: Temperatura objetivo de funcionamiento óptimo de baterías
   // simTemp: Variable que recoge el valor del sensor DHT22, para simulacon
   
   PID pid(&input, &output, &setpoint, 2.0, 0.5, 1.5, DIRECT); 
   // y los parámetros KP=2.0, KI=0.5, KD=1.5 en modo DIRECTO
   
   bool flag = false; // Protección ante rebotes del boton
   
   uint32_t btnTimer = 0;    
   int mode_PID = 0; // Almacena en que modo estoy del algoritmo PID para temperatura
   int count = 0;    
   unsigned long lastTime;
   ```
   -  **Función SETUP**<br>
   Esta función se ejecuta una única vez, en ella se incializa la consola para mostrar mensajes (11500 baudios), la pantalla LCD, se indica el direccionamiento de los pines y se hace una lectura del sensor para comprobar que funciona correctamente. Respecto al algoritmo PID, en esta función establecemos los límites de la salida, el modo de funcionamiento en AUTOMATIC y declaramos el BOTON para dentro de la función de control de temperatura podamos movernos en el menu y establecer la temperatura inicial y poner el PID a funcionar.
   ```cpp
      void setup() {
      Serial.begin(11500);
      lcd.init(); 
      lcd.backlight();
      
     pinMode(PIN_ALG_PID, INPUT_PULLUP);     //BOTON para pasar de modos dentro del algoritmo PID
     pinMode(STATE_BUTTON, INPUT_PULLUP);

      
      pid.SetMode(AUTOMATIC);
      pid.SetOutputLimits(-255, 255);
      
      int chk = DHT.read22(DHTPIN);
      if (chk == DHTLIB_OK) {
         simTemp = DHT.temperature;
      }
      lastTime = millis();
      }
   ```
Dentro de la función para el control de la temperatura con el algoritmo PID encontramos el código adjunto. Se divide en dos estados internos, uno en el que mediante la lectura del sensor DHT22 se almacena el valor de la temperatura en una variable para posteriormente en el modo interno dos se ejecute el algoritmo PID, en función de su valor activamos con diferentes combinaciones de colores un LED RGB, si es ROJO (Resistencias térmicas ON), VERDE (Temperatura en el rango óptimo) y AZUL (Ventilador de enfriamiento ON).

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

**ALGORITMO DE CONTROL DE LUMINOSIDAD**

Este algoritmo de control de la luminosidad ajusta su posición mediante un controlador PID. Implementa dos LDR, uno hará referencia al OESTE y otro al ESTE, con esto hemos querido imitar que si el sol (durante el amanecer) sale por el Este la luminosidad será máxima en el LDR y por lo tanto el SERVO (Placa solar) debe de orientarse hacia ese lado. De la misma manera si el sol se esta ya poniendo por el OESTE el servo se orienta hacía la caidad del sol e igualmente si el sol esta arriba del todo el servo apunta a la mitada.
También hemos puesto una cadena de LEDs que según va anocheciendo se van encendiendo. Así cuando sea de noche estarán todos encendidos y durante las horas del atardecer solo unos pocos. Para el control de los LEDs hemos utilizado un registro de desplazamiento 74HC595.

   -  **Declaración de variables**<br>
   ```cpp
      double luz_este, luz_oeste;
      double error_luz, salida_pid_luz;      
      double angulo_servo = 90;                            // Ángulo inicial del servo motor (posición media)
      // Configuración del controlador PID para la orientación solar
      PID pidLuz(&error_luz, &salida_pid_luz, 0, 1.2, 0.01, 0.3, DIRECT);

      pinMode(LATCH_PIN, OUTPUT);
      pinMode(CLOCK_PIN, OUTPUT);
      pinMode(DATA_PIN, OUTPUT);
   ```
   -  **Función CONTROL_LUMINOSIDAD_SOLAR**<br>
   ```cpp
   // Lectura de sensores de luz (LDR) del este y del oeste
   luz_este = analogRead(LDR_ESTE);                          // Lee el valor analógico del sensor de luz ESTE
   luz_oeste = analogRead(LDR_OESTE);                        // Lee el valor analógico del sensor de luz OESTE
   error_luz = luz_este - luz_oeste;                         // Calcula la diferencia de luz entre ESTE y OESTE

   // Ajuste del ángulo del servo
   angulo_servo += salida_pid_luz;                           // Se suma o resta la corrección al ángulo actual
   angulo_servo = constrain(angulo_servo, 0, 180);           // Limita el ángulo entre 0° y 180° (evita errores mecánicos)
   servoMotor.write((int)angulo_servo);                      // Envía el nuevo ángulo al servo

   int luzActual = analogRead(LDR_ESTE);                    
   int diferencia = UMBRAL_LUZ - luzActual;                  
   int ledsEncendidos = map(diferencia, UMBRAL_LUZ, 0, 0, 8);// Se convierte la diferencia en un número de LEDs a encender
   ledsEncendidos = constrain(ledsEncendidos, 0, 8);         // Asegura que no haya más de 8 LEDs encendidos

   // Crea el byte que representa qué LEDs deben encenderse (1 bit por LED)
   byte salida = 0;
   for (int i = 0; i < ledsEncendidos; i++) {
     bitSet(salida, i); // Enciende los bits del byte de salida, de izquierda a derecha
   }

   // Envía el byte al 74HC595 para controlar los LEDs
   digitalWrite(LATCH_PIN, LOW);                          
   shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, salida);         // Envía los bits (de más significativo a menos)
   digitalWrite(LATCH_PIN, HIGH);                           // Actualiza los valores de los LEDs

   lcd.setCursor(0, 0);              
   lcd.print("Seguidor Solar ");     
   lcd.setCursor(0, 1);             
   lcd.print("Ang: ");              
   lcd.print((int)angulo_servo);     
   lcd.print(" E-O:");              
   lcd.print((int)error_luz);        
   ```
---

## 🧪 Pruebas de simulación

Simulaciones realizadas para verificar el comportamiento del sistema:

- 🔍 **Pruebas del funcionamiento en WOKWI**
  
A continuación adjuntaremos una serie de imagenes y .gifs que demuestren el correcto funcionamiento del código completo. Este es el enlace para acceder a nuestro proyecto en WOKWI: https://wokwi.com/projects/432483156211621889<br>

-   *MODO 1: En este modo se van a visualizar todos los parámetros del ambiente, implementarán modos de control para cuando la temperatura sea <0ºC apage la carga de baterías y muestre un mensaje por el display LCD. También se controla la humedad, la distancia a la boya de algún objeto y la intensidad lumínica.*

  ![Captura de pantalla 2025-05-31 171659](https://github.com/user-attachments/assets/64447560-dc93-4da0-a30c-1600cfbb5c9f)

-   *MODO 2: En este modo se ejecutará el Algoritmo PID para el control de la temperatura, en el primer modo fijaremos la temperatura ambiente y en el segundo se muestra como actua el algoritmo PID recalculando el valor de la temperatura continuamente hasta ajustarlo a nuestro valor objetivo 25ºC* <br>
Fijando la temperatura ambiente -><br>
![Captura de pantalla 2025-05-31 172312](https://github.com/user-attachments/assets/8172b0e3-e3c8-4ac2-8b63-42456b228e7b)

El siguiente ejemplo ilustra cómo el algoritmo PID hace que la temperatura descienda y se ajuste al valor que queremos de temperatura en las baterías, siendo este de 25 °C. A medida que se aproxima a 25ºC se observa como la "correción" es más lenta y no van tán rápido.

El video es un fragmento donde se observan dos elementos clave: el primero es la pantalla LCD, que a diferencia del primer video cambia su interfaz y muestra cómo se autoajusta la temperatura gracias al algoritmo PID; y el LED RGB, que según la temperatura que haya en ese momento, cumple una función u otra. Es decir,<br> 
         -   si el LED está en AZUL, la temperatura está por encima de 25 °C (±1 °C) y se activa el ventilador;<br>
         -   si está en el rango de 25 °C (±1 °C), se pondrá en VERDE; <br>
         -   y si está por debajo, se activan las resistencias y se pone en ROJO.<br>
         ![CONTROL TEMPERATURA](https://github.com/user-attachments/assets/a4a00db8-5f12-4326-8cc0-a4e38cf2bf2a)

-   *MODO 3: Simulación del nivel de oleaje, el LED indicará el nivel de oleaje según su color, si esta en verde el mar está en calma, si esta en amarillo el oleaje empieza a ser peligroso y si esta en rojo el oleaje está al máximo y por el temporal se muestra en la pantalla un mensaje de perdida de conexión y se alerta con sonidos.*

![Captura de pantalla 2025-05-31 172904](https://github.com/user-attachments/assets/be1c5f15-a2c5-4508-ba4e-fd064581a76e)

-   *MODO 4: En esta imagen se observa el modo de funcionamiento que mide la velocidad del viento, en este modo según la velocidad del tiempo medida (valor introducido a través del potenciometro) se moverá el servo más o menos rápido.*

![Captura de pantalla 2025-05-31 172919](https://github.com/user-attachments/assets/d4b07a5d-d2ed-4f9b-b5be-b59929ba2d27)

-   *MODO 5: En este modo se implementa una simulación de la activación de las resistencias termicas encargadas de calentar la batería cuando desciende de 0ºC, se puede observar como variando la potencia que entregamos se calienta más o menos rápido, pero a consecuencia de consumir más baterías.*
  
![CALENTADOR DE RESISTENCIAS TERMICAS](https://github.com/user-attachments/assets/624b6aa5-8443-4908-b22a-5bdbe3eeff65)

-   *MODO 6: En este modo se muestra el control de la luminosidad, observamos los dos LDR que "indican" si habrá más luz en el OESTE o en el ESTE, según ese valor de luminosidad se orienta el servo a esa posición y si la luminosidad en el LDR del ESTE es mucha (el sol estará saliendo) o será pleno día por ende la cadena de LEDs se apagará, si el valor en Lux es pequeño ya estará comenzando a anochecer y se encenderán los LEDs.*

  ![CONTROL LUMINOSIDAD](https://github.com/user-attachments/assets/c9360a19-c686-4c0a-afa5-b581ab6001bc)

---
