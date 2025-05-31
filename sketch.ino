//GRUPO 12

/*******************************************************
 *             ACTIVIDAD 2, BOYA DE MAR                *
 *                LISTA DE TAREAS                      *
 *******************************************************/

#include <LiquidCrystal_I2C.h>   //Librería para el manejo de la pantalla LCD
#include <dht.h>                 //Librería para el manejo del sensor DHT22
#include <Servo.h>               //Librería para el control del Servo
#include <PID_v1.h>

#define LDR_OESTE A0            //PIN Analog 0 LDR que imita sol saliendo por el OESTE
#define LDR_ESTE A2             //PIN Analog 2 LDR que imita sol saliendo por el ESTE
#define DHTPIN 3                //PIN Digit 3 DHT22 (Humedad y Temperatura) 
#define PIN_TRIG 5              //PIN Digit 5 Salida Sensor Distancia 
#define PIN_ECHO 9              //PIN Digit 6 Entrada Sensor Distancia
#define STATE_BUTTON 6          //PIN Analog 2 Pulsador avanzar por estados
#define PIN_ALG_PID 8            //BOTON para el control del algoritmo PID
#define red_led 13              //PIN Digit 13 LED Canal R
#define green_led 12            //PIN Digit 12 LED Canal G
#define blue_led 11             //PIN Digit 11 LED Canal B
#define altavoz 4               //PIN Digit 4 Altavoz (Buzzer)
#define potenciometro A1        //PIN Analog 1 Potenciometro

#define SERVO 10
#define DATA_PIN 11    // DS
#define LATCH_PIN 2    // ST_CP
#define CLOCK_PIN 7   // SH_CP
#define UMBRAL_LUZ 818  // 80% de iluminación natural (1023 * 0.8)

// Simbolo de gota de agua
byte hum[8] = { 
  B00100, B00100, B01010, B01010, B10001, B10001, B10001, B01110 
};

// Simbolo LCD Temperatura
byte temp[8] = { 
  B00100, B01010, B01010, B01110, B01110, B11111, B11111, B01110 
};

// Flecha derecha
byte arrowRight[8] = { 
  B00000, B00100, B00110, B11111, B00110, B00100, B00000, B00000 
};

//Campana alarma
byte bell[] = {
  B00100, B01110, B01110, B01110, B11111, B00000, B00100, B00000
};


             /*PHOTORESISTOR (LDR) SENSOR MODULE */
const float GAMMA = 0.7;          //Valores extraidos del DATASHEET 
const float RL10 = 50;            //RL10 (Resistencia 10kohm del LDR)

             /*    Lectura del sensor DHT22      */
dht DHT;
             /*Variable para el control del servo*/
Servo servoMotor;
             /*    Inicializar la pantalla LCD   */
LiquidCrystal_I2C lcd(0x27,20,4); //LiquidCrystal_I2C lcd(0x27, 16, 2);

//Variables de control
int count = 0, count_GENERAL = 0;
int mode = 0, mode_PID = 0; 
bool flag1 = false, flag = false;
uint32_t btnTimer1 = 0, btnTimer = 0;
unsigned long lastTime;
const int ZONA_MUERTA = 5;  // umbral mínimo para mover el servo

//Variables generales
int aux_clear_lcd = 0;

/*VARIABLES ALGORITMO PID TEMPERATURA*/
double input, output, setpoint = 25.0, simTemp;
PID pidTemp(&input, &output, &setpoint, 2.0, 0.5, 1.5, DIRECT);

/*VARIABLES ALGORITMO PID ORIENTACION SOLAR*/
double luz_este, luz_oeste;
double error_luz, salida_pid_luz;
double angulo_servo = 90;
PID pidLuz(&error_luz, &salida_pid_luz, 0, 1.2, 0.01, 0.3, DIRECT);

void setup() { //La rutina SETUP se ejecuta una vez
  Serial.begin(11500);
  pinMode(LDR_OESTE, INPUT);
  pinMode(LDR_ESTE, INPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(STATE_BUTTON, INPUT_PULLUP);
  pinMode(PIN_ALG_PID, INPUT_PULLUP);
  pinMode(red_led, OUTPUT);
  pinMode(green_led, OUTPUT);
  pinMode(blue_led, OUTPUT);
  pinMode(altavoz, OUTPUT);
  pinMode(LATCH_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);


  lcd.init(); lcd.backlight();
  lcd.createChar(3, bell); //Creación de carácteres personalizados
  lcd.createChar(4, arrowRight);
  lcd.createChar(5, hum);
  lcd.createChar(7, temp);

  pidTemp.SetMode(AUTOMATIC);
  pidTemp.SetOutputLimits(-255, 255);
  pidLuz.SetMode(AUTOMATIC);
  pidLuz.SetOutputLimits(-5, 5); // paso máximo de corrección por loop
  
  servoMotor.attach(SERVO);
  lastTime = millis(); 
}

void loop() { 
  handleButton();

  switch(mode) {
    case 0: modo_ahorro(); break;
    case 1: visualizar_parametros(); break;
    case 2: algoritmoPID_temp(); break;
    case 3: sim_oleaje(); break;
    case 4: sim_anemometro(); break;
    case 5: sim_resistencias_termicas(); break;
    case 6: control_luminosidad_solar(); break;
  }

  delay(100);  // Pequeña pausa para evitar saturar el sistema  
}

/*  FUNCIÓN PARA AVANZAR POR LOS MODOS DEL CÓDIGO  */
void handleButton(){
  //Función extraída del DATASHEET del Fotoresistor
  int analogValue = analogRead(LDR_OESTE);
  float voltage = analogValue / 1024. * 5;
  float resistance = 2000 * voltage / (1 - voltage / 5);
  float lux = pow(RL10 * 1e3 * pow(10, GAMMA) / resistance, (1 / GAMMA)); 

  bool btnState = !digitalRead(STATE_BUTTON);
  if (btnState && !flag && millis() - btnTimer > 100) {
    flag = true;
    btnTimer = millis();
    count++;
    if (count > 6) count = 0; // Modos: 0 a 5
    mode = count;

    Serial.print("Modo actual: ");
    Serial.println(mode);

    noTone(altavoz);
    lcd.clear();
    setColor(false, false, false);
  }

  if (!btnState && flag && millis() - btnTimer > 100) {
    flag = false;
    btnTimer = millis();
  }

}

/* MODO AHORRO DE BATERÍA */
void modo_ahorro(){
    lcd.setCursor(0,1);
    lcd.print("                    ");
    lcd.setCursor(0,2);
    lcd.print("                    ");
    lcd.setCursor(0,0);
    lcd.print("                    ");
    lcd.noBacklight();
}

/*  CONTROL DE LA LUMINOSIDAD MEDIANTE EL SIGUIENTE CON PLACA SOLAR  */
void control_luminosidad_solar(){
  luz_este = analogRead(LDR_ESTE);
  luz_oeste = analogRead(LDR_OESTE);
  error_luz = luz_este - luz_oeste;

  // ZONA MUERTA para evitar ajustes por pequeñas diferencias
  if (abs(error_luz) < ZONA_MUERTA) {
    salida_pid_luz = 0;
  } else {
    pidLuz.Compute();
  }

  angulo_servo += salida_pid_luz;
  angulo_servo = constrain(angulo_servo, 0, 180);  // Protección de límite
  servoMotor.write((int)angulo_servo);

  // Control iluminación con 8 LEDs y 74HC595
  int luzActual = analogRead(LDR_ESTE);
  int diferencia = UMBRAL_LUZ - luzActual;
  int ledsEncendidos = map(diferencia, UMBRAL_LUZ, 0, 0, 8);
  ledsEncendidos = constrain(ledsEncendidos, 0, 8);

  byte salida = 0;
  for (int i = 0; i < ledsEncendidos; i++) {
    bitSet(salida, i);
  }
  //Encender cadena de LEDs
  digitalWrite(LATCH_PIN, LOW);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, salida);
  digitalWrite(LATCH_PIN, HIGH);

  lcd.setCursor(0, 0);
  lcd.print("Seguidor Solar ");
  lcd.setCursor(0, 1);
  lcd.print("Ang: ");
  lcd.print((int)angulo_servo);
  lcd.print(" E-O:");
  lcd.print((int)error_luz);

  delay(100);
}

/*  SIMULACIÓN RESISTENCIAS TERMICAS EN RESISTENCIAS*/
void sim_resistencias_termicas(){
  static float temp_bateria = -2.0; // Temperatura inicial simulada
  int rawControl = analogRead(A3); 
  float potencia = map(rawControl, 0, 1023, 0, 100); // Potencia %

  // Activa la "resistencia térmica"
  if (temp_bateria < 25.0 && potencia > 0) {
    temp_bateria += 0.1 * (potencia / 100.0); // Subida progresiva
  } else if (potencia == 0 && temp_bateria > -4.0) {
    temp_bateria -= 0.05; // Se enfría lentamente si no hay calefacción
  }

  lcd.backlight();
  lcd.display();
  lcd.setCursor(0, 0);
  lcd.print("Bateria: ");
  lcd.print(temp_bateria, 1);
  lcd.print(" C   ");

  lcd.setCursor(0, 1);
  lcd.print("Potencia: ");
  lcd.print(potencia, 0);
  lcd.print("%    ");

  if (temp_bateria < 0.0) {
    setColor(false, false, true); // Azul = muy frio
  } else if (temp_bateria < 10.0) {
    setColor(true, true, false); // Amarillo = calentando
  } else {
    setColor(false, true, false); // Verde = OK
  }

  delay(500);


}


/*  SIMULACIÓN DE ANEMÓMETRO  */
void sim_anemometro(){
  lcd.backlight();
  lcd.display();
  lcd.setCursor(0,0);
  lcd.print("Velocidad viento:");

  int rawValue = analogRead(potenciometro);          
  float windSpeed = map(rawValue, 0, 1023, 0, 100);  // 0-100 km/h
  int delayTime = map(rawValue, 0, 1023, 500, 50);   // Más viento = menos delay = más rápido

  servoMotor.write(45);                                  // Giro hacia un lado
  delay(delayTime);
  servoMotor.write(135);                                 // Giro hacia el otro lado
  delay(delayTime);

  lcd.setCursor(0,1);
  lcd.print(windSpeed);
  lcd.print(" km/h     ");
}

/* SIMULACIÓN DEL OLEAJE  */
void sim_oleaje(){
  lcd.backlight();
  lcd.display();
  lcd.setCursor(0, 0);
  lcd.print("Nivel de oleaje:");

  int rawValue = analogRead(potenciometro);
  int oleaje = map(rawValue, 0, 1023, 0, 100);

  lcd.setCursor(0, 1);
  lcd.print(oleaje);
  lcd.print(" %             ");

  if (oleaje < 30) {
    // Mar en calma
    setColor(false, true, false); // Verde
    noTone(altavoz);
  } 
  else if (oleaje < 70) {
    // Oleaje medio
    setColor(true, true, false); // Amarillo
    noTone(altavoz);
  } 
  else {
    // Oleaje fuerte → pérdida de conexión
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("PERDIDA CONEXION");
    lcd.setCursor(0, 1);
    lcd.print("INTENTANDO RECON...");
    
    tone(altavoz, 800);
    for (int i = 0; i < 6; i++) {
      setColor(true, false, false); // Rojo
      delay(300);
      setColor(false, false, false);
      delay(300);
    }
    noTone(altavoz);
  }

  delay(500);

}


//Esta función convierte 0-->-4ºC y 1023-->25ºC
int conversion_temperatura(int value){
  if(value == -0.12 or value == -0.09){
    value = 0.48;
  }
  return -4.0 + (value * (25.0 + 4.0) / 1023.0);
}


 /*/////////// VISUALIZO PARAMETROS AMBIENTE ///////////*/
void visualizar_parametros(){
  //Función extraída del DATASHEET del Fotoresistor
  //Calculo de parámetros y conversión
  int analogValue = analogRead(LDR_OESTE);
  float voltage = analogValue / 1024. * 5;
  float resistance = 2000 * voltage / (1 - voltage / 5);
  float lux = pow(RL10 * 1e3 * pow(10, GAMMA) / resistance, (1 / GAMMA));


  /////////////////////////////////////////TEMPERATURA
  //Variables para Temperatura y Humedad
  DHT.read22(DHTPIN);
  float temp = DHT.temperature;
  float hum = DHT.humidity;

  //Inicializo la LCD Display
  lcd.backlight();
  lcd.display();
  lcd.setCursor(0,0);

  //Visualizo la temperatura en la línea 1 del Display
  lcd.setCursor(0, 0);
  lcd.write(7);
  lcd.setCursor(1, 0);
  lcd.print(temp, 1); //lcd.print(valor a visualizar, número de decimales)
  lcd.print(" C   ");
  if(temp < 0.0){
    tone(altavoz, 1000);            // Enciende altavoz
    setColor(true, false, false); 
    lcd.setCursor(10, 0);
    lcd.print("CHARGE OFF");
    delay(700);
    setColor(false, false, false);  // Apago el LED RGB
    aux_clear_lcd = 1;
  }
  else {
  if(aux_clear_lcd == 1){
    aux_clear_lcd = 0;
    // Borra la línea donde estaba "CHARGE OFF"
    lcd.setCursor(10, 0); // Mismo sitio donde escribiste el texto
    lcd.print("          "); // 10 espacios para borrar el mensaje
  }

  noTone(altavoz);                // Apaga altavoz
}

  /////////////////////////////////////////HUMEDAD
  lcd.setCursor(0, 1);  
  lcd.write(5);
  lcd.setCursor(1,1);
  lcd.print(DHT.humidity,1);
  lcd.print(" %   ");

  /////////////////////////////////////////LUMINOSIDAD
  lcd.setCursor(0,2);
  lcd.print("* ");
  lcd.print(lux,1);
  lcd.print("  lux      ");
  lcd.setCursor(0,2);

  if(lux < 50){
    setColor(true, true, false);
    delay(300);
    setColor(false, false, false);
    delay(300);
  } else {
    setColor(false, false, false);
  }

  /////////////////////////////////////////DISTANCIA
  
  digitalWrite(PIN_TRIG, HIGH); //Inicio de nueva medida distancia
  delayMicroseconds(5);
  digitalWrite(PIN_TRIG, LOW);

  int duration = pulseIn(PIN_ECHO, HIGH);
  if((duration / 58) < 20){
    tone(altavoz, 1000);
    setColor(true, true, false);
  } else {
    noTone(altavoz);
  }
  lcd.setCursor(0,3);
  lcd.write(4);
  lcd.print((duration / 58),1);
  lcd.print("   m   ");

  delay(100);
}

/* ALGORITMO PID para el control de la temperatura*/
void algoritmoPID_temp(){
  bool btnState1 = !digitalRead(PIN_ALG_PID);
  if (btnState1 && !flag1 && millis() - btnTimer1 > 100) {
    flag1 = true;
    btnTimer1 = millis();
    count_GENERAL++;
    if (count_GENERAL > 1) count_GENERAL = 0;
    mode_PID = count_GENERAL;
    lcd.clear();
  }
  if (!btnState1 && flag1 && millis() - btnTimer1 > 100) {
    flag1 = false;
    btnTimer1 = millis();
  }

  // Submodo 0: lectura de temperatura
  if (mode_PID == 0) {
    int chk = DHT.read22(DHTPIN);
    if (chk == DHTLIB_OK) simTemp = DHT.temperature;
      lcd.setCursor(0, 0);
      lcd.print("TEMP AMB        ");
      lcd.setCursor(0, 1);
      lcd.print(simTemp, 1);
      lcd.print(" C        ");
    }

  // Submodo 1: PID de temperatura
  else {
    input = simTemp;
    pidTemp.Compute();

    simTemp += (output / 255.0) * 5.0 - 0.1 * (simTemp - setpoint);

    double delta = simTemp - setpoint;
      if (delta > 1.0) {
        analogWrite(red_led, 0);
        analogWrite(green_led, 0);
        analogWrite(blue_led, 255);  // Azul = enfriando
      } else if (delta < -1.0) {
        analogWrite(red_led, 255);
        analogWrite(green_led, 0);
        analogWrite(blue_led, 0);    // Rojo = calentando
      } else {
        analogWrite(red_led, 0);
        analogWrite(green_led, 255);
        analogWrite(blue_led, 0);    // Verde = estable
      }
        lcd.setCursor(0, 0);
        lcd.print("MODO PID        ");
        lcd.setCursor(0, 1);
        lcd.print(simTemp, 2);
        lcd.print(" C ->25C ");
      }

  delay(50);

}

//Función que inicializa el LED RGB al color correspondiente
void setColor(bool r, bool g, bool b) {
  digitalWrite(red_led, r ? HIGH : LOW);
  digitalWrite(green_led, g ? HIGH : LOW);
  digitalWrite(blue_led, b ? HIGH : LOW);
  delay(100);
}