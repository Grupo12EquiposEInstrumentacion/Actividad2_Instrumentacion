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

- 🧾 **Código comentado**

---

## 🧪 Pruebas de simulación

Simulaciones realizadas para verificar el comportamiento del sistema:

- 🔍 **Pruebas del funcionamiento en WOKWI**
  
IMAGENES DE WOKWI FUNCIONANDO  

---
