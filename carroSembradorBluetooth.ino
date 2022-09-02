//Librerias
#include "BluetoothSerial.h"
#include <Arduino.h>
#include <ESP32Servo.h>

//Definicion de variables
#define MotorDerechoAdelante 32
#define MotorDerechoAtras 33
#define MotorIzquierdoAdelante 25
#define MotorIzquierdoAtras 26
#define MotorTuercaAdelante 22
#define MotorTuercaAtras 23
#define ledRojo 19
#define ledVerde 18
#define ledAzul 5
char caracter;

//Variables enable para los motores Derecha e Izquierda el control de velocidad del carro
#define enable1Pin 14
#define enable2Pin 12
#define enable3Pin 15

//Configuracion Bluetooth
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

//Configuracion para obtencion de la temperatura interna de la ESP32
#ifdef __cplusplus
extern "C" {
#endif
uint8_t temprature_sens_read();
#ifdef __cplusplus
}
#endif
uint8_t temprature_sens_read();

//Variables para el control de velocidad del carro
const int freq = 8000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 100;

//Instancias
Servo servo1;
Servo servo2;

//Funcion que determina las direcciones en las que se movera el carro
void direccionesAuto(int valorMotorDerAde, int valorMotorDerAtr, int valorMotorIzqrAde, int valorMotorIzqAtr, int tiempo){
  //analogWrite(enable1Pin, 100);
  //analogWrite(enable2Pin, 100);
  digitalWrite(MotorDerechoAdelante, valorMotorDerAde);
  digitalWrite(MotorDerechoAtras, valorMotorDerAtr);
  digitalWrite(MotorIzquierdoAdelante, valorMotorIzqrAde);
  digitalWrite(MotorIzquierdoAtras, valorMotorIzqAtr);
  delay(tiempo);
  //analogWrite(enable3Pin, 100);
}

//Funcion que determina el manejo manual del carro, segun la acciones de los botones en la appp
void direccionesManual(int valorMotorDerAde, int valorMotorDerAtr, int valorMotorIzqrAde, int valorMotorIzqAtr){
  //analogWrite(enable1Pin, 100);
  //analogWrite(enable2Pin, 100);
  digitalWrite(MotorDerechoAdelante, valorMotorDerAde);
  digitalWrite(MotorDerechoAtras, valorMotorDerAtr);
  digitalWrite(MotorIzquierdoAdelante, valorMotorIzqrAde);
  digitalWrite(MotorIzquierdoAtras, valorMotorIzqAtr);
  //analogWrite(enable3Pin, 100);
}

//Funcion que manejara el movimiento de los servos
void movimientoServos(Servo servo, int tiempo, int anguloIni, int anguloFin){
  servo.write(anguloIni);
  delay(tiempo);
  servo.write(anguloFin);
  delay(tiempo);
}

//Activa el motor que servira para hacer el hueco el la tierra y llamara al servo1 para
//que este permita caer el motor al nivel del suelo
void taladrar(int valorMotorTuerAde, int valorMotorTueAtr, int tiempo){
  analogWrite(enable3Pin, 255);
  digitalWrite(MotorTuercaAdelante, valorMotorTuerAde);
  digitalWrite(MotorTuercaAtras, valorMotorTueAtr);
  delay(tiempo);
  digitalWrite(MotorTuercaAdelante, valorMotorTueAtr);
  digitalWrite(MotorTuercaAtras, valorMotorTueAtr);
}

////Activa el servo2 que permitira que la semilla caiga en el hueco el la tierra
void sembrar(){
  movimientoServos(servo2,500,0,30);
}

//Mostrara un color determinado segun la accion que el carro este realizando en ese momento
void encendidoLedAuto(int ledRojo1, int ledAzul1, int ledVerde1, int tiempo){
  digitalWrite(ledRojo,ledRojo1);
  digitalWrite(ledVerde,ledVerde1);
  digitalWrite(ledAzul,ledAzul1);
  delay(tiempo);
}

//Mostrara el color asignado por cada opcion de manera manual
void encendidoLedManual(int ledRojo1, int ledAzul1, int ledVerde1){
  digitalWrite(ledRojo,ledRojo1);
  digitalWrite(ledVerde,ledVerde1);
  digitalWrite(ledAzul,ledAzul1);
}

void setup() {
  SerialBT.begin("BluetoothCarroESP32");
  Serial.begin(9600);
  Serial.begin(115200);
  
  //Seteando variables de salida
  pinMode(MotorDerechoAdelante, OUTPUT);
  pinMode(MotorDerechoAtras, OUTPUT);
  pinMode(MotorIzquierdoAdelante, OUTPUT);
  pinMode(MotorIzquierdoAtras, OUTPUT);
  pinMode(MotorTuercaAdelante, OUTPUT);
  pinMode(MotorTuercaAtras, OUTPUT);
  pinMode(ledRojo,OUTPUT);
  pinMode(ledVerde,OUTPUT);
  pinMode(ledAzul,OUTPUT);

  //Asignando los ciclos de velocidad para los motores Derecha e Izquierda
  pinMode(enable1Pin, OUTPUT);
  ledcSetup(pwmChannel, freq, resolution);
  pinMode(enable2Pin, OUTPUT);
  ledcSetup(pwmChannel, freq, resolution);
  pinMode(enable3Pin, OUTPUT);
  analogWrite(enable1Pin, 250);
  analogWrite(enable2Pin, 250);
  analogWrite(enable3Pin, 250);
  
  digitalWrite(MotorDerechoAdelante, LOW);
  digitalWrite(MotorDerechoAtras, LOW);
  digitalWrite(MotorIzquierdoAdelante, LOW);
  digitalWrite(MotorIzquierdoAtras, LOW);
  digitalWrite(MotorTuercaAdelante, LOW);
  digitalWrite(MotorTuercaAtras, LOW);

  //iniciamos el servo para que empieze a trabajar con el pin 21 y 4
  servo1.attach(21);
  servo2.attach(4);
}

void loop() {
  // put your main code here, to run repeatedly:
  //Temperatura interna en C
  float tempInt=(temprature_sens_read() - 32) / 1.8;
    //Lectura de temperatura enviada por Bluetooth a la aplicacion
    SerialBT.print(tempInt);
    SerialBT.print(",");
  if (SerialBT.available() > 0) {
    //SerialBT.println(";");
    caracter = SerialBT.read();
     switch(caracter) {
      case 'A' :
      //Adelante: COLOR IDENTIFICATIVO 'VERDE'
        direccionesManual(255,0,255,0);
        encendidoLedManual(0,0,255);
        Serial.println("AdelanteM");
        break;
      case 'D' :
      //Derecha: COLOR IDENTIFICATIVO 'MAGENTA'
        direccionesManual(255,0,0,0);
        encendidoLedManual(255,255,0);
        Serial.println("DerechaM");
        break;
      case 'I' :
      //Izquierda: COLOR IDENTIFICATIVO 'AMARILLO'
         direccionesManual(0,0,255,0);
         encendidoLedManual(255,0,255);
         Serial.println("IzquierdaM");
         break;
      case 'R' :
      //Atras: COLOR IDENTIFICATIVO 'AZUL'
         direccionesManual(0,255,0,255);
         encendidoLedManual(0,255,0);
         Serial.println("AtrasM");
         break;
      case 'P' :
      //Parar: COLOR IDENTIFICATIVO 'ROJO'
         direccionesManual(0,0,0,0);
         encendidoLedManual(255,0,0);
         Serial.println("PararM");
         break;
      case 'T' :
         movimientoServos(servo1,5000,0,180);
         taladrar(255,0,3000);
         Serial.println("TaladrarM");
         break;
      case 'S' :
         movimientoServos(servo2,500,0,15);
         Serial.println("SembrarM");
         break;
      case 'X':
         Serial.println("Control Automatizado");
          //--------------------PRIMERA VUELTA
          //Adelante: COLOR IDENTIFICATIVO 'VERDE'
          direccionesAuto(255,0,255,0,3000);
          encendidoLedAuto(0,0,255,3000);
          //Parar: COLOR IDENTIFICATIVO 'ROJO'
          direccionesAuto(0,0,0,0,1000);
          encendidoLedAuto(255,0,0,1000);
          //Taladrar
          movimientoServos(servo1,2000,0,180);
          taladrar(255,0,3000);
          //Sembrar
          sembrar();
          //--------------------SEGUNDA VUELTA
          //Adelante: COLOR IDENTIFICATIVO 'VERDE'
          direccionesAuto(255,0,255,0,3000);
          encendidoLedAuto(0,0,255,3000);
          //Parar: COLOR IDENTIFICATIVO 'ROJO'
          direccionesAuto(0,0,0,0,1000);
          encendidoLedAuto(255,0,0,1000);
          //Taladrar
          movimientoServos(servo1,2000,0,180);
          taladrar(255,0,3000);    
          //Sembrar
          sembrar();
      default :
         printf("oK");
   }
  }

}
