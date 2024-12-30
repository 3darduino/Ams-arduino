#include <config.h>
#include <coolant_control.h>
#include <cpu_map.h>
#include <defaults.h>
#include <eeprom.h>
#include <gcode.h>
#include <grbl.h>
#include <jog.h>
#include <limits.h>
#include <motion_control.h>
#include <nuts_bolts.h>
#include <planner.h>
#include <print.h>
#include <probe.h>
#include <protocol.h>
#include <report.h>
#include <serial.h>
#include <settings.h>
#include <spindle_control.h>
#include <stepper.h>
#include <system.h>


#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);

#define STEP1_PIN 2
#define DIR1_PIN 5
#define STEP2_PIN 3
#define DIR2_PIN 6
#define STEP3_PIN 4
#define DIR3_PIN 7
#define STEP4_PIN 12
#define DIR4_PIN 13

#define SERVO1_PIN A0
#define SERVO2_PIN A1
#define SERVO3_PIN A2
#define SERVO4_PIN A3

#define ENABLE_PIN 8

#define UP_BUTTON_PIN 10

#define OK_BUTTON_PIN 11

#define SIMPLE_BUTTON_PIN 9  // Nuevo pulsador
int empujon = 1;
int vueltas = 5;
int vueltasrapidas =20;
int velocidad = 250;
int velocidadlenta = 3000;
int tiempoEspera = 3000;

Servo servo1, servo2, servo3, servo4;

int menuIndex = 0;
const int menuItems = 8;
const char* menuOptions[menuItems] = {
    "Extruir 1", "Retraer 1", "Extruir 2", "Retraer 2",
    "Extruir 3", "Retraer 3", "Extruir 4", "Retraer 4"
};

void setup() {
    lcd.init();
    lcd.backlight();

    pinMode(STEP1_PIN, OUTPUT);
    pinMode(DIR1_PIN, OUTPUT);
    pinMode(STEP2_PIN, OUTPUT);
    pinMode(DIR2_PIN, OUTPUT);
    pinMode(STEP3_PIN, OUTPUT);
    pinMode(DIR3_PIN, OUTPUT);
    pinMode(STEP4_PIN, OUTPUT);
    pinMode(DIR4_PIN, OUTPUT);

    servo1.attach(SERVO1_PIN);
    servo2.attach(SERVO2_PIN);
    servo3.attach(SERVO3_PIN);
    servo4.attach(SERVO4_PIN);

    servo1.write(180);
    servo2.write(180);
    servo3.write(180);
    servo4.write(180);

    delay(1000);

    servo1.detach();
    servo2.detach();
    servo3.detach();
    servo4.detach();
    

    pinMode(UP_BUTTON_PIN, INPUT_PULLUP);
    
    pinMode(OK_BUTTON_PIN, INPUT_PULLUP);

    pinMode(SIMPLE_BUTTON_PIN, INPUT_PULLUP);

    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, LOW);


    Serial.begin(9600);
    Serial.println("Iniciando...");
    lcd.setCursor(0, 0);
    lcd.print("Iniciando...");

    mostrarMenu();
}
void loop() {
    // Leer los pulsadores
    if (digitalRead(UP_BUTTON_PIN) == LOW) {
        menuIndex = (menuIndex > 0) ? menuIndex - 1 : menuItems - 1;
        mostrarMenu();
        delay(300); // Anti-rebote
    }
   
    if (digitalRead(OK_BUTTON_PIN) == LOW) {
        ejecutarOpcion(menuIndex);
        delay(300); // Anti-rebote
    }

    // Leer el cuarto pulsador para activar comandos directamente
    if (digitalRead(SIMPLE_BUTTON_PIN) == LOW) {
        long tiempoPulsado = millis();
        while (digitalRead(SIMPLE_BUTTON_PIN) == LOW) {
            // Esperar a que se suelte el botón
        }
        tiempoPulsado = millis() - tiempoPulsado;

        if (tiempoPulsado > 200 && tiempoPulsado < 1000) {
            ejecutarOpcion(8); // Extruir 1
        } else if (tiempoPulsado < 2000) {
            ejecutarOpcion(9); // Retraer 1
        } else if (tiempoPulsado < 3000) {
            ejecutarOpcion(10); // Extruir 2
        } else if (tiempoPulsado < 4000) {
            ejecutarOpcion(11); // Retraer 2
        } else if (tiempoPulsado < 5000) {
            ejecutarOpcion(12); // Extruir 3
        } else if (tiempoPulsado < 6000) {
            ejecutarOpcion(13); // Retraer 3
        } else if (tiempoPulsado < 7000) {
            ejecutarOpcion(14); // Extruir 4
        } else if (tiempoPulsado < 8000) {
            ejecutarOpcion(15); // Retraer 4
        } 

        delay(300); // Anti-rebote
    }
}

void mostrarMenu() {
    lcd.clear();
    lcd.setCursor(7, 0);
    lcd.print("Menu:");
    lcd.setCursor(5, 2);
    lcd.print(menuOptions[menuIndex]);
}

void ejecutarOpcion(int opcion) {
    switch (opcion) {
        case 0:
            manejarFilamentorapido(1, STEP1_PIN, DIR1_PIN, servo1, true); // Extruir 1
            break;
        case 1:
            manejarFilamentorapido(1, STEP1_PIN, DIR1_PIN, servo1, false); // Retraer 1
            break;
        case 2:
            manejarFilamentorapido(2, STEP2_PIN, DIR2_PIN, servo2, true); // Extruir 2
            break;
        case 3:
            manejarFilamentorapido(2, STEP2_PIN, DIR2_PIN, servo2, false); // Retraer 2
            break;
        case 4:
            manejarFilamentorapido(3, STEP3_PIN, DIR3_PIN, servo3, true); // Extruir 3
            break;
        case 5:
            manejarFilamentorapido(3, STEP3_PIN, DIR3_PIN, servo3, false); // Retraer 3
            break;
        case 6:
            manejarFilamentorapido(4, STEP4_PIN, DIR4_PIN, servo4, true); // Extruir 4
            break;
        case 7:
            manejarFilamentorapido(4, STEP4_PIN, DIR4_PIN, servo4, false); // Retraer 4
            break;
        case 8:
            manejarFilamento(1, STEP1_PIN, DIR1_PIN, servo1, true); // Extruir 1
            break;
        case 9:
            manejarFilamento(1, STEP1_PIN, DIR1_PIN, servo1, false); // Retraer 1
            break;
        case 10:
            manejarFilamento(2, STEP2_PIN, DIR2_PIN, servo2, true); // Extruir 2
            break;
        case 11:
            manejarFilamento(2, STEP2_PIN, DIR2_PIN, servo2, false); // Retraer 2
            break;
        case 12:
            manejarFilamento(3, STEP3_PIN, DIR3_PIN, servo3, true); // Extruir 3
            break;
        case 13:
            manejarFilamento(3, STEP3_PIN, DIR3_PIN, servo3, false); // Retraer 3
            break;
        case 14:
            manejarFilamento(4, STEP4_PIN, DIR4_PIN, servo4, true); // Extruir 4
            break;
        case 15:
            manejarFilamento(4, STEP4_PIN, DIR4_PIN, servo4, false); // Retraer 4
            break;
    }
    mostrarMenu(); // Volver al menú después de completar la acción
}
void manejarFilamento(int motorNumero, int stepPin, int dirPin, Servo& servo, bool extruir) {
    if (extruir) {
        // Agarrar el filamento y extruir
        mensajeLCD("Servo " + String(motorNumero) + ": Extruyendo");
        servo.attach(obtenerServoPin(motorNumero));
        moverServo(servo, 0);
        delay(1000);
        mensajeLCD("Motor " + String(motorNumero) + ": Extruir");
        moverMotor(stepPin, dirPin, HIGH, vueltas); // 20 vueltas
        delay(tiempoEspera);
        moverMotorlento(stepPin, dirPin, HIGH, empujon); // 2 vueltas más
        mensajeLCD("Servo " + String(motorNumero) + ":Libre");
        moverServo(servo, 180);
        delay(1000);
        servo.detach();
    } else {
        // Agarrar el filamento y retraer
        mensajeLCD("Servo " + String(motorNumero) + ": Extruyendo");
        servo.attach(obtenerServoPin(motorNumero));
        moverServo(servo, 0);
        delay(1000);
        mensajeLCD("Motor " + String(motorNumero) + ": Retraer");
        moverMotor(stepPin, dirPin, LOW, vueltas); // 20 vueltas
        mensajeLCD("Servo " + String(motorNumero) + ":Libre");
        moverServo(servo, 180);
        delay(1000);
        servo.detach();
    }
}
void manejarFilamentorapido(int motorNumero, int stepPin, int dirPin, Servo& servo, bool extruir) {
    if (extruir) {
        // Agarrar el filamento y extruir
        mensajeLCD("Servo " + String(motorNumero) + ": Extruyendo");
        servo.attach(obtenerServoPin(motorNumero));
        moverServo(servo, 0);
        delay(1000);
        mensajeLCD("Motor " + String(motorNumero) + ": Extruir");
        moverMotor(stepPin, dirPin, HIGH, vueltasrapidas); // 20 vueltas
        delay(tiempoEspera);
        mensajeLCD("Servo " + String(motorNumero) + ":Libre");
        moverServo(servo, 180);
        delay(1000);
        servo.detach();
    } else {
        // Agarrar el filamento y retraer
        mensajeLCD("Servo " + String(motorNumero) + ": Extruyendo");
        servo.attach(obtenerServoPin(motorNumero));
        moverServo(servo, 0);
        delay(1000);
        mensajeLCD("Motor " + String(motorNumero) + ": Retraer");
        moverMotor(stepPin, dirPin, LOW, vueltasrapidas); // 20 vueltas
        mensajeLCD("Servo " + String(motorNumero) + ": Libre");
        moverServo(servo, 180);
        delay(1000);
        servo.detach();
    }
}

void moverMotor(int stepPin, int dirPin, int dir, int vueltas) {
    // Configurar la dirección del motor
    digitalWrite(dirPin, dir);
    // Girar el motor el número de vueltas completas
    for (int i = 0; i < vueltas * 800; i++) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(velocidad);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(velocidad);
    }
}
void moverMotorlento(int stepPin, int dirPin, int dir, int vueltas) {
    // Configurar la dirección del motor
    digitalWrite(dirPin, dir);
    // Girar el motor el número de vueltas completas
    for (int i = 0; i < vueltas * 800; i++) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(velocidadlenta);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(velocidadlenta);
    }
}
void moverServo(Servo& servo, int angulo) {
    // Mover el servo al ángulo especificado
    servo.write(angulo);
    delay(1000); // Esperar a que el servo alcance la posición
}

int obtenerServoPin(int motorNumero) {
    switch (motorNumero) {
        case 1: return SERVO1_PIN;
        case 2: return SERVO2_PIN;
        case 3: return SERVO3_PIN;
        case 4: return SERVO4_PIN;
        default: return -1; // Error: pin no válido
    }
}

void mensajeLCD(const String& mensaje) {
    // Mostrar el mensaje en el LCD y en el monitor serial
    Serial.println(mensaje);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(mensaje);
}

