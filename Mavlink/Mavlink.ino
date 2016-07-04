#include <NewPing.h>
#include "C:\Users\Fu\Documents\Arduino\libraries\mavlink\common\mavlink.h"

//==================================INICIALIZACIONES======================================//
/*Inicialización de los pines de los sensores. Librería "NewPing"
  NewPing NAME(Trigger, Echo, MAXDIST);
  El valor de MAXDIST es la distancia máxima que mide la librería.
  Si algún Echo devuelve un valor mayor de dicha distancia, se descarta automáticamente*/
NewPing sonar0(3, 4, 300);
NewPing sonar1(5, 6, 300);
NewPing sonar2(7, 8, 300);
NewPing sonar3(9, 10, 300);
NewPing sonar4(11, 12, 300);

//Variable utilizada para controlar que el HeartBeat se envíe cada segundo
unsigned long HeartbeatTime = 0; 

//Variables utilizadas para que sólo envie un RCOverride cada vez
//que se modifique, y no saturar a la controladora de ordenes redundantes
uint16_t PitchOut;
uint16_t RollOut;
uint8_t n = 0;
bool enviado = false;

#define NDistancias 5
#define DistanciaCerca 150  //Distancia a la que empieza a actuar el control
#define AltMin 100 //Altura a la que empieza a actuar el control
#define DistMin 20 //Diferencia mínima entre dos distancias del mismo eje para moverse.
#define Compensacion 100 //Tiempo que actúa la compensación de inercia

//Registro para guardar los datos de cada sensor
struct Sensores {
  uint16_t Distancias[NDistancias] = {0};
  uint16_t MediaDistancias = 0;
  bool Cerca = false;
  bool Activo = false;
  unsigned long CompensarTime = 0;
};

//Se inician las variables de cada sensor
#define NSensores 5
Sensores Sensor[NSensores];

//====================================PROGRAMA============================================//

void setup() {
  Serial.begin(57600);
}

void loop() {
  if ( (millis() - HeartbeatTime) > 1000 ) {
    HeartbeatTime = millis();
    FHeartBeat();
  }
  FSensores();
  FRCOverride();
}

//===========================================FUNCIONES====================================//
//Tarea encargada de medir los sensores
void FSensores() {
  ShiftArrays();
  MedirSensores();
  MediaDistancias();
  ComprobarDistancias();
}

//Tarea que envía los comandos de movimiento según las distancias detectadas por los sensores
void FRCOverride() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len;
    
  uint16_t Pitch = ComprobarPitch(Pitch);
  uint16_t Roll = ComprobarRoll(Roll);
  if( Pitch != PitchOut || Roll != RollOut ) {
    //Sólo envía cuando cambia, se evita sobrecargar la controladora
    PitchOut = Pitch;
    RollOut = Roll;
    enviado = false;
  }else if( enviado == false ) {
    n += 1;
    if(n == 2){
      //Se evita que se manden comandos distintos de manera consecutiva. 
      CompensarInercia();
      n = 0;
      RCOverride(&msg, len, buf, PitchOut, RollOut);
      enviado = true;
    }
  }
}

//Desplaza cada array de Distancias en una posición
void ShiftArrays() {
  for (uint8_t i = 0; i < NSensores; i++) {
    for (uint8_t j = NDistancias - 1; j > 0; j--) {
      Sensor[i].Distancias[j] = Sensor[i].Distancias[j - 1];
    }
  }
}

//==================================SENSORES=====================================//
//Se miden los sensores, y se colocan en la posición 0 de cada array
void MedirSensores() {
  Sensor[0].Distancias[0] = sonar0.ping_cm();
  Sensor[1].Distancias[0] = sonar1.ping_cm();
  Sensor[2].Distancias[0] = sonar2.ping_cm();
  Sensor[3].Distancias[0] = sonar3.ping_cm();
  Sensor[4].Distancias[0] = sonar4.ping_cm();
}

//Se realiza la media de todas las distancias. Los 0 se descartan
void MediaDistancias() {
  for (uint8_t i = 0; i < NSensores; i++) {
    uint16_t Total = 0;
    uint8_t Num = 0;
    for (uint8_t j = 0; j < NDistancias; j++) {
      if (Sensor[i].Distancias[j] != 0 && Sensor[i].Distancias[j] < 300) {
        Total += Sensor[i].Distancias[j];
        Num += 1;
      }
    }
    if (Num != 0) {
      Sensor[i].MediaDistancias = Total / Num;
    } else if (Num == 0) {
      Sensor[i].MediaDistancias = 0;
    }
  }
  /*Serial.print("\n\rDistancias: ");
  Serial.print(Sensor[4].MediaDistancias);
  Serial.print(",");
  Serial.print(Sensor[1].MediaDistancias);
  Serial.print(",");
  Serial.print(Sensor[2].MediaDistancias);
  Serial.print(",");
  Serial.print(Sensor[3].MediaDistancias);
  Serial.print("cm\n\r");*/
}

//Se comprueba si las media obtenida está por debajo del umbral.
void ComprobarDistancias() {
  //Mínimo de 5 para la distancia. existen errores de medida 
  for (uint8_t i = 0; i < NSensores; i++) {
    if (Sensor[i].MediaDistancias > 5 && Sensor[i].MediaDistancias < DistanciaCerca) {
      Sensor[i].Cerca = true;
    } else {
      Sensor[i].Cerca = false;
    }
  }
}

//========================MOVIMIENTO=========================//
uint16_t ComprobarPitch(uint16_t Pitch) {
  int16_t Diferencia = Sensor[0].MediaDistancias - Sensor[2].MediaDistancias;
  if( Sensor[4].MediaDistancias > AltMin || Sensor[4].MediaDistancias == 0 ) {
    if( abs(Diferencia) > DistMin ) {
    //Diferencia mayor de 20 entre distancias
    if( Sensor[0].Cerca == true ) {
      //Detecta el frontal
      if( Sensor[2].Cerca == true ) {
        //Detecta el trasero
        if( Sensor[0].MediaDistancias < Sensor[2].MediaDistancias ) {
          //El sensor frontal tiene una distancia menor
          Sensor[0].Activo = true;
          return( Pitch = ValorRC( Sensor[0].MediaDistancias, 1 ) );
        }else{
          //El sensor trasero tiene una distancia menor
          Sensor[2].Activo = true;
          return( Pitch = ValorRC( Sensor[2].MediaDistancias, 0 ) );
        }
      }else{
        //Detecta el frontal, pero no el trasero
        Sensor[0].Activo = true;
        return( Pitch = ValorRC( Sensor[0].MediaDistancias, 1 ) );
      }
    }else {
      //No detecta el frontal
      if( Sensor[2].Cerca == true ) {
        //Detecta el trasero
        Sensor[2].Activo = true;
        return( Pitch = ValorRC( Sensor[2].MediaDistancias, 0 ) );
      }else{
        //Ambos tienen una distancia mayor de 150
        return( Pitch = 0 );
      }
    }
  }else if( Sensor[0].Cerca == true && Sensor[2].MediaDistancias == 0 ) {
    //Detecta el de adelante, y el de detrás al no detectar nada, devuelve 0
    Sensor[0].Activo = true;
    return( Pitch = ValorRC( Sensor[0].MediaDistancias, 1 ) );
    }else if ( Sensor[0].MediaDistancias == 0 && Sensor[2].Cerca == true ) {
      //Lo mismo pero lo contrario
      Sensor[2].Activo = true;
      return( Pitch = ValorRC( Sensor[2].MediaDistancias, 0 ) );
      }else {
        //No detecta ninguno. Ambos a 0
        return( Pitch = 0 );
      }
  }else{
    return( Pitch = 0 );
  }
}

uint16_t ComprobarRoll(uint16_t Roll) {  
  int16_t Diferencia = Sensor[1].MediaDistancias - Sensor[3].MediaDistancias;
  if( Sensor[4].MediaDistancias > AltMin || Sensor[4].MediaDistancias == 0 ) {
    if( abs(Diferencia) > DistMin ) {
      //Diferencia mayor de 20 entre distancias
      if( Sensor[1].Cerca == true ) {
        //Detecta el derecho
        if( Sensor[3].Cerca == true ) {
          //Detecta el izquierdo
          if( Sensor[1].MediaDistancias < Sensor[3].MediaDistancias ) {
            //El sensor derecho tiene una distancia menor
            Sensor[1].Activo = true;
            return( Roll = ValorRC( Sensor[1].MediaDistancias, 0 ) );
          }else{
            //El sensor izquierdo tiene una distancia menor
            Sensor[3].Activo = true;
            return( Roll = ValorRC( Sensor[3].MediaDistancias, 1 ) );
          }
        }else{
          //Detecta el derecho, pero no el izquierdo
          Sensor[1].Activo = true;
          return( Roll = ValorRC( Sensor[1].MediaDistancias, 0 ) );
        }
      }else {
        //No detecta el derecho
        if( Sensor[3].Cerca == true ) {
          //Detecta el izquierdo
          Sensor[3].Activo = true;
          return( Roll = ValorRC( Sensor[3].MediaDistancias, 1 ) );
        }else{
          //Ambos tienen una distancia mayor de 150
          return( Roll = 0 );
        }
      }
    }else if( Sensor[1].Cerca == true && Sensor[3].MediaDistancias == 0 ) {
      //Detecta el derecho, y el izquierdo al no detectar nada, devuelve 0
      Sensor[1].Activo = true;
      return( Roll = ValorRC( Sensor[1].MediaDistancias, 0 ) );
      }else if ( Sensor[1].MediaDistancias == 0 && Sensor[3].Cerca == true ) {
        //Lo mismo pero lo contrario
        Sensor[3].Activo = true;
        return( Roll = ValorRC( Sensor[3].MediaDistancias, 1 ) );
        }else {
          //No detecta ninguno. Ambos a 0
          return( Roll = 0 );
        }
  }else {
    return( Roll = 0 );
  }
}

//Devuelve un valor de salida dependiendo de la distancia
//A mayor distancia, menor es la necesidad de movimiento. 
//La variable "Aumentar" es para saber en qué dirección es.
uint16_t ValorRC( uint16_t Distancia, bool Aumentar ) {
  if( Distancia < 30 ) {
    if( Aumentar == true ) {
      return( 1800 );
    }else{
      return( 1200 );
    }
  }else if( Distancia < 60 ) {
    if( Aumentar == true ) {
      return( 1750 );
    }else{
      return( 1250 );
    }
  }else if( Distancia < 90 ) {
    if( Aumentar == true ) {
      return( 1700 );
    }else{
      return( 1300 );
    }
  }else if( Distancia < 120 ) {
    if( Aumentar == true ) {
      return( 1650 );
    }else{
      return( 1350 );
    }
  }else{
    if( Aumentar == true ) {
      return( 1600 );
    }else{
      return( 1400 );
    }
  }
}

void CompensarInercia() {
  //Modifica el valor que se envia por RCOverride para compensar
  //la inercia producida por el movimiento anterior
  //Esta compensación se mantiene durante Compensación
  
  if( PitchOut == 0 ) {
    if( Sensor[0].Activo ) {
      //Pitch = 0 y Sensor Activo quiere decir que ha pasado de
      //moverse, a estar quieto. Hay que contrarrestar
      PitchOut = 1350;
      if(Sensor[0].CompensarTime == 0){
        Sensor[0].CompensarTime = millis();
      }
    }else if( Sensor[2].Activo == true ) {
      PitchOut = 1650;
      if(Sensor[2].CompensarTime == 0){
        Sensor[2].CompensarTime = millis();
      }
    }
  }
  if( RollOut == 0 ) {
    if( Sensor[1].Activo == true ) {
      RollOut = 1650;
      if(Sensor[1].CompensarTime == 0){
        Sensor[1].CompensarTime = millis();
      }
    }else if( Sensor[3].Activo == true ) {
      RollOut = 1350;
      if(Sensor[3].CompensarTime == 0){
        Sensor[3].CompensarTime = millis();
      }
    }
  }

  //Comprueba los 4 primeros sensores, si CompensarTime != 0, es que se ha modificado el valor para compensar inercia. Si pasan 100ms, sale.
  for(uint8_t i = 0; i < NSensores; i++){
    if(Sensor[i].CompensarTime != 0){
      if(millis()-Sensor[i].CompensarTime > Compensacion){
        Sensor[i].CompensarTime = 0;
        Sensor[i].Activo = false;  
      }
    }
  }
}

//============================MAVLINK==========================//
//Tarea encargada de enviar un HeartBeat cada segundo
void FHeartBeat() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len;
  // System ID = 255 = GCS
  mavlink_msg_heartbeat_pack(255, 0, &msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, 0, 1, 0);

  // Copy the message to send buffer
  len = mavlink_msg_to_send_buffer(buf, &msg);

  // Send the message (.write sends as bytes)
  Serial.write(buf, len);

  //Serial.write("\n\rHeartBeat\n\r");
}

void RCOverride(mavlink_message_t *msg, uint16_t len, uint8_t *buf, uint16_t PitchOut, uint16_t RollOut) {
  //Empaqueta y envía los datos de Pitch y Roll calculados. Sólo envia si el dato es nuevo
  mavlink_msg_rc_channels_override_pack(255, 0 , msg, 1, 0, RollOut, PitchOut, 0, 0, 0, 0, 0, 0);
  len = mavlink_msg_to_send_buffer(buf, msg);
  Serial.write(buf, len);
  /*Serial.print("\n\rPitch: ");
  Serial.print(PitchOut);
  Serial.print(",");
  Serial.print(" Roll: ");
  Serial.print(RollOut);*/
}

  /*//Armar Dron
    //Pack the message
    //uint16_t mavlink_msg_command_long_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,uint8_t target_system, uint8_t target_component, uint16_t command, uint8_t confirmation, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
    mavlink_msg_command_long_pack(255, 0, &msg, 1, 0, MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0);

    len = mavlink_msg_to_send_buffer(buf, &msg);

    // Send the message (.write sends as bytes)
    Serial.write(buf, len);
    delay(1000);*/

        /*mavlink_msg_rc_channels_override_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
    uint8_t target_system, uint8_t target_component, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw,
    uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw)*/
    
  /*Channel 1 = Roll
    Channel 2 = Pitch
    Channel 3 = Throttle
    Channel 4 = Yaw*/

  /*Sensor0 = Delantero
    Sensor1 = Derecha
    Sensor2 = Trasero
    Sensor3 = Izquierda*/
