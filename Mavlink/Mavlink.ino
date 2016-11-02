#include <NewPing.h>
#include "C:\Users\shaof\OneDrive\Apuntes\4 Grado\2 Semestre\TFG\Arduino\libraries\mavlink\common\mavlink.h"

//==================================INICIALIZACIONES======================================//
/*Inicialización de los pines de los sensores. Librería "NewPing"
  NewPing NAME(Trigger, Echo, MAXDIST);
  El valor de MAXDIST es la distancia máxima que mide la librería.
  Si algún Echo devuelve un valor mayor de dicha distancia, se descarta automáticamente*/
NewPing sonar0(3, 4, 300);
NewPing sonar1(5, 6, 300);
NewPing sonar2(7, 8, 300);
NewPing sonar3(9, 10, 300);
NewPing sonar4(11, 12, 100);
 
//Variable utilizada para controlar que el HeartBeat se envíe cada segundo
unsigned long HeartbeatTime = 0; 

//Variables utilizadas para que sólo envie un RCOverride cada vez
//que se modifique, y no saturar a la controladora de ordenes redundantes
uint16_t Pitch = 0;
uint16_t Roll  = 0;
uint16_t PitchOut = 0;
uint16_t RollOut  = 0;
uint16_t PitchOutTemp = 0;
uint16_t RollOutTemp  = 0;
uint8_t n         = 0;

#define NDistancias     5
#define DistanciaCerca  100  //Distancia a la que empieza a actuar el control
#define AltMin          70 //Altura a la que empieza a actuar el control
#define DistMin         50 //Diferencia mínima entre dos distancias del mismo eje para moverse.
#define Compensacion    800 //Tiempo que actúa la compensación de inercia en ms

//Registro para guardar los datos de cada sensor
struct Sensores {
  uint16_t Distancias[NDistancias]  = {0};
  uint16_t MediaDistancias          = 0;
  bool Cerca                        = false;
  bool Activo                       = false;
  unsigned long CompensarTime       = 0;
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
  
  Pitch  = ComprobarPitch(Pitch);
  Roll   = ComprobarRoll(Roll);
  
  CompensacionInercia();

  if( Pitch != PitchOutTemp || Roll != RollOutTemp ){
    n = 0;
    PitchOutTemp = Pitch;
    RollOutTemp  = Roll;
  }else{
    n += 1;
    if(n == 4){
      RollOut = RollOutTemp;
      PitchOut = PitchOutTemp;
      RCOverride(&msg, len, buf, PitchOut, RollOut);
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
    int Total   = 0;
    uint8_t Num = 0;
    for (uint8_t j = 0; j < NDistancias; j++) {
      if (Sensor[i].Distancias[j] != 0  && Sensor[i].Distancias[j] < 300) {
        Total += Sensor[i].Distancias[j];
        Num += 1;
      }
    }
    if (Num > 3) {
      Sensor[i].MediaDistancias = Total / Num;
    } else {
      Sensor[i].MediaDistancias = 0;
    }
  }
  /*Serial.print("\n\rDistancias: ");
  Serial.print(Sensor[0].MediaDistancias);
  Serial.print(",");
  Serial.print(Sensor[1].MediaDistancias);
  Serial.print(",");
  Serial.print(Sensor[2].MediaDistancias);
  Serial.print(",");
  Serial.print(Sensor[3].MediaDistancias);
  Serial.print(",");
  Serial.print(Sensor[4].MediaDistancias);
  Serial.print("cm\n\r");*/
}

//Se comprueba si las media obtenida está por debajo del umbral.
void ComprobarDistancias() {
  //Mínimo de 10 para la distancia. existen errores de medida 
  for (uint8_t i = 0; i < NSensores; i++) {
    if (Sensor[i].MediaDistancias != 0 && Sensor[i].MediaDistancias < DistanciaCerca) {
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
    //Diferencia mayor de 30 entre ambos sensores
    if( Sensor[0].Cerca == true ) {
      //Detecta el frontal
      if( Sensor[2].Cerca == true ) {
        //Detecta el trasero
        if( Sensor[0].MediaDistancias < Sensor[2].MediaDistancias ) {
          //El sensor frontal tiene una distancia menor
          return( Pitch = ValorRC( Sensor[0].MediaDistancias, 1 ) );
        }else{
          //El sensor trasero tiene una distancia menor
          return( Pitch = ValorRC( Sensor[2].MediaDistancias, 0 ) );
        }
      }else{
        //Detecta el frontal, pero no el trasero
        return( Pitch = ValorRC( Sensor[0].MediaDistancias, 1 ) );
      }
    }else {
      //No detecta el frontal
      if( Sensor[2].Cerca == true ) {
        //Detecta el trasero
        return( Pitch = ValorRC( Sensor[2].MediaDistancias, 0 ) );
      }else{
        //Ambos tienen una distancia mayor de 150
        return( Pitch = 0 );
      }
    }
  }else if( Sensor[0].Cerca == true && Sensor[2].MediaDistancias == 0 ) {
    //Detecta el de adelante, y el de detrás al no detectar nada, devuelve 0
    return( Pitch = ValorRC( Sensor[0].MediaDistancias, 1 ) );
    }else if ( Sensor[0].MediaDistancias == 0 && Sensor[2].Cerca == true ) {
      //Lo mismo pero lo contrario
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
            return( Roll = ValorRC( Sensor[1].MediaDistancias, 0 ) );
          }else{
            //El sensor izquierdo tiene una distancia menor
            return( Roll = ValorRC( Sensor[3].MediaDistancias, 1 ) );
          }
        }else{
          //Detecta el derecho, pero no el izquierdo
          return( Roll = ValorRC( Sensor[1].MediaDistancias, 0 ) );
        }
      }else {
        //No detecta el derecho
        if( Sensor[3].Cerca == true ) {
          //Detecta el izquierdo
          return( Roll = ValorRC( Sensor[3].MediaDistancias, 1 ) );
        }else{
          //Ambos tienen una distancia mayor de 150
          return( Roll = 0 );
        }
      }
    }else if( Sensor[1].Cerca == true && Sensor[3].MediaDistancias == 0 ) {
      //Detecta el derecho, y el izquierdo al no detectar nada, devuelve 0
      return( Roll = ValorRC( Sensor[1].MediaDistancias, 0 ) );
      }else if ( Sensor[1].MediaDistancias == 0 && Sensor[3].Cerca == true ) {
        //Lo mismo pero lo contrario
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
      return( 1700 );
    }else{
      return( 1300 );
    }
  }else if( Distancia < 90 ) {
    if( Aumentar == true ) {
      return( 1675 );
    }else{
      return( 1325 );
    }
  }else if( Distancia < 150 ) {
    if( Aumentar == true ) {
      return( 1650 );
    }else{
      return( 1350 );
    }
  }
}

void CompensacionInercia(){

  if(PitchOut > 1500 && Sensor[0].Activo == false && Sensor[2].Activo == false){
    Sensor[0].Activo = true;
  }else if(PitchOut < 1500 && PitchOut != 0 && Sensor[2].Activo == false && Sensor[0].Activo == false){
    Sensor[2].Activo = true;
  }else if(PitchOut == 0 && Sensor[0].Activo == true && Sensor[0].CompensarTime == 0){
    Sensor[0].CompensarTime = millis();
  }else if(PitchOut == 0 && Sensor[2].Activo == true && Sensor[2].CompensarTime == 0){
    Sensor[2].CompensarTime = millis();
  }

  if(RollOut > 1500 && Sensor[3].Activo == false && Sensor[1].Activo == false){
    Sensor[3].Activo = true;
  }else if(RollOut < 1500 && RollOut != 0 && Sensor[1].Activo == false && Sensor[3].Activo == false){
    Sensor[1].Activo = true;
  }else if(RollOut == 0 && Sensor[1].Activo == true && Sensor[1].CompensarTime == 0){
    Sensor[1].CompensarTime = millis();
  }else if(RollOut == 0 && Sensor[3].Activo == true && Sensor[3].CompensarTime == 0){
    Sensor[3].CompensarTime = millis();
  }

  for(int i = 0; i < 4; i++){
    if(Sensor[i].CompensarTime != 0 && (Sensor[i].CompensarTime + Compensacion > millis())){
      switch(i){
        case 0:
          Pitch = 1300;
          break;
        case 1:
          Roll = 1700;
          break;
        case 2:
          Pitch = 1700;
          break;
        case 3:
          Roll = 1300;
          break;
        default:
          break;
      }
    }else if(Sensor[i].CompensarTime != 0){
      switch(i){
        case 0:
        case 2:
          PitchOut = 0;
          Sensor[i].Activo = false;
          Sensor[i].CompensarTime = 0;
          break;
        case 1:
        case 3:
          RollOut = 0;
          Sensor[i].Activo = false;
          Sensor[i].CompensarTime = 0;
          break;
        default:
          break;
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
