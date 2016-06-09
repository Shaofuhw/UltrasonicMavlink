#include <NewPing.h>
#include "C:\Users\Fu\Documents\Arduino\libraries\mavlink\common\mavlink.h"

//=========================================INICIALIZACIONES======================================//
/*Inicialización de los pines de los sensores. Librería "NewPing"
  NewPing NAME(Trigger, Echo, MAXDIST);
  El valor de MAXDIST es la distancia máxima que mide la librería.
  Si algún Echo devuelve un valor mayor de dicha distancia, se descarta automáticamente*/
NewPing sonar0(24, 25, 300);
NewPing sonar1(26, 27, 300);
NewPing sonar2(28, 29, 300);
NewPing sonar3(30, 31, 300);

//Variable utilizada para controlar que el HeartBeat se envíe cada segundo
unsigned long HeartbeatTime = 0; 

//Variables utilizadas para que sólo envie un RCOverride cada vez que se modifique, y no saturar a la controladora de ordenes redundantes
uint16_t PitchOut;
uint16_t RollOut;
uint8_t n = 0;
bool enviado = false;

//Registro para guardar los datos de cada sensor
#define NDistancias 5
struct Sensores {
  uint16_t Distancias[NDistancias] = {0};
  uint16_t MediaDistancias = 0;
  bool Cerca = false;
};

//Se inician las variables de cada sensor
#define NSensores 4
Sensores Sensor[NSensores];

//====================================PROGRAMA===========================================================//

void setup() {
  Serial.begin(57600);
  //Activar pines 22 y 23 para alimentar a los ultrasonidos, ya que sólo se dispone de 3 salidas de 5v
  pinMode(22, OUTPUT);           // set pin to input
  digitalWrite(22, HIGH);       // turn on pullup resistors
  pinMode(23, OUTPUT);           // set pin to input
  digitalWrite(23, HIGH);       // turn on pullup resistors
}

void loop() {
  if ( (millis() - HeartbeatTime) > 1000 ) {
    HeartbeatTime = millis();
    FHeartBeat();
  }
  FSensores();
  FRCOverride();
  /*//Armar Dron
    //Pack the message
    //uint16_t mavlink_msg_command_long_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,uint8_t target_system, uint8_t target_component, uint16_t command, uint8_t confirmation, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
    mavlink_msg_command_long_pack(255, 0, &msg, 1, 0, MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0);

    len = mavlink_msg_to_send_buffer(buf, &msg);

    // Send the message (.write sends as bytes)
    Serial.write(buf, len);
    delay(1000);*/
}

//=======================================================FUNCIONES=======================================================//
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
    
  uint16_t Pitch = ComprobarPitch(Pitch);
  uint16_t Roll = ComprobarRoll(Roll);
  if( Pitch != PitchOut || Roll != RollOut) {
    //Sólo envía cuando cambia, se evita sobrecargar la controladora
    PitchOut = Pitch;
    RollOut = Roll;
    enviado = false;
  }else if(enviado == false) {
    n += 1;
    if(n == 3){
      //Se evita que se manden comandos distintos de manera consecutiva
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

//Se miden los sensores, y se colocan en la posición 0 de cada array
void MedirSensores() {
  Sensor[0].Distancias[0] = sonar0.ping_cm();
  Sensor[1].Distancias[0] = sonar1.ping_cm();
  Sensor[2].Distancias[0] = sonar2.ping_cm();
  Sensor[3].Distancias[0] = sonar3.ping_cm();
}

//Se realiza la media de todas las distancias. Los 0 se descartan, ya que son medidas que no se han podido realizar
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
  Serial.print(Sensor[0].MediaDistancias);
  Serial.print(",");
  Serial.print(Sensor[1].MediaDistancias);
  Serial.print(",");
  Serial.print(Sensor[2].MediaDistancias);
  Serial.print(",");
  Serial.print(Sensor[3].MediaDistancias);
  Serial.print("cm\n\r");*/
}

//Se comprueba si las media obtenida está por debajo del umbral. 150cm en este caso
void ComprobarDistancias() {
  //Se establece un mínimo de 5 para la distancia ya que existen ciertos errores de medida en dichos valores
  for (int i = 0; i < NSensores; i++) {
    if (Sensor[i].MediaDistancias > 5 && Sensor[i].MediaDistancias < 150) {
      Sensor[i].Cerca = true;
    } else {
      Sensor[i].Cerca = false;
    }
  }
}

//=======MAVLINK======//

uint16_t ComprobarPitch(uint16_t Pitch) {
  int16_t Diferencia = Sensor[0].MediaDistancias - Sensor[2].MediaDistancias;
  if( abs(Diferencia) > 20 ) {
    //Diferencia mayor de 20 entre distancias
    if( Sensor[0].Cerca == true ) {
      //Detecta el frontal
      if( Sensor[2].Cerca == true ) {
        //Detecta el trasero
        if( Sensor[0].MediaDistancias < Sensor[2].MediaDistancias ) {
          //El sensor frontal tiene una distancia menor
          return( Pitch = 1750 );
        }else{
          //El sensor trasero tiene una distancia menor
          return( Pitch = 1250 );
        }
      }else{
        //Detecta el frontal, pero no el trasero
        return( Pitch = 1750 );
      }
    }else {
      //No detecta el frontal
      if( Sensor[2].Cerca == true ) {
        //Detecta el trasero
        return( Pitch = 1250 );
      }else{
        //Ambos tienen una distancia mayor de 150
        return( Pitch = 0 );
      }
    }
  }else if( Sensor[0].Cerca == true && Sensor[2].MediaDistancias == 0 ) {
    //Detecta el de adelante, y el de detrás al no detectar nada, devuelve 0
    return( Pitch = 1750 );
    }else if ( Sensor[0].MediaDistancias == 0 && Sensor[2].Cerca == true ) {
      //Lo mismo pero lo contrario
      return( Pitch = 1250 );
      }else {
        //No detecta ninguno. Ambos a 0
        return( Pitch = 0 );
      }
}

uint16_t ComprobarRoll(uint16_t Roll) {  
  int16_t Diferencia = Sensor[1].MediaDistancias - Sensor[3].MediaDistancias;
  if( abs(Diferencia) > 20 ) {
    //Diferencia mayor de 20 entre distancias
    if( Sensor[1].Cerca == true ) {
      //Detecta el derecho
      if( Sensor[3].Cerca == true ) {
        //Detecta el izquierdo
        if( Sensor[1].MediaDistancias < Sensor[3].MediaDistancias ) {
          //El sensor derecho tiene una distancia menor
          return( Roll = 1250 );
        }else{
          //El sensor izquierdo tiene una distancia menor
          return( Roll = 1750 );
        }
      }else{
        //Detecta el derecho, pero no el izquierdo
        return( Roll = 1250 );
      }
    }else {
      //No detecta el derecho
      if( Sensor[3].Cerca == true ) {
        //Detecta el izquierdo
        return( Roll = 1750 );
      }else{
        //Ambos tienen una distancia mayor de 150
        return( Roll = 0 );
      }
    }
  }else if( Sensor[1].Cerca == true && Sensor[3].MediaDistancias == 0 ) {
    //Detecta el derecho, y el izquierdo al no detectar nada, devuelve 0
    return( Roll = 1250 );
    }else if ( Sensor[1].MediaDistancias == 0 && Sensor[3].Cerca == true ) {
      //Lo mismo pero lo contrario
      return( Roll = 1750 );
      }else {
        //No detecta ninguno. Ambos a 0
        return( Roll = 0 );
      }
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
