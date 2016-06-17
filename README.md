# Control de Obstáculos Ultrasonidos
Trabajo fin de grado Ingeniería Electrónica y Automática

El siguiente código se ha realizado para una placa Arduino.
Utiliza la librería "NewPing" para medir la distancia de 5 sensores ultrasonidos HC-SR04.
4 Sensores para medir distancias laterales y uno para medir altura.
Si el sensor inferior mide más de 1.5 metros, empieza a actuar el control. Si alguno de los sensores laterales mide una distancia menor de 1.5 metros, manda un comando MAVLINK de RCOverride, el cual sobreescribe las entradas que recibe el APM de la controladora de vuelo.
