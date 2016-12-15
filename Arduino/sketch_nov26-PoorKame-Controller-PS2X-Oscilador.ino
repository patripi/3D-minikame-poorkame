// POORKAME
// Robot 3D diseño según MiniKame de Javier Isabel y version LowCost de OwenLab
// Programación original
//
// Madrid, Noviembre 2016                           PATRIPI
//
// ********************************************************
//
//  FORMATO: ternas separadas por '|'
//  inicio id con '*'
//  final id con '#'
//  Tipo Comando 'S' (stop emergencia) o 'R' (run normal)
//  ComandoServo = 0 al 7 (identifica al servo)
//  ComandoPosicion = 0 al 2 (identifica el ángulo a fijar)
//  Especiales: '88' pausa corta, '99' pausa larga
//
// ejemplo: *R|88|11|01|42|99|22|#
// ejemplo: *R|01|#
//
// ********************************************************
// BT Serial - BQ ZUM BT
//
// App Android en MIT App Inventor 2
// probado con AI Companion
//
// ********************************************************
// 26-nov-16: añado soporte control mando PS2X.
// Rutinas de movimientos completos. 
// ********************************************************
// 27-nov-16: añado soporte movimiento por miniKame de Javi
// con los osciladores (visto tutorial Obijuan octosnake).
// Rutinas de movimiento completos.
// ********************************************************
// 27-nov-16: añado en App Android lógica para acceder al
// soporte movimiento basado en osciladores.
// Rutinas de movimiento completos.
//
// ********************************************************
// Mejoras pendientes:
//    - gestión de errores
//

//  Mapeo de pines a servos:
//   _________   ________   _________
//  |(2)______)(0)      (1)(______(3)|
//  |__|       |   KAME   |       |__|
//             |          |
//             |          |
//             |          |
//   _________ |          | _________
//  |(6)______)(4)______(5)(______(7)|
//  |__|                          |__|
//                  /\
//                  |
//             USBs |

#include <Servo.h>
#include <miniKamePPF.h>
#include <PS2X_lib.h>

/*
  // ******** Soporte para accionamiento Manual con Ternas+BT+Android*****
  //SERVOS INTERIORES
  Servo servo_0;
  Servo servo_1;
  Servo servo_4;
  Servo servo_5;
  //SERVOS EXTERIORES
  Servo servo_2;
  Servo servo_3;
  Servo servo_6;
  Servo servo_7;

  Servo Servos[8] = {servo_0, servo_1, servo_2, servo_3, servo_4, servo_5, servo_6, servo_7};

  // Define Message variables
  #define START_MSJ_CHAR '*'  //inicio
  #define END_MSJ_CHAR '#'    //fin
  #define DIV_CMD_CHAR '|'    //separador, sólo para facilitar lectura de la secuencia
  #define CMD_RUN 'R'         //ejecuta la secuencia almacenada
  #define CMD_STOP 'S'        //stop, interrumpe ejecución

  const int ANG_MIN_INT = 30;  // min angular servos interiores Ref: 30 grados
  const int ANG_MID_INT = 65;  // mid angular servos interiores Ref: 82 grados
  const int ANG_MAX_INT = 100;  // max angular servos interiores Ref: 135 grados
  const int ANG_MIN_EXT = 30;  // min angular servos exteriores Ref: 30 grados
  const int ANG_MID_EXT = 65;  // mid angular servos exteriores Ref: 60 grados
  const int ANG_MAX_EXT = 100;  // max angular servos exteriores Ref: 100 grados

  #define DELAY_SHORT 500       //pausa corta
  #define DELAY_LONG 1000       //pausa larga

  // Rellenar Matriz que define qué ángulos le corresponde a cada posición 0,1,2 segun el servo
  const int Posiciones[8][3] = {
  {ANG_MAX_INT, ANG_MID_INT, ANG_MIN_INT}, //SERVO 0 creciente C
  {ANG_MIN_INT, ANG_MID_INT, ANG_MAX_INT}, //SERVO 1 decreciente D
  {ANG_MIN_EXT, ANG_MID_EXT, ANG_MAX_EXT}, //SERVO 2 creciente C
  {ANG_MAX_EXT, ANG_MID_EXT, ANG_MIN_EXT}, //SERVO 3 decreciente D
  {ANG_MIN_INT, ANG_MID_INT, ANG_MAX_INT}, //SERVO 4 decreciente D
  {ANG_MAX_INT, ANG_MID_INT, ANG_MIN_INT}, //SERVO 5 creciente C
  {ANG_MAX_EXT, ANG_MID_EXT, ANG_MIN_EXT}, //SERVO 6 decreciente D
  {ANG_MIN_EXT, ANG_MID_EXT, ANG_MAX_EXT}  //SERVO 7 creciente C
  };

  #define iLongitudMensaje 200   // Numero de Comandos capaz de recibir +1
  String  sMensaje;              // Cadena recibida
  char cMensaje[iLongitudMensaje]; // MATRIZ para procesar los comandos
*/

// ***** Soporte control por mando RF PS2X ****
  PS2X ps2x;
  byte vibrate = 0;
  // ********************************************


// **** Soporte movimientos con octosnake ****
miniKamePPF robot;
bool running = 0;
// *******************************************

void setup() {
  /*// ******** Soporte para accionamiento Manual con Ternas+BT+Android*****
    // Definir attach de Servos a Pines
    // evitamos el Pin 0 y el 1 (RX TX)
    //SERVOS INTERIORES
    servo_0.attach(10);
    servo_1.attach(11);
    servo_4.attach(4);
    servo_5.attach(5);
    //SERVOS EXTERIORES
    servo_2.attach(2);
    servo_3.attach(3);
    servo_6.attach(6);
    servo_7.attach(7);

    //Inicializar variables de mensaje
    sMensaje = "*R|01|11|21|31|41|51|61|71|#";
    RellenarArrayMensaje(sMensaje);
    EjecutarComandos();
  */

  //Inicializar Comunicacion BT
  Serial.begin(19200);        // Con la BT ZUM son 19200 baudios
  //Serial.begin(9600);           // Para Debugg con Serial Monitor
  Serial.println("Comenzamos"); // for debug

  // ******* Inicializar mando PS2X
  int error = ps2x.config_gamepad(A5, A3, A2, A4, false, false); //(clock, command, attention, data)
  
  /*if (error == 0) {
    Serial.println("Found Controller, configured successful");
    Serial.println("Try out all the buttons, X will vibrate the controller, faster as you press harder;");
    Serial.println("holding L1 or R1 will print out the analog stick values.");
    Serial.println("Go to www.billporter.info for updates and to report bugs.");
    }
    else if (error == 1)
    Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");
    else if (error == 2)
    Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");
    else if (error == 3)
    Serial.println("Controller refusing to enter Pressures mode, may not support it. ");
  */
  /* PPF: Atención! si descomento este if...then... anterior, se incementa en un +20pp el consumo de memoria dinámica*/

  switch (ps2x.readType()) {
    case 0:
    Serial.println("Unknown Controller type");
    break;
    case 1:
    Serial.println("DualShock Controller Found");
    break;
    case 2:
    Serial.println("GuitarHero Controller Found");
    break;
    }
  

  // ******* Inicializar miniKame ***********************************
  robot.init();
  robot.hello();

}

void loop() {

  // ***** Control Comandos por mando PS2X **************************
  ps2x.read_gamepad(false, vibrate);

    // Controlamos los botones izq del gamepad:
    if (ps2x.Button(PSB_PAD_UP))
    {
    Serial.println("pulsado ps2x.Button(PSB_PAD_UP)");
    robot.walk(1, 550);
    running = 1;
    }
    else if (ps2x.Button(PSB_PAD_RIGHT))
    {
    Serial.println("pulsado ps2x.Button(PSB_PAD_RIGHT)");
    robot.turnR(1, 550);
    running = 1;
    }
    else if (ps2x.Button(PSB_PAD_LEFT))
    {
    Serial.println("pulsado ps2x.Button(PSB_PAD_LEFT)");
    robot.turnL(1, 550);
    running = 1;
    }
    else if (ps2x.Button(PSB_PAD_DOWN))
    {
    Serial.println("pulsado ps2x.Button(PSB_PAD_DOWN)");
    robot.frontBack(4, 200);
    }

    // Controlamos boton de select y start:
    if (ps2x.Button(PSB_START))                  //will be TRUE as long as button is pressed
    Serial.println("pulsado ps2x.Button(PSB_START)");
    if (ps2x.Button(PSB_SELECT)) {
    Serial.println("pulsado ps2x.Button(PSB_SELECT)");
    robot.home();
    }
    // Controlamos los botones dch del gamepad:
    if (ps2x.Button(PSB_GREEN)) {
    Serial.println("pulsado ps2x.Button(PSB_GREEN)");
    robot.hello();
    }
    if (ps2x.Button(PSB_RED)) {
    Serial.println("pulsado ps2x.Button(PSB_RED)");
    robot.pushUp(2, 5000); 
    }
    if (ps2x.Button(PSB_PINK))   {
    Serial.println("pulsado ps2x.Button(PSB_PINK)");
    robot.upDown(4, 250);
    }
    if (ps2x.Button(PSB_BLUE))        {
    Serial.println("pulsado ps2x.Button(PSB_BLUE)");
    robot.dance(2, 1000);
    }
    // Controlamos los botones delanteros del gamepad:
    if (ps2x.Button(PSB_L1)) {
    Serial.println("pulsado ps2x.Button(PSB_L1)");
    robot.run(4, 500); //era 2000
    }
    if (ps2x.Button(PSB_R1)) {
    Serial.println("pulsado ps2x.Button(PSB_R1)"); // AQUI IRÁ EL ONMIWALK
    // omniWalk(float steps, float T, bool side, float turn_factor)
    robot.omniWalk(4, 1000, 1, 20);
    }
    if (ps2x.Button(PSB_L2)) {
    Serial.println("pulsado ps2x.Button(PSB_L2)");
    robot.jump();
    }
    if (ps2x.Button(PSB_R2)) {
    Serial.println("pulsado ps2x.Button(PSB_R2)");// AQUI IRÁ EL MOONWALK
    robot.moonwalkL(4, 2000);
    }
    if (ps2x.Button(PSB_L3))
    Serial.println("pulsado ps2x.Button(PSB_L3)");
    if (ps2x.Button(PSB_R3))
    Serial.println("pulsado ps2x.Button(PSB_R3)");

    /* MAS EJEMPLOS DE USO PS2X:
      ps2x.ButtonPressed(PSB_RED)) {         //will be TRUE if button was JUST pressed
      ps2x.ButtonReleased(PSB_PINK))   {         //will be TRUE if button was JUST released
      ps2x.NewButtonState(PSB_BLUE))        {   //will be TRUE if button was JUST pressed OR released

      vibrate = ps2x.Analog(PSAB_BLUE);        //this will set the large motor vibrate speed based on
                                              //how hard you press the blue (X) button
      if (ps2x.NewButtonState())               //will be TRUE if any button changes state (on to off, or off to on)
    Controlamos los valores de los joysticks:
    if (ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1)) // print stick values if either is TRUE
    {
      Serial.print("Stick Values:");
      Serial.print(ps2x.Analog(PSS_LY), DEC); //Left stick, Y axis. Other options: LX, RY, RX
      Serial.print(",");
      Serial.print(ps2x.Analog(PSS_LX), DEC);
      Serial.print(",");
      Serial.print(ps2x.Analog(PSS_RY), DEC);
      Serial.print(",");
      Serial.println(ps2x.Analog(PSS_RX), DEC);
    }
      delay(50);    //hay que meter un delay
    // ******** fin soporte PS2 ***********************
  */
  /*
    // ******** Soporte para accionamiento Manual con Ternas+BT+Android*****
    // 1º Conseguir el mensaje
    sMensaje = leerMensaje ();
    TrazasAuxilizaresComm(sMensaje);

    //2º Ejecutar los comandos
    if (sMensaje != "") {
      EjecutarComandos();
    }
  */

  // ***** Control Rutinas App Android   **************************
 // EjecutarRutina();
  delay(50);

}

//Ejecución del mensaje recibido, enviando las órdenes a los servomotores
void EjecutarRutina()
{

  if (Serial.available() > 0)        //verificar si hay nuevos mensajes
  {
    char inChar = Serial.read();
    if ( inChar == 'A') {
      Serial.println("Ejecuto RUT_WALK");
      robot.walk(1, 550);
      running = 1;
      return;
    } else if (inChar == 'B') {
      Serial.println("Ejecuto RUT_TURN_R");
      robot.turnR(1, 550);
      running = 1;
      return;
    } else if (inChar == 'C') {
      Serial.println("Ejecuto RUT_TURN_L");
      robot.turnL(1, 550);
      running = 1;
      return;
    } else if (inChar == 'D') {
      Serial.println("Ejecuto RUT_FRONT_BACK");
      robot.frontBack(4, 200);
      return;
    } else if (inChar== 'E') {
      Serial.println("Ejecuto RUT_HOME");
      robot.home();
      return;
    } else if (inChar == 'F') {
      Serial.println("Ejecuto RUT_HELLO");
      robot.hello();
      return;
    } else if (inChar == 'G') {
      Serial.println("Ejecuto RUT_PUSH_UP");
      robot.pushUp(2, 5000);
      return;
    } else if (inChar == 'H') {
      Serial.println("Ejecuto RUT_UP_DOWN");
      robot.upDown(4, 250);
      return;
    } else if (inChar== 'I') {
      Serial.println("Ejecuto RUT_DANCE");
      robot.dance(2, 1000);
      return;
    } else if (inChar== 'J') {
      Serial.println("Ejecuto RUT_RUN");
      robot.run(5, 2000);
      return;
    } else if (inChar == 'K') {
      Serial.println("Ejecuto RUT_JUMP");
      robot.jump();
      return;
    } else if (inChar == 'L') {
      Serial.println("Ejecuto RUT_OMNI_WALK");
      robot.omniWalk(4, 1000, 1, 20);
      return;
    } else if (inChar == 'M') {
      Serial.println("Ejecuto RUT_MOON_WALK");
      robot.moonwalkL(4, 2000);
      return;
    } else
    {
      // NO DEBERIA PASAR POR AQUI
      Serial.println("COMANDO CON FORMATO DE MENSAJE INCORRECTO - RETURN");
      return;
    }

  }
}


/*//Auxiliar para relleno de Array a partir de la String de Mensaje
  void RellenarArrayMensaje(String sMensaje) {

  // primero limpio
  for (int j = 0; j < iLongitudMensaje; j++) {
    cMensaje[j] = "";
  }
  //relleno
  sMensaje.toCharArray(cMensaje, 200);

  }
*/
/*
  //Ejecución del mensaje recibido, enviando las órdenes a los servomotores
  void EjecutarComandos()
  {
  char TipoComando;
  char ComandoServo;
  char ComandoPosicion;
  String ComandoDoble;

  // comienza a recorrer la array del mensaje, identificando pares
  for (int i = 0; i < iLongitudMensaje + 1; i = i + 3) {

    ImprimirTerna("FOR ", i, cMensaje[i], cMensaje[i + 1], cMensaje[i + 2]); //traza

    TipoComando = "";
    ComandoServo = "";
    ComandoPosicion = "";
    ComandoDoble = "";

    //1º TRATAMOS SI ES EL INICIO DEL MENSAJE
    if (cMensaje[i] == START_MSJ_CHAR) {
      TipoComando = cMensaje[i + 1];
      if (TipoComando == CMD_STOP)
      {
        // ORDEN STOP - Parada Emergencia
        Serial.println("COMANDO PARADA DE EMERGENCIA!!");
        //PararEmergencia();
        return;
      } else if (TipoComando == CMD_RUN) {
        // ORDEN RUN - pasamos a la siguiente terna
        Serial.println("COMANDO RUN - PASO A LA SIGUIENTE TERNA");
      } else
      {
        // NO DEBERIA PASAR POR AQUI
        Serial.println("COMANDO CON FORMATO DE MENSAJE INCORRECTO - RETURN");
        return;
      }
    }
    //2º TRATAMOS SI SON COMANDOS DE ACCION O PAUSA
    else if (cMensaje[i] != END_MSJ_CHAR) {
      ComandoServo = cMensaje[i];
      ComandoPosicion = cMensaje[i + 1];
      ComandoDoble = String(cMensaje[i]) + String(cMensaje[i + 1]);
      if (ComandoDoble == "88") //2a ES UNA PAUSA
      {
        delay(DELAY_SHORT); // DELAY_SHORT - PAUSA CORTA
        Serial.println("Ejecuto PAUSA corta y paso a siguiente terna");
      } else if ( ComandoDoble == "99")
      {
        delay(DELAY_LONG);  // DELAY_LONG - PAUSA LARGA
        Serial.println("Ejecuto PAUSA larga y paso a siguiente terna");
      }
      else if (ComandoServo == '0' || ComandoServo == '1' || ComandoServo == '2' || ComandoServo == '3' || ComandoServo == '4' || ComandoServo == '5' || ComandoServo == '6' || ComandoServo == '7') { //2b ES UNA ACCION DE MOTOR
        int angulo = Posiciones[(ComandoServo - '0')][(ComandoPosicion - '0')];
        Servos[ComandoServo - '0'].write(angulo);
        //  ImprimirComandoMotor (ComandoServo, ComandoPosicion); //traza
      } else {    //2c NO es un comando para Servo
        Serial.println("COMANDO RARO: no es pausa ni orden para motor! - RETURN");
        return;
      }
    }
    //3º TRATAMOS SI ES EL FINAL DEL MENSAJE
    else if (cMensaje[i] == END_MSJ_CHAR) {
      Serial.println("Final de Ejecución #");
      return;
    }
  }
  return;
  }
*/
/*
  // Método para volcar el mensaje recibido por Serial, vuelca a string y a array
  String leerMensaje()
  {
  byte numBytesAvailable = Serial.available();
  char inChar = -1;
  String msj = "";          //limpiamos string mensaje LOCAL
  int j;                    //limpiamos array de char GLOBAL
  for (j = 0; j < iLongitudMensaje; j++) {
    cMensaje[j] = "";
  }
  int i = 0;
  if (numBytesAvailable > 0)        //verificar si hay nuevos mensajes
  {
    while (numBytesAvailable > 0 ) //mientras el mensaje no termine
    {
      inChar = Serial.read();          //leemos char
      msj += inChar;                   //RELLENAMOS LA STRING LOCAL QUE RETORNAREMOS
      cMensaje[i] = inChar;            //RELLENAMOS LA ARRAY GLOBAL
      i++;
      delay(10);                       //esperamos el siguiente char

      if (inChar == END_MSJ_CHAR) {    //terminamos
        delay (2000);
        return msj;
      }
    }
  }
  else
    return "";                        //return "" si no hay mensaje;
  }
*/
/*

  //Traza auxiliar muestra la cadena que está recibiendo Arduino por el Serial
  void TrazasAuxilizaresComm(String msj)
  {
  if (msj != "") {
  Serial.print("Traza Auxiliar recibido la array 3: ");
  Serial.println(cMensaje);
  }
  }
  //Traza auxiliar de la terna obtenida
  void ImprimirTerna (String Comando, int i, char a, char b, char c) {
  Serial.print("Terna Comando : ");
  Serial.print(Comando);
  Serial.print(" ");
  Serial.print(i);
  Serial.print(" posicion 1: ");
  Serial.print(a);
  Serial.print(" posicion 2: ");
  Serial.print(b);
  Serial.print(" posicion 3: ");
  Serial.println(c);
  }
  //Traza auxiliar del comando enviado a servomotor
  void ImprimirComandoMotor (char ComandoServo, char ComandoPosicion) {
  Serial.print("MUEVO MOTOR: ");
  Serial.print(ComandoServo - '0');
  Serial.print(" a la posicion: ");
  Serial.print(ComandoPosicion - '0');
  Serial.print(" d.h. un angulo: ");
  Serial.println(Posiciones[ComandoServo - '0'][ComandoPosicion - '0']);
  }
*/


