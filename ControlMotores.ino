#include <OrangutanLCD.h>
#include <PololuQTRSensors.h>
#include <Pololu3pi.h>

OrangutanLCD lcd;
Pololu3pi robot;


unsigned int sensors[5]; // an array to hold sensor values


//variables
int BtonA = 9;
int BtonB = 12;
int BtonC = 13;

//parametros generales
int velocidad;
int setPoint = 2000;
int Ki;                    //Gancancia proporcional
int Kp;                    //Ganancia Integral
unsigned int error;
void setup() {
  
robot.init(2000);
OrangutanLCD::clear();
lcd.gotoXY(0,0);
lcd.print("Press C para");
lcd.gotoXY(0,1);
lcd.print("Calibrar");
while(digitalRead(BtonC)) ; 

lcd.clear();
lcd.print("Calibrando");  
 for (int i = 0; i < 200; i++)  // make the calibration take about 10 seconds
  {  
    robot.calibrateLineSensors(IR_EMITTERS_ON);
  }  

lcd.clear();  
lcd.print("Terminado");  
Menu();
}


void loop() {
    
 }
    
    
///******************************************************////   
void controlPI(){
  unsigned int proporcional;
  unsigned int integral;
  unsigned int errorAnterior;
  unsigned int control;
  lcd.clear();
  lcd.gotoXY(0,0);
  lcd.print("inicio PI");
  while(digitalRead(BtonC)){
  unsigned int posicionRobot = sensores();
  
  error = setPoint - posicionRobot;
  proporcional = Kp*error;
  integral = Ki*(error + errorAnterior);

  control = proporcional + integral;
  
  motores(velocidad + control, velocidad - control);      //La velocidad inicial la destina el usuario. NO tiene limite en la potencia sobre los motores, si supera 255 entonces
                                                          // potencia se reincia...
  
  errorAnterior = error;
  }//ejecucion del controlador
  
  }//Fin control PI
///*******************************************************////  
void controlOnOff(){
  lcd.clear();
  lcd.gotoXY(0,0);
  lcd.print("inicio OnOff");
  while(digitalRead(BtonC)){
  unsigned int posicionRobot = sensores();
   if (posicionRobot > setPoint){ 
      motores(velocidad,0);
    }
   if (posicionRobot < setPoint){
     motores(0,velocidad);
   }
  }
  delay(300);
  motores(0,0);
  lcd.gotoXY(0,0);
  lcd.print("fin control OnOff");
  while(digitalRead(BtonC));  
  delay(300);
}


//Funciones para configurar los parametros de cada controlador
void parametrosONOFF(){
    //seleccionar la velocidad. 
    lcd.clear();                  
    lcd.gotoXY(4, 0);             // parte superior
    lcd.print("On - Off");  

while(digitalRead(BtonC)){    //con el Boton C grabamos la velocidad
  if (!digitalRead(BtonA)){
    delay(200);
    velocidad++;
    lcd.gotoXY(0, 1);        
    lcd.print("                ");        
    }
  if (!digitalRead(BtonB)){
    delay(200);
    velocidad--;
    lcd.gotoXY(0, 1);        
    lcd.print("                ");
    }

    lcd.gotoXY(0, 1); 
   
   if ((velocidad >255)) {velocidad = 0;}
   if ((velocidad <0)) {velocidad = 255;}

    lcd.gotoXY(0, 1);             // go to the fourth character of the second LCD line
    lcd.print("Velocidad: ");  
    lcd.print(velocidad);  

}// grabada la velocidad
    lcd.gotoXY(0, 1);             // go to the fourth character of the second LCD line
    lcd.print("Press C to Start");  
    while(digitalRead(BtonC)) ;   //espera a que presion el Botn C para iniciar
    delay(300);
    controlOnOff();
}// Parametros control On Off  

void  parametrosHISTERESIS(){

}




//Funcion cargar parametros PI
void parametrosPI(){
    //seleccionar la Kp. 
    lcd.clear();                  
    lcd.gotoXY(2, 0);             // parte superior
    lcd.print("Control PI");  

while(digitalRead(BtonC)){    //con el Boton C grabamos la velocidad
  if (!digitalRead(BtonA)){
    delay(200);
    velocidad++;
    lcd.gotoXY(0, 1);        
    lcd.print("                ");        
    }
  if (!digitalRead(BtonB)){
    delay(200);
    velocidad--;
    lcd.gotoXY(0, 1);        
    lcd.print("                ");
    }

    lcd.gotoXY(0, 1); 
   
   if ((velocidad >255)) {velocidad = 0;}
   if ((velocidad <0)) {velocidad = 255;}

    lcd.gotoXY(0, 1);             // go to the fourth character of the second LCD line
    lcd.print("Velocidad: ");  
    lcd.print(velocidad);  

}// grabada la velocidad
delay(1000);
lcd.clear();
lcd.gotoXY(9, 1);     
lcd.print("Ki:    ");  

while(digitalRead(BtonC)){    //con el Boton C grabamos Kp
  if (!digitalRead(BtonA)){
    delay(200);
    Kp++;


    }
  if (!digitalRead(BtonB)){
    delay(200);
    Kp--;


    }

  
   if ((Kp >255)) {Kp = 0;}
   if ((Kp <0)) {Kp = 255;}

    lcd.gotoXY(0, 1);             // go to the fourth character of the second LCD line
    lcd.print("Kp:    ");
    lcd.gotoXY(5, 1);  
    lcd.print(Kp);  

}// grabada Kp

delay(350);
lcd.gotoXY(9, 1);     
lcd.print("Ki:    ");  
    
while(digitalRead(BtonC)){    //con el Boton C grabamos Ki
  if (!digitalRead(BtonA)){
    delay(200);
    Ki++;
    }
  if (!digitalRead(BtonB)){
    delay(200);
    Ki--;
    }
   
   if ((Ki >255)) {Ki = 0;}
   if ((Ki <0)) {Ki = 255;}

    lcd.gotoXY(9, 1);     
    lcd.print("Ki:    ");  
    lcd.gotoXY(13, 1);
    lcd.print(Ki);  

}// grabada Ki
delay(350);

lcd.clear();
    lcd.gotoXY(0, 1);             // go to the fourth character of the second LCD line
    lcd.print("Press C to Start");  
    while(digitalRead(BtonC)) ;   //espera a que presion el Botn C para iniciar
    delay(300);
    controlPI();

}//Paramteros PI



void parametrosPIWindUp(){

}
void parametrosPIFiltro(){

}
//Fin Fnciones para configurar los parametros de cada controlador

//Esta funcion permite seleccionar el tipo de controlador que deseamos probar.
void Menu(){
int controlador = 1;  
lcd.clear();             
lcd.gotoXY(1, 0);   
lcd.print("Select Control");

while(digitalRead(BtonC)){    //con el Boton C grabamos el tipo de controlador
  
  if (!digitalRead(BtonA)){
    delay(300);
    controlador++;
    lcd.gotoXY(0, 1);        
    lcd.print("                ");        
    }
  if (!digitalRead(BtonB)){
    delay(300);
    controlador--;
    lcd.gotoXY(0, 1);        
    lcd.print("                ");
    }

    lcd.gotoXY(0, 1); 
    switch(controlador) {
      case 1:
          lcd.print("On/Off");          
          break;
      case 2:
          lcd.print("On/Off + Histeresis");          
          break;
      case 3:
          lcd.print("PI");          
          break;
      case 4:
          lcd.print("PI + WindUp");          
          break;
      case 5:
          lcd.print("PI + Filtro");          
          break;          
      }
      if ((controlador >5)) {controlador = 1;}
      if ((controlador <1)) {controlador = 5;}
    
  
  }//whiule bton C
lcd.clear();                  // clear the LCD
lcd.gotoXY(0, 0);             // go to the fourth character of the second LCD line
lcd.print("Controlador");  
lcd.gotoXY(0, 1);             // go to the fourth character of the second LCD line
lcd.print("Seleccionado: ");  
lcd.print(controlador);  
delay(3000);

 switch(controlador) {
      case 1:
          parametrosONOFF();
          break;
      case 2:
          parametrosHISTERESIS();
          break;
      case 3:
          parametrosPI();   
          break;
      case 4:
          parametrosPIWindUp();
          break;
      case 5:
          parametrosPIFiltro();
          break;          
   }

}//Menu

//Funcion para enviar el sentido de giro al robot.
void motores(int vel_Iz, int vel_Der){

  if (vel_Iz > 0 ){
      analogWrite(6,vel_Iz);
      analogWrite(5,0);                  // El motor eta controlado por los puero analos 5 y 6. Cuando vel es positivo gira en un saentido
    }else{
      analogWrite(5,-vel_Iz);
      analogWrite(6,0);                  // El motor eta controlado por los puero analos 5 y 6. Cuando vel es positivo gira en otro saentido
      }
    if (vel_Der > 0 ){
      analogWrite(11,vel_Der);
      analogWrite(3,0);                  // El motor eta controlado por los puero analos 3 y 11. Cuando vel es positivo gira en un saentido
    }else{
      analogWrite(3,-vel_Der);
      analogWrite(11,0);                  // El motor eta controlado por los puero analos 3 y 11. Cuando vel es positivo gira en otro saentido
      }
 }// Motores
 
 
 //Lectura de los sensores
unsigned int  sensores(){
   unsigned int posRobot = robot.readLine(sensors, IR_EMITTERS_ON);
   return(posRobot)    ;
   }
