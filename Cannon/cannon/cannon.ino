
#include <EasyTransfer.h>

EasyTransfer ETin;
EasyTransfer ETout;

  //Receive data for EasyTransfer
struct RECEIVE_DATA_STRUCTURE{
    char chardata[4];
};

  //Send data for EasyTransfer
struct SEND_DATA_STRUCTURE{
    char chardata[4];
};

SEND_DATA_STRUCTURE dataReceive;
RECEIVE_DATA_STRUCTURE dataSend;

  // Actuator related pins
#define aFF2 6
#define aFF1 7
#define aPWM 8
#define aDIR 9
#define aPOS A8

  // Relay control pins
#define inflatorRelay 5
#define fireRelay 4
#define blowoffRelay 3

  // Sensor pins
#define pressureSen A9
#define proxSen A0

  // Angle constants
const int MIN_ANGLE = 10;
const int MAX_ANGLE = 80;

  // Transducer constants
const double MIN_PSI_READING = 200.0;
const double MAX_PSI_READING = 400.0;
const double PSI_READING_INTERVAL = MAX_PSI_READING - MIN_PSI_READING;
const double PSI_INTERVAL = 50.0;

int angle;
int aDirection;
bool set_angle;

bool armed;
bool safe;
bool previousSafe;
bool isNoPower;


void setup(){
    pinMode(aFF2, INPUT);      
    pinMode(aFF1, INPUT);
    pinMode(aPWM, OUTPUT);
    pinMode(aDIR, OUTPUT);
    pinMode(aPOS, INPUT);
    pinMode(fireRelay,OUTPUT);
    pinMode(blowoffRelay, OUTPUT);
    pinMode(inflatorRelay, OUTPUT);
    pinMode(pressureSen, INPUT);
    pinMode(proxSen, INPUT);
  
    Serial.begin(9600);
    Serial1.begin(9600);
    ETin.begin(details(dataReceive), &Serial1);                      // Using Serial 1 for easy transfer
    ETout.begin(details(dataSend), &Serial1);
    
    aDirection = 0;                                                   // Initialize actactor direction to 0 (retract)
    armed = false;                                                    // Initialize arm flag
    isNoPower = false;                                                // Initialize power flag
    previousSafe = true;                                              // Initialize safe flag
    set_angle = false;                                                // Initialize set_angle flag
  
      // Relay initialization
    digitalWrite(fireRelay, 1);                                       // Turn off all relays except the blow off
    digitalWrite(inflatorRelay, 1);
    digitalWrite(blowoffRelay, 0);
    
      // Setup timer1 interrupt
    noInterrupts();
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    OCR1A = 0;
  
    OCR1A = 65500;                                                    // Control timer time with 65536 being the maximum value (slowest)
    TCCR1B |= (1 << WGM12);
    TCCR1B |= (1 << CS12);
    TIMSK1 |= (1 << OCIE1A);
    interrupts();
       
      // Angle initialization
    digitalWrite(aDIR, 1);                                            // Initialize cannon to 10 degree
    analogWrite(aPWM, 255);
    angle = MIN_ANGLE;
    while(abs(analogRead(aPOS) - angleToReading(10)) > 1) {
      angleOperation();
    }
    analogWrite(aPWM, 0);
    digitalWrite(aDIR, aDirection);

    sendCommand("a" + String(angle));                                  // Send angle and pressure status to control box
    sendCommand("p" + String((int)round(getPsi())));
}


void loop(){
    serialProcess();                                                   // Check serial communication
    
    if(set_angle){                                                     // Adjust connan angle if set_angle flag is set
        angleOperation();
    }    
                                                                       
    if(getPsi() >= 51.0){                                              // Turn off blowoff solenoid and inflator when pressure reaches 51 psi until it is back to 45 psi
        digitalWrite(blowoffRelay, 1);
        digitalWrite(inflatorRelay, 1);
        while(getPsi() > 45.0);
        digitalWrite(blowoffRelay, 0);
    }
    safe = (getProximitySensorReading() < 50.0);                              // Check the proximity sensor
    
    if(!safe && previousSafe){                                         // Safe -> Not safe: Send signal to control box and turn off inflator
        sendCommand("f01");
        digitalWrite(inflatorRelay, 1);
    } else if(!previousSafe && safe){                                  // Not safe -> Safe: Send signal to control box
        sendCommand("f02");
    }
    previousSafe = safe;                                               // Save the current safety status
}


  // Timer 1 event
ISR(TIMER1_COMPA_vect){
    Serial.println("ANGLE: " + String(angle));                        // Display values to serial monitor
    Serial.println("PRESSURE: " + String(getPsi()));
    Serial.println("PROXIMITY: " + String(getProximitySensorReading()));
    Serial.println();
    
    if(previousSafe){                                                  // Continually send pressure value to control box if it is under safe mode
        if(isNoPower && getPsi() >= 0){                                // No power -> Power: Send signal to control box and reset POWER flag
            sendCommand("f03");
            isNoPower = false;
        }
        sendCommand("p" + String((int)round(getPsi())));
    }
}


  // Process message from control box
void serialProcess(){
    if(ETin.receiveData()){
        Serial.println(String(dataReceive.chardata));
      
        switch(dataReceive.chardata[0]){
              // Angle
            case 'a':
                if(!armed){
                    angle = (dataReceive.chardata[1] - 0x30) * 10 + (dataReceive.chardata[2] - 0x30);             // Set angle and set_angle flag
                    set_angle = true;
                }
                break;

              // Pressure
            case 'p':
                if(dataReceive.chardata[1] == 'u' && dataReceive.chardata[2] == 'h' && !armed){
                    digitalWrite(inflatorRelay, 0);                                                               // Turn on inflator and blowoff solenoid (Increasing pressure)
                    digitalWrite(blowoffRelay, 0);
                } else if(dataReceive.chardata[1] == 'd' && dataReceive.chardata[2] == 'h' && !armed){
                    digitalWrite(inflatorRelay, 1);                                                               // Turn off inflator and blowoff solenoid (Decreasing pressure)
                    digitalWrite(blowoffRelay, 1);
                } else if(dataReceive.chardata[2] == 'r' && !armed){
                    digitalWrite(inflatorRelay, 1);                                                               // Turn off inflator and turn on blowoff solenoid (Stay)
                    digitalWrite(blowoffRelay, 0);
                }
                break;
  
              // Fire code
            case 'f':
                if(dataReceive.chardata[1] == '3' && dataReceive.chardata[2] == '0'){
                    sendCommand("a" + String(angle));                                                             // Return angle, pressure and safety status to control box
                    sendCommand("p" + String((int)round(getPsi())));
                    sendCommand("f30");
                    if(!previousSafe){ 
                        sendCommand("f01"); 
                    }
                } else if(dataReceive.chardata[1] == '2' && dataReceive.chardata[2] == '0' && armed){   
                    digitalWrite(fireRelay, 0);                                                                   // Turn on the firing solenoid for 2 second and turn off inflator (Firing)
                    digitalWrite(inflatorRelay, 1);
                    delay(2000);
                    digitalWrite(fireRelay, 1);
                } else if(dataReceive.chardata[1] == '1' && dataReceive.chardata[2] == '0'){
                    armed = !armed;                                                                               // Arm and disarm the connan
                }
                break;
        }
        
    }
}

  
void angleOperation(){
    int requiredReading = angleToReading(angle);
    int reading = analogRead(aPOS);
    if(abs(reading - requiredReading) > 1){                                        // If not yet reach the target angle, activate the actuator
        aDirection = (reading - requiredReading) < 0 ? 1 : 0;                      // in the direction toward the target
        digitalWrite(aDIR, aDirection);
        analogWrite(aPWM, 100);
    } else {
        analogWrite(aPWM, 0);                                                     // If reach target, stop actuator and reset set_angle flag
        set_angle = false;
    }
}


  // Convert angle to the reading of actuator
int angleToReading(int angle){
    int ang = angle;
    if(angle <= MIN_ANGLE){ 
        ang = 10;
    }
    else if(angle >= MAX_ANGLE){ 
        ang = 80;
    }
    
    return (int) round(0.0005 * pow(ang, 3) - 0.0274 * pow(ang, 2) - 12.452 * ang + 961.19);      // Equation got from mapping the reading vs. angle
}


double getPsi(){
    double reading = 0;                                                           // Get 50 readings from the tranducer and take the average as the final reading
    for(int i = 0; i < 50; i++){
        reading += analogRead(pressureSen);
    }
    reading /= 50.0;
    
    if(reading <= MIN_PSI_READING){                                                 
        if(reading < MIN_PSI_READING - 20){                                        // Determine if there is no power to the transducer
            isNoPower = true; 
            return -1; 
        }
        return 0;
    }
    
    return (reading - MIN_PSI_READING) * PSI_INTERVAL / PSI_READING_INTERVAL;
}


double getProximitySensorReading(){
    double reading = 0;                                                           // Get 100 readings from the sensors and take the average as the final reading
    for(int i = 0; i < 100; i++){
        reading += analogRead(proxSen);
    }
    return reading /= 100.0;
}


void sendCommand(String command){                                                 // Send 4-character command to control box using easy transfer
    if(command[0] == 'p' && command.length() < 3){
        dataSend.chardata[0] = command[0];
        dataSend.chardata[1] = '0';
        dataSend.chardata[2] = command[1];
        dataSend.chardata[3] = 0;
    } else {
        dataSend.chardata[0] = command[0];
        dataSend.chardata[1] = command[1];
        dataSend.chardata[2] = command[2];
        dataSend.chardata[3] = 0;
    }
    ETout.sendData();
}


