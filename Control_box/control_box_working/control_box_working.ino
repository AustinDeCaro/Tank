#include <LiquidCrystal.h>
#include <EasyTransfer.h>

EasyTransfer ETin, ETout;

struct SEND_DATA_STRUCTURE{
  char chardata[4];
};

struct RECEIVE_DATA_STRUCTURE{
  char chardata[4];
};

SEND_DATA_STRUCTURE dataSend;
RECEIVE_DATA_STRUCTURE dataReceive;

#define GND 40  // A pin set to LOW to act as gnd for the LCD display
#define RS	24	// RS pin for J204A LCD display
#define E  	22	// E pin for J204A LCD display
#define D4  36	// Data pins for J204A
#define D5  34	
#define D6  32
#define D7  30
#define LL  50  // LCD Backlight pin
#define BL  51  // Button Light pin 
#define AUP  25	// Angle up pin. Left joystick
#define ADW  23	// Angle down pin. Left joystick
#define PUP	 29	// Pressure up pin. Right joystick
#define PDW	 27	// Pressure down pin. Right joystick
#define ARM  2	// Arm/Disarm toggle switch pin
#define FIR  3	// Fire pin. BIG RED BUTTON

#define MAX_ANGLE 80
#define MIN_ANGLE 10
#define MAX_PRESSURE 51
#define MIN_PRESSURE -1

int nopow = 0;              // Flag for when the cannon is without power. It gets set when we recive "p-1"
int puheld = 0;             // Pressure up held flag.
int pdheld = 0;             // Pressure down held flag.
int joystickupheld = 0;     // Angle up held flag.
int joystickdwheld = 0;     // Angle down held flag.
int fired = 0;              // flag that gets set when you press the fire button. It's so that you can't fire twice without disarming the cannon first.
int ignoreFlag = 0;					// flag is set if the cannon is locked. Ignore all interrupt input.
int CannonReadyFlag = 0;			// Is the Cannon ready to accept commands
int safe = 1;						// mode of the cannon for now only 1 or 0
int angle = -1;						// cannon angle. Initialized to -1 so it doesn't display at the start.
int pressure = -1;					// cannon pressure. Same as above.
char* command;						// pointer to the string we will get over serial and interpreted as a command
long debouncing_time = 100;			// I think it's in milisec. If interrupt is triggering more than once increase this value
volatile unsigned long last_micros = 1;

LiquidCrystal lcd(RS, E, D4, D5, D6, D7);		// Creates an lcd object so that we can use the LiquidCrystal library

void setup() 
{
  pinMode(44, HIGH);
  pinMode(LL, OUTPUT);
  pinMode(BL, OUTPUT);
  pinMode(GND, OUTPUT);
  digitalWrite(44, HIGH);
  digitalWrite(GND, LOW);
  digitalWrite(LL, HIGH);
  digitalWrite(BL, HIGH);
  lcd.begin(20, 4);                               // 20, 4 is our lcd size
	displayMenu();														      // Initialize the lcd to display the Menu without values
	pinMode(AUP, INPUT_PULLUP);											// Making the interrupt pin INPUT_PULLUP makes it less sensitive to noise, but switches the logic. 
	pinMode(ADW, INPUT_PULLUP);											// Making the interrupt pin INPUT_PULLUP makes it less sensitive to noise, but switches the logic.
	pinMode(PUP, INPUT_PULLUP);											// Making the interrupt pin INPUT_PULLUP makes it less sensitive to noise, but switches the logic.
	pinMode(PDW, INPUT_PULLUP);											// Making the interrupt pin INPUT_PULLUP makes it less sensitive to noise, but switches the logic.
	pinMode(ARM, INPUT_PULLUP);											// Making the interrupt pin INPUT_PULLUP makes it less sensitive to noise, but switches the logic.
	pinMode(FIR, INPUT_PULLUP);											// Making the interrupt pin INPUT_PULLUP makes it less sensitive to noise, but switches the logic.
	Serial.begin(9600);
  Serial2.begin(9600);                            // Direct EasyTransfer UART
  Serial3.begin(9600);                            // BLIP RS232 UART. Currently not used. I'll leave the code for it here for now.
  ETout.begin(details(dataSend), &Serial2);
  ETin.begin(details(dataReceive), &Serial2);
  dataSend.chardata[0] = 'f';
  dataSend.chardata[1] = '3';
  dataSend.chardata[2] = '0';
  dataSend.chardata[3] = 0;
  ETout.sendData();
  Serial3.print(dataSend.chardata);
}

void loop() 
{
	if(CannonReadyFlag)                      // Wait until the cannon is ready for input to enable all input interrupts.
  {
    noInterrupts();           // disable all interrupts
    TCCR1A = 0;               //timer1 setup
    TCCR1B = 0;
    TCNT1  = 0;               // Reset the count in the timer

    OCR1A = 5000;            // compare match register I'm not sure exactly how much that value ammounts to but it is roughly 200-250 milisec
    TCCR1B |= (1 << WGM12);   // Clear Timer Count on interrupt mode
    TCCR1B |= (1 << CS12);    // 256 prescaler means that the timer wil tick 256 times slower than the clock speed
    TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
    interrupts();
 
    attachInterrupt(digitalPinToInterrupt(FIR), fireDebounce, RISING);
    updateMenu();
    CannonReadyFlag = 0;
  }

  if(ETin.receiveData())
    handleCommand(dataReceive.chardata);
}

void serial3Event()
{
  char buf[128] = "";          // Set up a small buffer for readBytes
  
  if(Serial3.available() < 4)
    return;                    // do nothing until we have 4 characters(1 command)

  Serial3.readBytes(buf, 4);     // Read the command

  Serial.print(buf);
  
  handleCommand(buf);           
  
}

ISR(TIMER1_COMPA_vect)          // timer compare interrupt service routine
{
  if(ignoreFlag)
    return;
  if((safe && !digitalRead(ARM)))
  {
    sendCommand("pur");
    arm();
  }
  if((!safe && digitalRead(ARM)))
    arm();
  if(!safe)
    return;
  if(!digitalRead(AUP))
  {
    if(angle < MAX_ANGLE)
    {
      angle++;
      if(angle%1 == 0)                        //Send the command every X increases of the angle. Currently 1.
      {
        dataSend.chardata[0] = 'a';
        dataSend.chardata[1] = angle/10 + 0x30;
        dataSend.chardata[2] = angle%10 + 0x30;
        dataSend.chardata[3] = 0;
        ETout.sendData();
        Serial3.print(dataSend.chardata);
      }
    }
    updateMenu();
    joystickupheld = 1;
  }
  else if(joystickupheld == 1)
  {
    dataSend.chardata[0] = 'a';
    dataSend.chardata[1] = angle/10 + 0x30;
    dataSend.chardata[2] = angle%10 + 0x30;
    dataSend.chardata[3] = 0;
    ETout.sendData();
    Serial3.print(dataSend.chardata);
    joystickupheld =0;
  }
  if(!digitalRead(ADW))
  {
    if(angle > MIN_ANGLE)
    {
      angle--;
      if(angle%1 == 0)
      {
        dataSend.chardata[0] = 'a';
        dataSend.chardata[1] = angle/10 + 0x30;
        dataSend.chardata[2] = angle%10 + 0x30;
        dataSend.chardata[3] = 0;
        ETout.sendData();
        Serial3.print(dataSend.chardata);
      }
    }
    updateMenu();
    joystickdwheld = 1;
  }
  else if(joystickdwheld == 1)
  {
    dataSend.chardata[0] = 'a';
    dataSend.chardata[1] = angle/10 + 0x30;
    dataSend.chardata[2] = angle%10 + 0x30;
    dataSend.chardata[3] = 0;
    ETout.sendData();
    Serial3.print(dataSend.chardata);
    joystickdwheld =0;
  }
  if(!digitalRead(PUP) && !puheld)
  {
    puheld = 1;
    sendCommand("puh");
  }
  if(digitalRead(PUP) && puheld)
  {
    puheld = 0;
    sendCommand("pur");
  }
  if(!digitalRead(PDW) && !pdheld)
  {
    pdheld = 1;
    sendCommand("pdh");
  }
  if(digitalRead(PDW) && pdheld)
  {
    pdheld = 0;
    sendCommand("pdr");
  }
}
// It's the deboun-cing functionw that are called during interrupt. It simply buffers up the interrupts only allowing once interrupt every debouncing_time milisec
// It calls the actual Interrupt service routine only after the debouncing.


void fireDebounce()
{
  if(ignoreFlag)
    return;
	if((long)(micros() - last_micros) >= debouncing_time*4000)
	{
		fire();
		last_micros = micros();
	}
}

// The ISRs (Interrupt Service Routine).
//These two will be heavily modified. I don't know exactly how we will do this yet.
void arm()
{	
  sendCommand("f10");
	safe = !safe;
  digitalWrite(BL, safe);
  fired = 0;
	//do extra things here like switching ignore flags, etc.
}

void fire()
{	
  if(!safe && !fired)
  {
    fired = 1;
     sendCommand("f20");
  }
}

void displayMenu()
{
	lcd.clear();
	lcd.setCursor(0, 1);			// Sets the lcd cursor to 1st line beggining
	lcd.print("deg");
	lcd.setCursor(8, 1);			// 2nd line beginning and etc.
	lcd.print("PSI");
	lcd.setCursor(16, 1);
	lcd.print("m/s");
//	lcd.setCursor(0, 3);
//	lcd.print("Speed:");
}

// The X values for setCursor were made so the values will be outputted at the end of each row without overflowwing into the next line. It assumes only 2 decimal values.
// 7 will show up as 07
void updateMenu()
{
  if(nopow)
    return;
	if(angle >= 0)
	{
		lcd.setCursor(0, 0);
		lcd.print(String(angle));              	//decmal value 223 is the value for the degree sign for the J204A degree character.
	}
  if(pressure >= 0)
  {
    lcd.setCursor(9, 0);
	  lcd.print(String(pressure/10) + String(pressure%10));
    if(pressure < 5)
    {
      lcd.setCursor(15, 0);
      lcd.print(" N/A ");
    }
    else
    {
      float velocity = 0.3182*pressure + 1.0126;
      velocity = round(velocity*10.0)/10.0;
      lcd.setCursor(15, 0);
      lcd.print(String(velocity));
    }
  }
}

void sendCommand(char* command)
{
   dataSend.chardata[0] = command[0];
   dataSend.chardata[1] = command[1];
   dataSend.chardata[2] = command[2];
   dataSend.chardata[3] = 0;
   ETout.sendData();
   Serial3.print(dataSend.chardata);

   Serial.print(dataSend.chardata);
}

void handleCommand(char* command)
{
  switch(command[0])        // If it is a then the command is aXX where XX is the curent angle. Example: a17 is angle of 17 deg.
  {
    case 'a':   //angle
    {
      if(angle != (command[1] - 0x30)*10 + (command[2] - 0x30))
      {
        angle = (command[1] - 0x30)*10 + (command[2] - 0x30);   // I don't know if this is the best way to convert the 2nd and 3rd char to a number but it works.
        updateMenu();
      }
      break;
    }
    case 'p':   //pressure
    {
      if(command[1] == '-')
      {
        pressure = -1;
        lcd.clear();
        lcd.print("No Power to Cannon");
        nopow = 1;
        break;
      }
      pressure = (command[1] - 0x30)*10 + (command[2] - 0x30);
      nopow = 0;
      updateMenu();
      break;
    }
    case 'f':   //firecode (error code)
    {
      switch(command[1])
      {
        case '0':     // Error State
        {
          if(command[2] == '1')
          {
            sendCommand("pur");
            ignoreFlag = 1;
            lcd.clear();
            lcd.print("Someone is in front!");
            break;
          }
          if(command[2] == '2');
          {
            if(!digitalRead(PUP))
               sendCommand("puh");
            if(!digitalRead(PDW))
              sendCommand("pdh");
            ignoreFlag = 0;
            lcd.clear();
            displayMenu();
            updateMenu();
            break;
          }
          if(command[2] == '3')
          {
            displayMenu();
            updateMenu();
            break;
          }
          break;
        }
        case '3':     // Cannon is ready.
        {
          CannonReadyFlag = 1;
          break;
        }
        case '4':     // Flip Cannon State
        {
          arm();
          break;
        }
      }
      break;
    }
  }
}

