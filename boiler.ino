#include <LiquidCrystal.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// --- LCD Configuration ---
const int rs=8, en=9, d4=10, d5=11, d6=12, d7=13;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// --- DS18B20 Configuration ---
#define ONE_WIRE_BUS 7
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// --- Variables ---
float tempActual = 0.0;
float tempDorit = 22.0; // You can change this value as needed
bool alertaApa=false;
bool ok=false;
int windowSize=1000;
unsigned long windowStartTime=0;
int onTime;



//bool ultimaStareStabila = true;
uint8_t istoriePin = 0;

void setup() {
  Serial.begin(9600);
  
  

  // Initialize LCD
  lcd.begin(16, 2);
  lcd.print("System Loading...");
  
  // Initialize Sensors
  sensors.begin();
  sensors.setResolution(10); // Set to 10-bit for a good balance of speed/detail
  
  delay(1000);
  lcd.clear();

  // Initialize ADC: Enable it and set Prescaler to 128
  // 16MHz / 128 = 125kHz (ideal for ADC accuracy)
  ADCSRA |= (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);


  //  Setup for Button (Pin 4) ---
  DDRD &= ~(1 << DDD4);    
  PORTD &= ~(1 << PORTD4);  

  // Setup for Button (Pin 3) ---
  DDRD &= ~(1 << DDD3);    
  PORTD &= ~(1 << PORTD3); 

  //Setup for water level (Pin 5)
  DDRD &= ~(1 << DDD5);    
  PORTD &= ~(1 << PORTD5); 

  //Setup slow PWM (Pin 6)
  DDRD |= (1 << DDD6);




  // --- Configurare Întrerupere Hardware (Pin 3) ---
  // ISC11=1 și ISC10=1 înseamnă "Rising Edge" (când trece de la LOW la HIGH)
  EICRA &= ~((1 << ISC11) | (1 << ISC10) | (1 << ISC01) | (1 << ISC00));
  //INT1
  EICRA |= (1 << ISC11) | (1 << ISC10);
  //INT0
  EICRA |= (1 << ISC00);

  // EIMSK activează întreruperea externă INT1 (Pin 3)
  EIMSK |= (1 << INT1);

  EIMSK |=(1 << INT0);

  sei();


  lcd.clear();
  lcd.begin(16, 2);
  lcd.print("Start system");
  delay(1000);

  while (true) {
  // 1. Wait until the pin goes HIGH
    if (PIND & (1 << PIND4)) {
   break;
  }   
}

}

ISR(INT1_vect)
{
  lcd.clear();
  lcd.begin(16, 2);
  lcd.print("Sys interuption");
  delay(1000);
  digitalWrite(6, LOW);
 
  while (true) {
   
    if (PIND & (1 << PIND4)) {
      break;
    }   
}
  
}

ISR(INT0_vect)
{
  alertaApa=true;
  digitalWrite(6, LOW);
  EIMSK &= ~(1 << INT0);

  
}

void loop() {
  //tratarea intreruperii INT0
  if(alertaApa)
  {
    digitalWrite(6, LOW);
    lcd.clear();
    lcd.print("No Water");
    delay(500);
    istoriePin=0x00;
    digitalWrite(6, LOW);

    while (1) { 
      ok=isWaterStable();
      delay(10);
       if(ok==true)
       {
        alertaApa = false;
        break;
       }
      }

    
    

    // Scriind un '1' în EIFR la bitul INTF0, ștergem orice întrerupere memorată eronat.
    EIFR |= (1 << INTF0); 

    // 4. REACTIVĂM întreruperea
    
    EIMSK |= (1 << INT0);

    lcd.clear();
    lcd.print("Water OK");
    delay(500);

  }

  if(!alertaApa){

  unsigned long now = millis();

  if(now-windowStartTime >=windowSize)
    {
      windowStartTime=now;

      tempDorit=readPotentiometer();
      tempActual = getTemperature();

      Serial.print("ActualTemp:"); // FĂRĂ spațiu la final
      Serial.print(tempActual);
      Serial.print(",");           // Separator (virgulă sau spațiu)

      Serial.print("Setpoint:");
      Serial.println(tempDorit);

      Serial.print("Actual Temp: ");
      Serial.println(tempActual);

      // Update the LCD Display
      updateDisplay(tempActual, tempDorit);

      float pidOutput= calculatePID(tempDorit,tempActual);

      onTime= (pidOutput / 255.0) * windowSize;
    }

    if (onTime > (now-windowStartTime))
  {
    digitalWrite(6, HIGH);
  }
  else
  {
    digitalWrite(6, LOW);
  }


  }
    

  

  



  
}

// --- FUNCTION: Write to LCD ---
void updateDisplay(float x, float y) {
  lcd.clear();

  // First Row
  lcd.setCursor(0, 0);
  lcd.print("temp act: ");
  lcd.print(x,1);

  // Second Row
  lcd.setCursor(0, 1);
  lcd.print("temp dor: ");
  lcd.print(y, 1);
}

// --- FUNCTION: Get Temperature ---
float getTemperature() {
  sensors.requestTemperatures(); 
  float temp = sensors.getTempCByIndex(0); 
  return temp;
}

float readPotentiometer() {
  // 1. Set Reference to AVcc (5V) and select Channel A0
  // Clearing MUX3:0 selects ADC0/A0
  ADMUX = (1 << REFS0); 

  // 2. Start conversion by setting ADSC bit to 1
  ADCSRA |= (1 << ADSC);

  // 3. Wait for conversion to finish (ADSC will become 0)
  while (ADCSRA & (1 << ADSC));

  // 4. Retrieve the 10-bit value (0 to 1023)
  int rawValue = ADC;

  // 5. Map the 0-1023 value to a 20-80°C range manually
  // Formula: (Value / MaxValue) * Range + Offset
  float mappedTemp = (rawValue / 1023.0) * (80.0 - 20.0) + 20.0;

  return mappedTemp;
}

bool isWaterStable() {
    // Shiftăm la stânga și adăugăm ultima citire pe prima poziție
    istoriePin = (istoriePin << 1) | ((PIND & (1 << PIND5)) >> PIND5);

     /* Serial.print("Istoric: ");
    for (int i = 7; i >= 0; i--) {
        Serial.println(bitRead(istoriePin, i)); // Printează bit cu bit pentru a vedea și zerourile de la început
    }  */
    
    if (istoriePin == 0xFF) return true;  // Semnal stabil HIGH
    //if (istoriePin == 0x00) return true; // Semnal stabil LOW
    
    // Dacă e în proces de bouncing, păstrăm ultima stare stabilă cunoscută
    return false; 
}

int calculatePID(float target, float current) {
  // --- PID Constants (Tune these for your heater) ---
  const float Kp = 50.0;
  const float Ki = 0.1;
  const float Kd = 0.1;

  // --- Persistent Variables ---
  static float integral = 0;
  static float lastError = 0;
  static unsigned long lastTime = 0;

  // 1. Calculate time elapsed
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  if (dt <= 0) dt = 0.1; // Prevent division by zero

  // 2. Calculate Error
  float error = target - current;

  // 3. Proportional term
  float P = Kp * error;

  float I=0;
  // 4. Integral term (with Anti-Windup)
  if(tempDorit-tempActual<2)
  {
      integral += error * dt;
      I = Ki * integral;

  }
  
  
  // Constrain Integral to prevent overshoot
  if (I > 255) { I = 255; integral = 255 / Ki; }
  if (I < 0)   { I = 0;   integral = 0; }

  // 5. Derivative term
  float D = Kd * (error - lastError) / dt;

  // 6. Final Output
  float output = P + I + D;

  // 7. Cleanup and State Saving
  lastError = error;
  lastTime = currentTime;

  // Return constrained PWM value
  return (int)constrain(output, 0, 255);
}
