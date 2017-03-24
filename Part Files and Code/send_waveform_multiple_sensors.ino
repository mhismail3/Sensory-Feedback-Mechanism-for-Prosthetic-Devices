#include <Wire.h>
#include <TimerOne.h>
#include <TimerThree.h>

#define NUM_OUTPUTS 3
#define NUM_SENSORS 3

/*========================================================================= 
 * 
 *   PRESSURE SENSOR VARIABLES
 *
  *========================================================================
 */

#define FREESCALE_ADDRESS 0xC0
#define SENSOR_ALL_ON 0x0C
#define SENSOR_ALL_OFF 0x0D

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
#define MPL115A2_ADDRESS                       (0x60)    // 1100000
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
#define MPL115A2_REGISTER_PRESSURE_MSB         (0x00)
#define MPL115A2_REGISTER_PRESSURE_LSB         (0x01)
#define MPL115A2_REGISTER_TEMP_MSB             (0x02)
#define MPL115A2_REGISTER_TEMP_LSB             (0x03)
#define MPL115A2_REGISTER_A0_COEFF_MSB         (0x04)
#define MPL115A2_REGISTER_A0_COEFF_LSB         (0x05)
#define MPL115A2_REGISTER_B1_COEFF_MSB         (0x06)
#define MPL115A2_REGISTER_B1_COEFF_LSB         (0x07)
#define MPL115A2_REGISTER_B2_COEFF_MSB         (0x08)
#define MPL115A2_REGISTER_B2_COEFF_LSB         (0x09)
#define MPL115A2_REGISTER_C12_COEFF_MSB        (0x0A)
#define MPL115A2_REGISTER_C12_COEFF_LSB        (0x0B)
#define MPL115A2_REGISTER_STARTCONVERSION      (0x12)

// constants for each sensor
float _mpl115a2_a0[NUM_SENSORS];
float _mpl115a2_b1[NUM_SENSORS];
float _mpl115a2_b2[NUM_SENSORS];
float _mpl115a2_c12[NUM_SENSORS];

// MCP23008 (demux) addresses
int addresses[8] = {0x80,0x40,0x20,0x10,0x08,0x04,0x02,0x01};


// Hysteresis and Control variables
double min_pressure[NUM_SENSORS] = {800,800,800};
double max_pressure[NUM_SENSORS] = {1600,1600,1600};
double overshoot_pressure[NUM_SENSORS] = {0,0,0};

float offset_pressure[NUM_SENSORS];
float std_threshold[NUM_SENSORS];
float std_pressure[NUM_SENSORS];
float filt_pressure[NUM_SENSORS];

float pressure_hundred[NUM_SENSORS][100];
int step_size = 15;
float pressure_past[NUM_SENSORS][15];
float pressure_mean[NUM_SENSORS];
int hysteresis_counter = 0;
bool first_hundred = false;

/*========================================================================= 
 * 
 *   STIMULATION VARIABLES
 *
  *========================================================================
 */

// CD4051be pins
int pin_a = 7;
int pin_b = 6;
int pin_c = 8;

bool pin_a_vals[5] = {0,1,0,1,0};
bool pin_b_vals[5] = {0,0,1,1,0};
bool pin_c_vals[5] = {0,0,0,0,1};

bool pin_a_off = 1;
bool pin_b_off = 1;
bool pin_c_off = 1;

// variables for setting amplitude ranges
bool set_min;
bool set_max;
bool need_pot_zero = true;
int set_count;

volatile int amps[NUM_OUTPUTS];
int mags[NUM_OUTPUTS];

int min_stim[NUM_OUTPUTS];
int max_stim[NUM_OUTPUTS];

elapsedMillis timeElapsed = 0;
int resolution = 12;
int max_amp_send = 4096;
int min_amp_send = 0;
int zero_voltage = 2048;

bool turn_off = false;

double freq = 50; // Hz
int pulse_width = 400;

volatile int count = 0; // represents which wave we are on currently

// PINS
int button_pin = 10;
int pot_pin = 16;

bool pressure_control; 


void setup() {
  pressure_control = false;
  Wire.begin();
  Serial.begin(9600);
  analogWriteResolution(resolution);
  
  // initialize pressure sensors
  
  // set I/O pins to outputs
  Wire.beginTransmission(0x20); //begins talking to the slave device
  Wire.write(0x00); //selects the IODIRA register
  Wire.write(0x00); //this sets all port A pins to outputs
  Wire.endTransmission(); //stops talking to device
  // read coefficients from sensors
  for(int i=0; i < NUM_SENSORS; i++)
  {
    pressure_mean[i] = 0;
    std_threshold[i] = 0;
    std_pressure[i] = 0;
    filt_pressure[i] = 0;
    offset_pressure[i] = -1;
    min_stim[i] = zero_voltage;
    max_stim[i] = max_amp_send;
    
    for(int j = 0; j < 100; j++)
    {
      pressure_hundred[i][j] = 0;
      if(j < 15)
        pressure_past[i][j] = 0;
    }
    // turn on sensor
    turn_on_sensor(i);
    delay(5);
    // read coefficients from sensor
    readCoefficients(i);
  }

  
  // initialize stim variables
  for(int i=0;i<NUM_OUTPUTS;i++)
  {
    mags[i] = zero_voltage;
    amps[i] = zero_voltage;
    min_stim[i] = zero_voltage;
    max_stim[i] = 4096;
  }
  // initialize set amplitudes stuff
  set_min = false;
  set_max = false;
  set_count = 0;
  pinMode(button_pin, INPUT);
  // initialize stim timers
  Timer1.initialize(1/freq*1000000/NUM_OUTPUTS); // send a wavefrom every 0.02 seconds == 20ms
  Timer1.attachInterrupt(send_waveform); // function that sends waveform
 
  Timer3.initialize(pulse_width/2); // period of a pulse_width
  Timer3.attachInterrupt(modify_waveform); // this timer stops sending a waveform
  Timer3.stop(); // don't start until wavefrom is sent
 
  pinMode(pin_a,OUTPUT);
  pinMode(pin_b,OUTPUT);
  pinMode(pin_c,OUTPUT);
  digitalWrite(pin_a,pin_a_vals[NUM_SENSORS-1]);
  digitalWrite(pin_b,pin_b_vals[NUM_SENSORS-1]);
  digitalWrite(pin_c,pin_c_vals[NUM_SENSORS-1]);
}

void send_waveform()
{
  amps[count] = mags[count];
  if(pressure_control == false)
  {
    Serial.print(((double)amps[count]-zero_voltage)/(zero_voltage)*3.3/2.0);
    Serial.print("  ");
  }
  // starts sending waveform (positive portion)
  analogWrite(A14, amps[count]); 
  Timer3.restart();

}

void modify_waveform()
{
  // turns wave off
  if(turn_off)
  {
    analogWrite(A14, zero_voltage);
    Timer3.stop(); 
    count = count+1;  
    Timer1.setPeriod(100);
    Timer1.restart();
    if(count == NUM_OUTPUTS) // reset count
    {
      count = 0;    
      Timer1.setPeriod(1/freq*1000000 - NUM_OUTPUTS*500)
      Timer1.restart();
      if(pressure_control == false)
        Serial.println();
          
    }
    digitalWrite(pin_a,pin_a_vals[count]);
    digitalWrite(pin_b,pin_b_vals[count]);
    digitalWrite(pin_c,pin_c_vals[count]);
      
  }
  // does biphasic portion
  else
  {
    analogWrite(A14, max_amp_send - amps[count]);
    Timer3.restart();
  }
  turn_off = !turn_off; 
}


void loop() 
{
  if(pressure_control==true)
  {
    pressure_stim();
  }
  else
  {
    stim();
    bool button_state = digitalRead(button_pin);
    if(button_state == 1)
      set_amplitudes();
  }
}

// determines mags[count] for all count < NUM_OUTPUTS so that the stimulation waveforms are correct
// stimulation waveforms are based on the pot value (1 to 1023)
void stim()
{
  int min_pot = 100;
  //Serial.println(analogRead(pot_pin));
  for(int i = 0; i < NUM_OUTPUTS; i++)
  {
    int pot_in = analogRead(pot_pin);
    if(need_pot_zero == true && pot_in < min_pot)
    {
      need_pot_zero = false;
      mags[i] = zero_voltage;
    }
    else if(need_pot_zero == false && i == set_count)
    {  
      mags[set_count] = convert_to_volts(pot_in);
      
    }
    else
    {
      mags[i] = zero_voltage;
    }
      
  }
  delay(5);
}

void set_amplitudes()
{
  
  if(set_min == false)
  {
    min_stim[set_count] = convert_to_volts(analogRead(pot_pin));
    set_min = true;
  }
  else if(set_max == false)
  {
    max_stim[set_count] = convert_to_volts(analogRead(pot_pin));
    set_max = true;
  }
  
  if(set_min == true && set_max == true && set_count < NUM_OUTPUTS-1)
  {
    set_min = false;
    set_max = false;
    set_count = set_count + 1;
    need_pot_zero = true;
  }
  else if(set_min == true && set_max == true)
  {
    set_min = false;
    set_max = false;
    set_count = 0;
    pressure_control = true;
    need_pot_zero = true;
  }

  delay(500);
  bool bs = 1;
  while(bs == 1)
    bs = digitalRead(button_pin);
}

double convert_to_volts(double pot_in)
{
  int min_pot = 100;
  int max_pot = 950;

  if(pot_in < min_pot)
    return zero_voltage;
  if(pot_in > max_pot)
    return max_amp_send;
   
  return (pot_in-min_pot)/(max_pot-min_pot)*(max_amp_send - zero_voltage) + zero_voltage;
}


void pressure_stim(){

  timeElapsed = 0;
  float oTemp=0;
  float oPressure = 0;
  float bad_sensor = 0;

  for(int i = 0; i < NUM_SENSORS; i++)
  {
    turn_on_sensor(i);
    delay(2);
    Wire.beginTransmission(MPL115A2_ADDRESS);
    i2cwrite((uint8_t)MPL115A2_REGISTER_STARTCONVERSION);
    i2cwrite((uint8_t)0x00);
    Wire.endTransmission();
    delay(2);
    uint16_t pressure,temp;
    Wire.beginTransmission(MPL115A2_ADDRESS);
    i2cwrite((uint8_t)MPL115A2_REGISTER_PRESSURE_MSB);  // Register
    Wire.endTransmission();

    Wire.requestFrom(MPL115A2_ADDRESS, 4);
    pressure = (( (uint16_t) i2cread() << 8) | i2cread()) >> 6;
    temp = (( (uint16_t) i2cread() << 8) | i2cread()) >> 6;
    oPressure = _mpl115a2_a0[i] + (_mpl115a2_b1[i] + _mpl115a2_c12[i]*temp ) * pressure + _mpl115a2_b2[i] * temp;
    if(!first_hundred)
    {
      if(offset_pressure[i] == -1)
        offset_pressure[i] = oPressure;
   
      pressure_hundred[i][hysteresis_counter] = oPressure; 
    }
    else
    {
      pressure_past[i][hysteresis_counter] = oPressure;
      apply_offset(i);
    }
    float scaled_pressure = (oPressure-offset_pressure[i])/(max_pressure[i]-offset_pressure[i]);
    if(scaled_pressure >= 0 && scaled_pressure <=1)
    {
    }
    else if(scaled_pressure > 1 || oPressure < overshoot_pressure[i])
    {  
      scaled_pressure = 1;
    }
    else
   {
      scaled_pressure = 0;
    }
    
    map_mags(scaled_pressure, i);
    Serial.print(oPressure);
    Serial.print("  ");  
    
   }
/*for(int i = 0; i < NUM_SENSORS; i++)
  {
    turn_on_sensor(i);
    if(i == 0)
      delay(2);
    // Get raw pressure and temperature settings
    Wire.beginTransmission(MPL115A2_ADDRESS);
    i2cwrite((uint8_t)MPL115A2_REGISTER_STARTCONVERSION);
    i2cwrite((uint8_t)0x00);
    Wire.endTransmission();
    if(i == 0)
    {
      delay(1);
      uint16_t pressure,temp;
      Wire.beginTransmission(MPL115A2_ADDRESS);
      i2cwrite((uint8_t)MPL115A2_REGISTER_PRESSURE_MSB);  // Register
      Wire.endTransmission();

      Wire.requestFrom(MPL115A2_ADDRESS, 4);
      pressure = (( (uint16_t) i2cread() << 8) | i2cread()) >> 6;
      temp = (( (uint16_t) i2cread() << 8) | i2cread()) >> 6;
      bad_sensor = _mpl115a2_a0[i] + (_mpl115a2_b1[i] + _mpl115a2_c12[i]*temp ) * pressure + _mpl115a2_b2[i] * temp;
   
    }
  }
  for(int i = 0; i < NUM_SENSORS; i++)
  {
    if(i != 0)
    {
      turn_on_sensor(i);
    
      uint16_t pressure,temp;
      Wire.beginTransmission(MPL115A2_ADDRESS);
      i2cwrite((uint8_t)MPL115A2_REGISTER_PRESSURE_MSB);  // Register
      Wire.endTransmission();

      Wire.requestFrom(MPL115A2_ADDRESS, 4);
      pressure = (( (uint16_t) i2cread() << 8) | i2cread()) >> 6;
      temp = (( (uint16_t) i2cread() << 8) | i2cread()) >> 6;
      oPressure = _mpl115a2_a0[i] + (_mpl115a2_b1[i] + _mpl115a2_c12[i]*temp ) * pressure + _mpl115a2_b2[i] * temp;
    }
    else
    {
      oPressure = bad_sensor;
    }
  
    if(!first_hundred)
    {
      if(offset_pressure[i] == -1)
        offset_pressure[i] = oPressure;
   
      pressure_hundred[i][hysteresis_counter] = oPressure; 
    }
    else
    {
      pressure_past[i][hysteresis_counter] = oPressure;
      apply_offset(i);
    }
    float scaled_pressure = (oPressure-offset_pressure[i])/(max_pressure[i]-offset_pressure[i]);
    if(scaled_pressure >= 0 && scaled_pressure <=1)
    {
    }
    else if(scaled_pressure > 1 || oPressure < overshoot_pressure[i])
    {  
      scaled_pressure = 1;
    }
    else
   {
      scaled_pressure = 0;
    }
    
    map_mags(scaled_pressure, i);
    Serial.print(oPressure);
    Serial.print("  ");  
  
  }*/
  Serial.println();
  
  hysteresis_counter+=1;
  if(!first_hundred && hysteresis_counter == 100)
  {
    setup_hysteresis();
  }
  else if(first_hundred && hysteresis_counter == step_size)
  {
    hysteresis_counter = 0;
  }
  
  while(timeElapsed <10); // sampling rate is 1/.1 = 10Hz
}

void map_mags(float scaled_p, int sensor)
{
    if(scaled_p < 0.05)
    {
      mags[sensor] = (scaled_p*(max_stim[sensor] - min_stim[sensor]) + min_stim[sensor] -zero_voltage)/2.0 + zero_voltage;
    }
    else
    {
      mags[sensor] = scaled_p*(max_stim[sensor] - min_stim[sensor]) + min_stim[sensor];  
    }
    
}
// Pressure sensor code

void turn_on_sensor(int i)
{
  Wire.beginTransmission(0x20); //starts talking to slave device
  Wire.write(0x09); //selects the GPIO pins
  Wire.write(addresses[i]); // turns on pins 0 and 1 of GPIOA
  Wire.endTransmission(); //ends communication with the device 
}

void readCoefficients(int i) {
  int16_t a0coeff;
  int16_t b1coeff;
  int16_t b2coeff;
  int16_t c12coeff;

  Wire.beginTransmission(MPL115A2_ADDRESS);
  i2cwrite((uint8_t)MPL115A2_REGISTER_A0_COEFF_MSB);
  Wire.endTransmission();

  Wire.requestFrom(MPL115A2_ADDRESS, 8);
  a0coeff = (( (uint16_t) i2cread() << 8) | i2cread());
  b1coeff = (( (uint16_t) i2cread() << 8) | i2cread());
  b2coeff = (( (uint16_t) i2cread() << 8) | i2cread());
  c12coeff = (( (uint16_t) (i2cread() << 8) | i2cread())) >> 2;
  
  _mpl115a2_a0[i] = (float)a0coeff / 8;
  _mpl115a2_b1[i] = (float)b1coeff / 8192;
  _mpl115a2_b2[i] = (float)b2coeff / 16384;
  _mpl115a2_c12[i] = (float)c12coeff;
  _mpl115a2_c12[i] /= 4194304.0;
}

static uint8_t i2cread(void) {
  uint8_t x;
  #if ARDUINO >= 100
  x = Wire.read();
  #else
  x = Wire.receive();
  #endif
  //Serial.print("0x"); Serial.println(x, HEX);
  return x;
}


static void i2cwrite(uint8_t x) {
  #if ARDUINO >= 100
  Wire.write((uint8_t)x);
  #else
  Wire.send(x);
  #endif
}


float mean(float array[], int array_length)
{
  float ret = 0;
  for(int i = 0; i < array_length; i++)
    ret+= array[i];
  return ret/array_length;
}

float stdev(float array[], float mean, int array_length)
{
  float ret = 0;
  for(int i = 0; i < array_length; i++)
  {
    ret+= (array[i]-mean)*(array[i]-mean);

  }

  return 0.1*ret/sizeof(array)*sizeof(float);
}

void setup_hysteresis()
{
  int array_length = sizeof(pressure_hundred[0])/sizeof(float);
  for(int i = 0; i < NUM_SENSORS; i++)
  {
    float m = mean(pressure_hundred[i], array_length);
    std_threshold[i] = stdev(pressure_hundred[i],m, array_length)*2;
  }
  hysteresis_counter = 0;
  first_hundred = true;
}

bool reached_max_pressure(int sensor)
{
  for(int i = 0; i < step_size; i++)
    if(pressure_past[sensor][i] < overshoot_pressure[sensor])
      return true;

  return false;
}

void apply_offset(int i)
{
  int array_length = sizeof(pressure_past[i])/sizeof(float);
  float pressure_mean = mean(pressure_past[i], array_length);
  std_pressure[i] = stdev(pressure_past[i], pressure_mean, array_length);
  if(std_pressure[i] < std_threshold[i] && !reached_max_pressure(i))
  {
    offset_pressure[i] = pressure_mean;
  }
}
