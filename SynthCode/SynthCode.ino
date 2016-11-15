
/*Synth Code
TODO: 
Write read function
Write UART command interface

Pin allocation:

2 - Synth Power
3 - Synth Enable 
5 - RF Enable
AD6 - Voltage divider (0.675)
AD3 - Diode reference 1.75V
AD5 - SDA
AD4 - SCL
10 - Latch enable
11 - Data
12 - Mux
13 - CLK*/

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);
#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2
#define LOGO16_GLCD_HEIGHT 16 
#define LOGO16_GLCD_WIDTH  16 

#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

//Synth Pin Definitions
#define PWPin 2
#define ENPin 3
#define RFENPin 5

#define LEPin 4
#define DATAPin 11
#define MUXPin 12
#define CLKPin 13

//Attenuator Pin Definitions
//A7 -- Attenuator shdn pin
#define ATTEN_CLK 9
#define ATTEN_DIN 7
#define ATTEN_DOUT 8
#define ATTEN_CS 6

long readRegister();
void initDeviceRegisters();
void writeRegister(unsigned long data);

/*########################SYNTH##############################*/
/*Register switches and addresses*/
//Adresses
const unsigned long Reg0 = (long)0b000;
const unsigned long Reg1 = (long)0b001;
const unsigned long Reg2 = (long)0b010;
const unsigned long Reg3 = (long)0b011;
const unsigned long Reg4 = (long)0b100;
const unsigned long Reg5 = (long)0b101;
const unsigned long Reg6 = (long)0b110;

//*****************Register 0 settings
//Int-N Frac-N mode control
const unsigned long INT_FRAC = (long)0b0;
const unsigned long INT_INT = (long)0b1<<31;

//*****************Register 1 settings 
const unsigned long CPT_NORM = (long)0b00<<27;
const unsigned long CPT_SOURCE = (long)0b10<<27;
const unsigned long CPT_SINK = (long)0b11<<27;

const unsigned long CPL_EN = (long)0b01<<29;

const unsigned long CPOC = (long)0b1<<31;


//*****************Register 2 settings
const unsigned long LD_FAST = (long)0b1<<31;
const unsigned long LD_SLOW = (long)0b0<<31;

const unsigned long SDN_LN = (long)0b00<<29;
const unsigned long SDN_LS1 = (long)0b10<<29;
const unsigned long SDM_LS2 = (long)0b11<<29;

const unsigned long MUX_TRI = (long)0b000<<26;
const unsigned long MUX_D_VDD = (long)0b001<<26;
const unsigned long MUX_D_GND = (long)0b010<<26;
const unsigned long MUX_RDIV = (long)0b011<<26;
const unsigned long MUX_NDIV2 = (long)0b100<<26;
const unsigned long MUX_ALD = (long)0b101<<26;
const unsigned long MUX_DLD = (long)0b110<<26;
const unsigned long MUX_SER = (long)0b100<<26;

const unsigned long DBR_EN = (long)0b1<<25;

const unsigned long RDIV2_EN = (long)0b1<<24;

const unsigned long REG4DB_EN = (long)0b1<<13;

const unsigned long LDF_FRAC = (long)0b0<<8;
const unsigned long LDF_INT = (long)0b1<<8;

const unsigned long LDP_10 = (long)0b0<<7;
const unsigned long LDP_6 = (long)0b1<<7;

const unsigned long PDP_NEG = (long)0b0<<6;
const unsigned long PDP_POS = (long)0b1<<6;

const unsigned long SHDN = (long)0b1<<5;

const unsigned long TRI_EN = (long)0b1<<4;

const unsigned long RST_R_AND_N = (long)0b1<<3;

//*****************Register 3 settings
const unsigned long VAS_SHDN = (long)0b1<<25;

const unsigned long RETUNE_EN = (long)0b1<<24;

const unsigned long CDM_FastLock = (long)0b01<<15;
const unsigned long CDM_PhaseMode = (long)0b10<<15;

//*****************Register 4 settings
const unsigned long FB_DIV = (long)0b0<<23;
const unsigned long FB_FUND = (long)0b1<<23;

const unsigned long BDIV_VCO_DIV = (long)0b0<<9;
const unsigned long BDIV_VCO_FUND = (long)0b1<<9;

const unsigned long RFB_EN = (long)0b1<<8;

const unsigned long RFA_EN = (long)0b1<<5;

//*****************Register 5 settings
//Fractional/Integer mode setting  (Not sure what this does)
const unsigned long FractionalN_Mode = (long)0b0;
const unsigned long IntegerNAuto_Mode = (long)0b1<<24;

//Lock detect pin function

const unsigned long LD_Low = (long)0b0;
const unsigned long LD_Digital_Lock_Detect = (long)0b01<<22;
const unsigned long LD_Analog_Lock_Detect = (long)0b10<<22;
const unsigned long LD_High = (long)0b11<<22;

//Mux mode
const unsigned long MUX_OUT_Three_State_MSB = (long)0b0<<18;
const unsigned long MUX_OUT_D_VDD_MSB = (long)0b0<<18;
const unsigned long MUX_OUT_D_GND_MSB = (long)0b0<<18;
const unsigned long MUX_OUT_R_Divider_Output_MSB = (long)0b0<<18;
const unsigned long MUX_OUT_N_Divider_Output_D2_MSB = (long)0b0<<18;
const unsigned long MUX_OUT_Analog_Lock_Detect_MSB = (long)0b0<<18;
const unsigned long MUX_OUT_Digital_Lock_Detect_MSB = (long)0b0<<18;
const unsigned long MUX_OUT_Serial_Out_MSB = (long)0b1<<18;

//Default register setting variables, will be set during initialisation
unsigned long reg0Default;
unsigned long reg1Default;
unsigned long reg2Default;
unsigned long reg3Default;
unsigned long reg4Default;
unsigned long reg5Default;

/*#####################ATTENUATOR########################*/



long freq = 55000000;
int amplitude = 0;
boolean up = true;
unsigned int attenuation = 0; //10bit 46dB 0.045dB steps

// the setup routine runs once when you press reset:
void setup() {  

  //Initialize pin functions
  pinMode(ENPin, OUTPUT);  
  pinMode(RFENPin, OUTPUT);
  pinMode(PWPin, OUTPUT); 
  pinMode(LEPin, OUTPUT);
  pinMode(DATAPin, OUTPUT);  
  pinMode(CLKPin, OUTPUT); 
  pinMode(MUXPin, INPUT);

  pinMode(SHDN, OUTPUT);
  pinMode(ATTEN_CLK, OUTPUT); 
  pinMode(ATTEN_DIN, OUTPUT);
  pinMode(ATTEN_DOUT, INPUT);  
  pinMode(ATTEN_CS, OUTPUT); 
  
  //Initialize screen
  
  //Initialize synth
  //Make sure power is off
  digitalWrite(PWPin,LOW);
  //Disable synth
  digitalWrite(ENPin,LOW);
  digitalWrite(RFENPin,LOW);
  //Set Latch
  digitalWrite(LEPin,HIGH);  

  //Initialize attenuator
  //Make sure power is off
  digitalWrite(A7,LOW);
  //Disable comunication
  digitalWrite(ATTEN_CS,HIGH);
  digitalWrite(ATTEN_DIN,LOW);
  digitalWrite(ATTEN_CLK,LOW);
  
  //Setup serial port
  Serial.begin(9600);
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  // init done
  
  //Turn on synth power
  digitalWrite(PWPin,HIGH);
  //Wait for internals to settle
  delay(10);
  //Enable synth
  digitalWrite(ENPin,HIGH);
  //Wait for internals to settle
  delay(10);

  //Turn on attenuator power
  digitalWrite(A7,HIGH);
  //Wait for internals to settle
  delay(10);
  
  //Initialize device and set defaults
  initDeviceRegisters();
  
  freq = 55000000;
  setRegistersForFrequency(freq);

  writeAttenDAC(0b1110000000);
  //Serial.println("Reading");
  //readAttenDAC();
  //Turn on RF 
  digitalWrite(RFENPin,HIGH);
  
}

// the loop routine runs over and over again forever:
void loop() {
 
  //manualControl();
  //modeSelect();
  updateScreen();

  
  delay(1000);
}

void modeSelect()
{
  if(analogRead(0) > 512 and freq != 55000000)
  {
    freq = 55000000;
    setRegistersForFrequency(freq);
  }
  if(analogRead(0) <= 512 and freq != 90000000)
  {
    freq = 90000000;
    setRegistersForFrequency(freq);
  }
}

void manualControl()
{
  int coarseFreq = analogRead(0); 
  int fineFreq = analogRead(1); 
  //attenuation = analogRead(2); 
  
  freq = ((((long)coarseFreq)*600000) + ((long)fineFreq)*10000) - ((((long)coarseFreq)*600000) + ((long)fineFreq)*10000)%100000;

  //TODO: Amplitude control must still be implemented into registers
  setRegistersForFrequency(freq);
  
}

void updateScreen()
{
    display.clearDisplay();
    
    //Show the frequency on the screen
    display.setTextColor(WHITE);
    display.setTextSize(1);
    
    display.setCursor(0,0);    
    display.println("Freq: ");    
    display.setCursor(50,0);    
    display.println(freq/100000);    
    display.setCursor(90,0);    
    display.println("MHz");
    
    display.setCursor(0,16);    
    display.println("Ampl: ");    
    display.setCursor(50,16);    
    display.println(((float)amplitude)*3 - 4);    
    display.setCursor(90,16);    
    display.println("dBm");

    display.setCursor(0,30);    
    display.println("Atten: ");    
    display.setCursor(50,30);    
    display.println(attenuation);    
    display.setCursor(90,30);    
    display.println("dB");
    
    display.setCursor(0,45);    
    display.println("Bat: ");    
    display.setCursor(50,45);    
    display.println(getBatteryVoltage());    
    display.setCursor(90,45);    
    display.println("V");
    
    display.display();
}

void setAttenuation(float atten)
{
  //TODO: Fix this!!!!!!!!!
  //Convert attenuation to DAC level
  int dac_setting = (int)(atten/((float)46/(float)1023)); 
  //Write the dac value to the attenuator
  writeAttenDAC(dac_setting);
}

void writeAttenDAC(unsigned int dac_setting)
{
  //Bring down CS pin
  digitalWrite(ATTEN_CS,LOW);
  delay(1);
  //dac_setting = 0b0011111111;
  Serial.println(dac_setting,BIN);
  
  //Set to write mode
  digitalWrite(ATTEN_DIN,LOW);
  delay(1);
  digitalWrite(ATTEN_CLK,HIGH);
  delay(1);
  digitalWrite(ATTEN_CLK,LOW);
  delay(1);

  //Set register to 0
  digitalWrite(ATTEN_DIN,LOW);
  delay(1);
  digitalWrite(ATTEN_CLK,HIGH);
  delay(1);
  digitalWrite(ATTEN_CLK,LOW);
  delay(1);
  digitalWrite(ATTEN_CLK,HIGH);
  delay(1);
  digitalWrite(ATTEN_CLK,LOW);
  delay(1);



  //Write 10 bit dac value
  unsigned long buffer = 0;
  byte i = 0;
  for(i = 0;i < 10;i++)
  {
    Serial.print("writing bit: ");
    //Set pin state to MSB in stream data
    buffer = dac_setting&0b1000000000;
    if(buffer)
    {
       digitalWrite(ATTEN_DIN,HIGH);
       Serial.println(1);
    }
    else
    {
       digitalWrite(ATTEN_DIN,LOW);
       Serial.println(0);
    }
    
    //Shift data to next bit for following transfer
    dac_setting = dac_setting<<1;
    
    //Wait some time and throw clk
    delay(1);
    digitalWrite(ATTEN_CLK,HIGH);
    delay(1);
    digitalWrite(ATTEN_CLK,LOW);
    delay(1);
  }
  delay(1);

  //Bring up the CS pin to latch in the value
  digitalWrite(ATTEN_CS,HIGH);
  delay(1);
}

void readAttenDAC()
{
  //Bring down CS pin
  digitalWrite(ATTEN_CS,LOW);
  delay(1);
  
  //Set to read mode
  digitalWrite(ATTEN_DIN,HIGH);
  delay(1);
  digitalWrite(ATTEN_CLK,HIGH);
  delay(1);
  digitalWrite(ATTEN_CLK,LOW);
  delay(1);

  //Set register to 0
  digitalWrite(ATTEN_DIN,LOW);
  delay(1);
  digitalWrite(ATTEN_CLK,HIGH);
  delay(1);
  digitalWrite(ATTEN_CLK,LOW);
  delay(1);
  digitalWrite(ATTEN_CLK,HIGH);
  delay(1);
  digitalWrite(ATTEN_CLK,LOW);
  delay(1);



  //Write 10 bit dac value
  unsigned long buffer = 0;
  byte i = 0;
  for(i = 0;i < 10;i++)
  {
    Serial.print("reading bit: ");
    //Set pin state to MSB in stream data
    
    if(digitalRead(ATTEN_DOUT))
    {
       Serial.println(1);
    }
    else
    {
       Serial.println(0);
    }     
    
    //Wait some time and throw clk
    delay(1);
    digitalWrite(ATTEN_CLK,HIGH);
    delay(1);
    digitalWrite(ATTEN_CLK,LOW);
    delay(1);
  }
  delay(1);

  digitalWrite(ATTEN_CS,HIGH);
  delay(1);
}


float getBatteryVoltage()
{
  //AD6 - Voltage divider (0.675)
  //AD3 - Diode reference 1.75V
  int refVoltage = analogRead(3); // This level is 1.75
  float unit = 1.75/refVoltage;
  int divider = analogRead(6); 
  
  return (float)((divider*unit)/(0.675));
}

void setRegisters()
{  
  writeRegister(0x01400005);
  writeRegister(0x63B202FC);
  writeRegister(0x00000133);
  writeRegister(0x80005E42);
  writeRegister(0x200303E9);
  writeRegister(0x00200000);
}

/*Writes the recommended sequence of registers on initialisation*/
void initDeviceRegisters()
{
    //Fastlock timeout
    //tFastlock = M*CDIV/fPFD = 0.328s if CDIV == 4095
  
    long reserved;
    /* Setting up register 5
    *  IntergerN mode is set to automatic
    *  Lock detect is set to digital lock detect
    *  MUX is set to tri state
    */
    reg5Default = IntegerNAuto_Mode|LD_Digital_Lock_Detect|MUX_OUT_Three_State_MSB|Reg5;
    writeRegister(reg5Default);
    Serial.println(reg5Default,HEX);
    
    /* Setting up register 4
    *  Setup port A and B power and if they are enabled.
    *  Setup port B output
    *  Select the frequency band as well as the output divider with the given equations.
    *      
    *  fPDF = fREF*((1+DBR)/(R*(1+RDIV2)))
    *  Set for 50MHz crystal 
    *  BS = fPDF/50kHz
    *
    *  fRFOUTA = 400
    *  DIVA = 8
    *  fVCO = fRFOUTA*DIVA 
    *  If FB == 1:
    *  N + (F/M) = fVCO/fPFD
    *  N = 80
    *  M = 4000
    *  F = 128
    */
    long DIVA = (long)3<<20;
    long APWR = (long)3<<3;
    long BPWR = (long)0<<6;
    long BS = (long)0b00100000<<12;
    long BS_MSB = (long)0b11<<24;
    reserved = (long)0b011000<<26;
    reg4Default = FB_FUND|BS|BS_MSB|DIVA|reserved|BDIV_VCO_FUND|BPWR|RFA_EN|APWR|Reg4;
    writeRegister(reg4Default);
    Serial.println(reg4Default,HEX);
    
    //Setting up register 3
    long CDIV = (long)38<<3;
    long VCO = (long)0b000000<<26;
    reg3Default = VCO|CDIV|Reg3;
    writeRegister(reg3Default);
    Serial.println(reg3Default,HEX);

    
    /* Setting up register 2
    *  Setup the clock mode and clock divider
    *  Setup the retune an VAS state machine
    *  Setyo the VCO selection 
    *  DBR = 0
    *  R = 1
    *  RDIV2 = 1
    *  fREF = 50MHz
    *  RSET = 5100
    *  ICP = 1.63/RSET(1+CP) = 0.32mA (Fast lock)
    */
    long CP = (long)0b1111<<9;
    long R = (long)0b0000000000001<<14;
    reg2Default = LD_FAST|SDN_LN|MUX_TRI|R|CP|LDF_FRAC|LDP_10|PDP_POS|Reg2;
    writeRegister(reg2Default);
    Serial.println(reg2Default,HEX);
 
    
    /* Setting up register 1
    *  Setup the modulus and phase value
    *  Set the charge pump settings
    */
    long M = (long)125<<3;
    long P = (long)6<<15;
    reg1Default = CPL_EN|CPT_NORM|P|M|Reg1;
    writeRegister(reg1Default);
    Serial.println(reg1Default,HEX);

    
    /* Setting up register 0
    *  Setup the fractional and intege division values
    */
    long FRAC = (long)0<<3;
    long N = (long)64<<15;
    reg0Default = INT_FRAC|N|FRAC|Reg0;
    writeRegister(reg0Default);  
    Serial.println(reg0Default,HEX);  
}

/*Writes the read request to the device and returns the result*/
long readRegister()
{
    //Send read request
    writeRegister(Reg6);
    
    //Clock to get data
    long data = 0;
    byte i = 0;
    for(i = 0;i < 32;i++)
    {
       //Throw clk and wait some time      
      digitalWrite(CLKPin,HIGH);
      delay(1);
      digitalWrite(CLKPin,LOW);
      delay(1);
      
      //Set pin state to MSB in stream data
      if(digitalRead(MUXPin))
         data = data|0b10000000000000000000000000000000;
      else
         data = data|0b00000000000000000000000000000000;
      
      //Shift data to next bit for following read
      data = data>>1;     
     
    }
    
    return data;   
}

//Writes a given register with the given data
void writeRegister(unsigned long data)
{
    //Set clock low
    digitalWrite(CLKPin,LOW);

    //Disable latch until shift register is full
    digitalWrite(LEPin,LOW);
    
    //Define data stream
    unsigned long buffer = 0;
    
    //Clock 32 bits of data
    byte i = 0;
    for(i = 0;i < 32;i++)
    {
      //Set pin state to MSB in stream data
      buffer = data&0b10000000000000000000000000000000;
      if(buffer)
         digitalWrite(DATAPin,HIGH);
      else
         digitalWrite(DATAPin,LOW);
      
      //Shift data to next bit for following transfer
      data = data<<1;
      
      //Wait some time and throw clk
      //delay(1);
      digitalWrite(CLKPin,HIGH);
      //delay(1);
      digitalWrite(CLKPin,LOW);

    }
    
    //Latch data through to register
    digitalWrite(LEPin,HIGH);
    //delay(1);
}

//Calculate registers settings DIVA, N, F and M for a given frequency, frequency is specified in cHz
boolean setRegistersForFrequency(long frequency)
{
    long DIVA = calculateDIVA(frequency);
    long VCO = calculateVCO(frequency,DIVA);
    setNFM(VCO,5000000);
    setDIVA(DIVA);
    
    return true; //Register settings were succesfully set
   
    return false; //Register settings could not be set 
}

//Sets DIVA in the register bank
int setDIVA(long diva)
{
    long mask = (long)0b11111111100011111111111111111111;
    long DIVA = (long)diva<<20;
    //Clear current diva values
    reg4Default = reg4Default&mask;
    reg4Default = reg4Default|DIVA;
    writeRegister(reg4Default);  
    Serial.println(reg4Default,HEX);
}

//Returns the appropriate DIVA value according a table, a negative value indicates an error
int calculateDIVA(long frequency)
{
    long maxFreq = 600000000;
    long minFreq = 2350000;
    int i = 0;
    
    //Check if frequency is within ranges
    if (frequency > maxFreq || frequency < minFreq)
      return -1;
    
    //Check outlying case
    if (frequency == maxFreq)
      return 0;
 
    //Loop through frequency ranges to select divider

    while(true)
    {
       if(frequency >= maxFreq/2 && frequency < maxFreq)
         return i;
       i++;
       maxFreq = maxFreq/2; 
    }
}

//Calculates the VCO frequency
long calculateVCO(long frequency,int diva)
{
   return ceil((frequency)*(pow(2,diva))); 
}

//Calculates the N, F and M terms returns true on success
boolean setNFM(long vco,long pfd)
{
    int f;
    int m;
    double error = 0.0001;
    double rhs = (((double)vco)/(double)pfd);
    
    //Get N as the whole integer of the rhs
    int n = floor(rhs);
  
    double x = ((double)rhs - (double)n);
    
    
    //Check if the error is still higher than specified
    if(x < error)
    {
       f = 0;
       m = 125; 
    }
    else if(1-error < x)
    {
       f = 0;
       m = 125; 
       n = n+1;
    }
    else
    {    

      //Get F and M using continued fractions 
      int lower_n = 0;
      int lower_d = 1;
      int upper_n = 1;
      int upper_d = 1;
      
      //Do a binary search to find the fraction
      int middle_n;
      int middle_d;
      while(true)
      {
          middle_n = lower_n + upper_n;
          middle_d = lower_d + upper_d;
          
          if (middle_d*(x+error)<middle_n)
          {
              upper_n = middle_n;
              upper_d = middle_d; 
              //Serial.println("Going down");
          }
          else if(middle_n < (x-error)*middle_d)
          {
             lower_n = middle_n;
             lower_d = middle_d; 
          }
          else
          {

            break;
          }
        
      }

      
      f = middle_n;
      m = middle_d;
    }
    

    

    
    //Check if denominator and numerator values are within limits
    if(n < 19 || n > 4091)
      return false;
    if(m < 2 || m > 4095)
      return false;

    //Write these values to the appropriate registers      
    long M = (long)m<<3;
    long P = (long)6<<15;
    writeRegister(CPL_EN|CPT_NORM|P|M|Reg1);
    Serial.println(CPL_EN|CPT_NORM|P|M|Reg1,HEX);
    long FRAC = (long)f<<3;
    long N = (long)n<<15;
    writeRegister(INT_FRAC|N|FRAC|Reg0);  
    Serial.println(INT_FRAC|N|FRAC|Reg0,HEX);
    return true;
    
}
