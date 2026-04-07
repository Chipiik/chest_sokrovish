#include <arduinoFFT.h>
arduinoFFT FFT = arduinoFFT();
#define size 64
#define RED 8
#define GRN 12
#define BLU 13
#include <SoftwareSerial.h>
#include <Servo.h>
Servo mas;
unsigned long last_time_beta_found;
bool beta_found_flag;
SoftwareSerial piSerial(10, 11);

void uniorBegin(/*const int channel, int number_of_channels*/)
{
  piSerial.begin(57600);
  piSerial.write(0xff);
  while (true)
  {
    String s = piSerial.readString();
    if (s == "OK\n")
    {
      String s;
      s += 0;
      s += ";";
      s += 3;
      piSerial.println(s);
      break;
    }
  }
}

bool uniorRead(uint8_t channel_no, float& value)
{
  while (piSerial.available()) piSerial.read();
  piSerial.write(channel_no);
  int readCnt = piSerial.readBytes((byte*)&value, sizeof(float));
  return (readCnt == sizeof(float)) && (value == value);
}

const int cEEG1 = 0;
const int cEMG1 = 3;

double M[64];
double O[64];

void setup() {
  // put your setup code here, to run once:
  last_time_beta_found = millis();
  mas.attach(9);
  mas.write(0);
  Serial.begin(115200);
//  Serial.print("Serial port started with 115200");
  uniorBegin();
  pinMode(RED, OUTPUT);
  pinMode(GRN, OUTPUT);
  pinMode(BLU, OUTPUT);
  for ( int i = 0; i < 64; i++) {
    O[i] = 0;
  }

}


int i = 0;
double mx = M[0];
int u = 0;
void loop() {
  // put your main code here, to run repeatedly:

  float v;
  float w;
  if (uniorRead(cEEG1, v) && uniorRead(cEMG1,w)) //(uniorRead(cEMG1,w)) 
  {
    Serial.print(w);   
    Serial.print('\t');
    Serial.print(beta_found_flag * 1000);
    Serial.print('\t'); 
    Serial.println(v);  

    M[i] = v;
    i++;
    if(w<20 and u==0)
    {
      for(int a=0; a<70; )
      {
        if(millis()-last_time_beta_found>150)
        {
          mas.write(a);
          a+=1;
          // last_time_beta_found=millis();
        }
      }
      u=90;
    }
    else
    {
      if(w>20 and u==90)
      {
        for( int g=70; g!=0; )
        {
          if(millis()-last_time_beta_found>70)
          {
            mas.write(g);
            g-=1;
            // last_time_beta_found=millis();
          }
        }
        u=0;
      }
    }
 
    if(i==64)
    {
      FFT.Compute(M, O, 64, FFT_FORWARD);
      FFT.ComplexToMagnitude(M, O, 64);
      for(int p = 1; p<64; p++)
      {
        if (M[p] > mx)
        {
          mx = M[p];
        }
      }
      beta_found_flag = false;
      for(int e = 16; e<32; e++)
      {
        if(M[e]>= 0.6 * mx)
        {
          beta_found_flag = true;
          break;
        }
      }
      if(beta_found_flag)
      {
        // last_time_beta_found = millis();
        Serial.println("-10000\t-10000\t-10000");
        analogWrite(RED, 0);
        analogWrite(GRN, 255);
        analogWrite(BLU, 150);
        last_time_beta_found = millis();
      }
      else if(millis()-last_time_beta_found > 1300)
      {
        analogWrite(RED, 255);
        analogWrite(GRN, 0);
        analogWrite(BLU, 0);
      }
    
      // for (int x = 0; x < 64; x++) O[x] = 0;
      i = 0;
    }  
  }
  delay(8);
}
