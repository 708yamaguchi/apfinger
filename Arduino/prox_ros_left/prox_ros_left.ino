#include <ros.h>
#include <Wire.h>
#include <APFinger/Proximities.h>

#define WIRE Wire

/***** GLOBAL CONSTANTS *****/

#define VCNL4040_ADDR 0x60 //7-bit unshifted I2C address of VCNL4040

//Command Registers have an upper byte and lower byte.
#define PS_CONF1 0x03
//#define PS_CONF2 //High byte of PS_CONF1
#define PS_CONF3 0x04
//#define PS_MS //High byte of PS_CONF3
#define PS_DATA_L 0x08
//#define PS_DATA_M //High byte of PS_DATA_L
#define ID 0x0C


#define NSENSORS 36
#define LOOP_TIME 100  // loop duration in ms

/***** ROS *****/
ros::NodeHandle  nh;
APFinger::Proximities proximity_msg;
ros::Publisher proximity_sensor_pub("proximity_sensors/left/all", &proximity_msg);
unsigned int proximities[NSENSORS];

int ChgI2CMultiplexer(unsigned char adrs,unsigned char ch)
{
  unsigned char c;
  Wire.beginTransmission(adrs);
  c = ch & 0x07;
  c = c | 0x08;
  Wire.write(c);
  return  Wire.endTransmission();
}

void measure_proximity(unsigned char adrs, int nsensors, int start_num)
{
  int i;
  // Start Multiplexer and Proximity Sensor
  for(i=0;i<nsensors;i++)
  {
    // nh.loginfo("first");
    delayMicroseconds(500); // maybe this can be shortened
    ChgI2CMultiplexer(adrs,i);
    startProxSensor();
  }

  // Measure Proximity and Stop Proximity Sensor
  for(i=0;i<nsensors;i++)
  {
    // nh.loginfo("second");
    delayMicroseconds(500); // maybe this can be shortened
    ChgI2CMultiplexer(adrs, i);
    proximities[i + start_num] = readFromCommandRegister(PS_DATA_L);
    stopProxSensor();
  }

  // Stop Multiplexer
  Wire.beginTransmission(adrs);
  Wire.write(0x00);
  Wire.endTransmission();

  // Set ROS msg
  proximity_msg.proximities = proximities;
  proximity_msg.proximities_length = NSENSORS;
}

void initVCNL4040()
{
  startProxSensor();

  delay(1);
  //Set the options for PS_CONF3 and PS_MS bytes
  //Set IR LED current to 75mA
  writeToCommandRegister(PS_CONF3, 0x00, 0b00000001);
}

void startProxSensor()
{
  //Clear PS_SD to turn on proximity sensing
  //Integrate 8T, Clear PS_SD bit to begin reading
  //Set PS to 16-bit
  writeToCommandRegister(PS_CONF1, 0b00001110, 0b00001000); //Command register, low byte, high byte
}

void stopProxSensor()
{
  //Set PS_SD to turn off proximity sensing
  //Set PS_SD bit to stop reading
  writeToCommandRegister(PS_CONF1, 0b00000001, 0b00000000); //Command register, low byte, high byte
}

//Reads a two byte value from a command register
unsigned int readFromCommandRegister(byte commandCode)
{
  Wire.beginTransmission(VCNL4040_ADDR);
  Wire.write(commandCode);
  Wire.endTransmission(false); //Send a restart command. Do not release bus.

  Wire.requestFrom(VCNL4040_ADDR, 2); //Command codes have two bytes stored in them

  unsigned int data = Wire.read();
  data |= Wire.read() << 8;

  return (data);
}

//Write a two byte value to a Command Register
void writeToCommandRegister(byte commandCode, byte lowVal, byte highVal)
{
  Wire.beginTransmission(VCNL4040_ADDR);
  Wire.write(commandCode);
  Wire.write(lowVal); //Low byte of command
  Wire.write(highVal); //High byte of command
  Wire.endTransmission(); //Release bus
}

void InitI2CMultiplexer(unsigned char adrs, int nsensors)
{
  int i;
  for(i=0;i<nsensors;i++)
  {
    delay(1);

    ChgI2CMultiplexer(adrs,i);
    initVCNL4040(); //Configure sensor

  }
  // Stop Multiplexer
  Wire.beginTransmission(adrs);
  Wire.write(0x00);
  Wire.endTransmission();

}

void setup()
{
  // setBaud(57600) for fetch's USB port
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(proximity_sensor_pub);
  while(!nh.connected())
  {
    nh.spinOnce();
  }
  // nh.loginfo("start setup");
  Wire.begin();
  // several multiplexer
  delay(10);
  InitI2CMultiplexer(0x70, 4);
  delay(10);
  InitI2CMultiplexer(0x71, 8);
  delay(10);
  InitI2CMultiplexer(0x72, 8);
  delay(10);
  InitI2CMultiplexer(0x73, 8);
  delay(10);
  InitI2CMultiplexer(0x74, 8);
  delay(10);
}


void loop()
{
  // nh.loginfo("loop start left");
  unsigned long time;
  time = millis();
  // several multiplexer
  measure_proximity(0x70, 4, 0);
  measure_proximity(0x71, 8, 4);
  measure_proximity(0x72, 8, 12);
  measure_proximity(0x73, 8, 20);
  measure_proximity(0x74, 8, 28);
  proximity_sensor_pub.publish(&proximity_msg);
  while (millis() < time + LOOP_TIME); // enforce constant loop time
  nh.spinOnce();
}
