/****************************************************************
 * Takes the readings from all sensors in femtofarads, puts them in an 
 *      array, and sends the array over the topic 
 * Note: FDC library has been modified from the default installation
 ****************************************************************/

#include <Wire.h>
#include <Protocentral_FDC1004.h>
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/UInt16.h>
#include <wholearm_skin_ros/TaxelData.h>

#define TCAADDR1 0x77 // 1st link multiplexer 111
#define TCAADDR2 0x74 // 2nd link multiplexer 100
#define TCAADDR3 0x71 // 3rd link multiplexer 001
#define TCAADDR4 0x72 // 4th link multiplexer 010

/*
 I2C Address
// R1 R2 R3
// 0  0  0   0x70
// 0  0  1   0x71
// 0  1  0   0x72
// 0  1  1   0x73
// 1  0  0   0x74
// 1  0  1   0x75
// 1  1  0   0x76
// 1  1  1   0x77
*/

#define UPPER_BOUND  0X4000                 // max readout capacitance
#define LOWER_BOUND  (-1 * UPPER_BOUND)
#define ONEA 0 // channel
#define ONEB 1
#define TWOA 2
#define TWOB 3
#define MEASURMENT 5                       // need one unused port on each multiplexer
#define NUM_TAXELS 56

int capdac = 5;

FDC1004 FDC;

// ROS taxel interface
ros::NodeHandle nh;

wholearm_skin_ros::TaxelData msg;
ros::Publisher pub("skin/taxels", &msg);

/****************************************************************
 * HELPER FUNCTIONS
 ****************************************************************/

void tcaselect(uint8_t bus, int tcaaddress) { 
  // bus selects which of tca outputs to write to 
  if (bus > 7) return;
  
  Wire.beginTransmission(tcaaddress);
  Wire.write(1 << bus); 
  Wire.endTransmission();
}

uint16_t gettaxelreading(uint8_t channel) {
  FDC.configureMeasurementSingle(channel, channel, capdac);
  FDC.triggerSingleMeasurement(channel, FDC1004_100HZ);

  //wait for completion
  delay(15);
  uint16_t value[2];
  if (! FDC.readMeasurement(channel, value))
  {
    int16_t msb = (int16_t) value[0];
    int32_t capacitance = ((int32_t)457) * ((int32_t)msb); //in attofarads
    capacitance /= 1000;   //in femtofarads
    capacitance += ((int32_t)3028) * ((int32_t)capdac);

    return (uint16_t) capacitance;
  }
}

/****************************************************************
 * MAIN CODE
 ****************************************************************/

void setup() {
  nh.initNode();
  nh.advertise(pub);

  Wire.begin(); // join i2c bus
  Serial.begin(115200); //start serial for output
  while (!Serial);
  delay(1000);
}


void loop() {

  uint16_t c[NUM_TAXELS] = {0};

  float measurement = 0;

  /* MULTIPLEXER 1 */
  tcaselect(MEASURMENT, TCAADDR4);
  
  tcaselect(0, TCAADDR1);  //scl0 sda0
  c[0] = gettaxelreading( ONEB );
  c[1] = gettaxelreading( TWOB );

  tcaselect(3, TCAADDR1); //scl3 sda3
  c[2] = gettaxelreading( TWOB );
  c[3] = gettaxelreading( ONEB );

  tcaselect(1, TCAADDR1); //scl1 sda1
  c[4] = gettaxelreading( ONEB );
  c[5] = gettaxelreading( ONEA );
  c[6] = gettaxelreading( TWOB );
  c[7] = gettaxelreading( TWOA );

  tcaselect(2, TCAADDR1); //scl2 sda2
  c[8] = gettaxelreading( TWOA );

    /* MULTIPLEXER 2 */
  tcaselect(MEASURMENT, TCAADDR1);
  
  tcaselect(3, TCAADDR2); //scl3 sda3
  c[9] = gettaxelreading( TWOB );
  c[10] = gettaxelreading( ONEB );

  tcaselect(4, TCAADDR2);
  c[11] = gettaxelreading( TWOB );
  c[12] = gettaxelreading( ONEB );

  tcaselect(0, TCAADDR2);  //scl0 sda0
  c[13] = gettaxelreading(ONEB);
  c[15] = gettaxelreading(TWOB);

  tcaselect(1, TCAADDR2); //scl1 sda1
  c[14] = gettaxelreading( TWOB );
  c[16] = gettaxelreading( ONEB );
  
  tcaselect(2, TCAADDR2); //scl2 sda2
  c[17] = gettaxelreading( ONEA );

  /* MULTIPLEXER 3*/
  tcaselect(MEASURMENT, TCAADDR2);

  tcaselect(4, TCAADDR3); 
  c[18] = gettaxelreading( TWOA );
  
  tcaselect(0, TCAADDR3);  //scl0 sda0
  c[19] = gettaxelreading( TWOA );
  c[20] = gettaxelreading( ONEA );

  tcaselect(7, TCAADDR3); //scl1 sda1
  c[21] = gettaxelreading( ONEA );
  c[22] = gettaxelreading( ONEB );
  c[23] = gettaxelreading( TWOA );
  c[24] = gettaxelreading( TWOB );

  tcaselect(2, TCAADDR3); //scl2 sda2
  c[25] = gettaxelreading( TWOA );
  c[26] = gettaxelreading( ONEA );
  c[27] = gettaxelreading( TWOB);
  c[28] = gettaxelreading( ONEB );

  tcaselect(6, TCAADDR3); //scl3 sda3
  c[29] = gettaxelreading( ONEB );
  c[30] = gettaxelreading( TWOB );
  c[31] = gettaxelreading( ONEA);
  c[32] = gettaxelreading( TWOA );
  
  tcaselect(1, TCAADDR3); //scl1 sda1
  c[33] = gettaxelreading( TWOA );
  c[34] = gettaxelreading( ONEA );
  c[35] = gettaxelreading( TWOB );
  c[36] = gettaxelreading( ONEB );

  /* MULTIPLEXER 4*/
  tcaselect(MEASURMENT, TCAADDR3);
  
  tcaselect(0, TCAADDR4);  //scl0 sda0
  c[37] = gettaxelreading( TWOB );
  c[38] = gettaxelreading( ONEA );
  c[39] = gettaxelreading( TWOA );

  tcaselect(1, TCAADDR4); //scl1 sda1
  c[40] = gettaxelreading( ONEA );
  c[41] = gettaxelreading( ONEB );
  c[42] = gettaxelreading( TWOA );
  c[43] = gettaxelreading( TWOB );

  tcaselect(2, TCAADDR4); //scl2 sda2
  c[44] = gettaxelreading( TWOA );
  c[45] = gettaxelreading( TWOB );
  c[46] = gettaxelreading( ONEB );
  c[47] = gettaxelreading( ONEA );

  tcaselect(3, TCAADDR4); //scl3 sda3
  c[48] = gettaxelreading( ONEA );
  c[49] = gettaxelreading( ONEB );
  c[50] = gettaxelreading( TWOB );
  c[51] = gettaxelreading( TWOA );

  tcaselect(4, TCAADDR4); //scl4 sda4
  c[52] = gettaxelreading( TWOA );
  c[53] = gettaxelreading( TWOB );
  c[54] = gettaxelreading( ONEB );
  c[55] = gettaxelreading( ONEA );

  msg.cdc = c;
  msg.cdc_length = NUM_TAXELS;
  msg.header.stamp = nh.now();

  pub.publish(&msg);
  nh.spinOnce();
  
} // end loop
