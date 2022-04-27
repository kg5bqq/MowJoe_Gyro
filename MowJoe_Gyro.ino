/*
 * MowJoe Magnometer located as far as possible from steel chassis and communicates
 * to the MowJoe Master via ESPNow protocol.
 * This does not require a router bu in the event we want to run a server on MowJoe Master,
 * we need to find out which WiFi channel the router has assigned to the web client and hence
 * also to MowJoe Master.
 * If none is found (ie no router connection) we'll default both MowJoe Master and this ESPNow
 * unit to channel '1'.
 *
*/

#include "Mowjoe_Gyro.h"

#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BNO08x_RVC.h"

#include <tgmath.h>
#include <RunningMedian.h>


#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "freertos/portmacro.h"

#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_now.h"
#include "esp_crc.h"




enum Commands{START,STOP,REPEAT,FREQ,REQUIRE_ACK};

volatile int Begin_Sample_Start_Time = 0;

volatile double Input;

volatile double _heading = 0;
volatile double _position = 0;

volatile bool start_sending = false;

//*********************** Quazi-SWAG of Magnetic Bias for  BNO08x Sensor Hub ******************

Adafruit_BNO08x_RVC rvc = Adafruit_BNO08x_RVC();

RunningMedian samples = RunningMedian(21);

TaskHandle_t gyro_task_handle;


// Define variables to store incoming readings
char incomingCommand[ESPNOW_MAX_COMMAND_LEN] = {0};
float incomingTemp;
float incomingHum;
float incomingPres;

// Variable to store if sending data was successful
String success;



float filter_heading(double x) {
  samples.add(x);
    float l = samples.getLowest();
    float m = samples.getMedian();
    float a = samples.getAverage(5);
    float h = samples.getHighest();
 return m; //Return the Lowest Rnning sample collection

}


char *command_arg[10][10] = {};
String user_command[20];

int parse_command(char* cmd_sent)
{

  int i = 0;
  int j = 0;
  char delimiters[] = "=:,";
  int ret_val = 0;


  char* token;
  char* rest = cmd_sent;
  enum Commands command;

	  while ((token = strtok_r(rest, delimiters, &rest))) {
			 //printf("--->%s\n\r", token);
			 user_command[i++] = token;

	  }

	  for(j=0;j<i;j++)
	   Serial.printf("user_command[%d]%s\n\r",j, user_command[j]);


   if(user_command[0].indexOf("START") >= 0) {


	   start_sending = true;
	   if(user_command[1].indexOf("YES") >= 0) {
		   Begin_Sample_Start_Time = user_command[2].toInt();
		   Serial.printf("%s %s %d\n\r", user_command[0], user_command[1], Begin_Sample_Start_Time);
				start_sending = true;
	   }


   }

   if(user_command[0].indexOf("STOP") >= 0) {
	   start_sending = false;
   }

   return ret_val;

}



// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  Serial.print("Bytes received: ");
  Serial.println(len);
  //Now copy in the latest..
  memcpy(incomingCommand,incomingReadings.command,sizeof(incomingCommand));

  parse_command(incomingReadings.command);

  incomingTemp = incomingReadings.temp;
  incomingHum = incomingReadings.bearing;
  incomingPres = incomingReadings.heading;
  Serial.printf("incomingReadings.heading = %lf\n\r", incomingReadings.heading);
}

//For some strange reason this next declaration needs to be located here!
esp_now_peer_info_t peerInfo;

void setup() {
  // Init Serial Monitor

	pinMode(RED_LED_PIN, OUTPUT);
	pinMode(GREEN_LED_PIN, OUTPUT);
	pinMode(BLUE_LED_PIN, OUTPUT);
	digitalWrite(RED_LED_PIN, LOW);
	digitalWrite(GREEN_LED_PIN, LOW);
	digitalWrite(BLUE_LED_PIN, LOW);

  Serial.begin(115200);
  Serial2.begin(115200); // This is the baud rate specified by the datasheet

  while (!Serial2)
    delay(10);

  if (!rvc.begin(&Serial2)) { // connect to the sensor over hardware serial
    while (1)
      delay(10);
  }

 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    digitalWrite(RED_LED_PIN, HIGH);
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer1
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);		//MowJoe_Master
//  memcpy(peerInfo.peer_addr, broadcastAddress2, 6);		//MowJoe_MotorController
  peerInfo.channel = 1;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer1");
    digitalWrite(RED_LED_PIN, HIGH);
    return;
  }

  // Register peer2
  memcpy(peerInfo.peer_addr, broadcastAddress2, 6);		//MowJoe_MotorController
  peerInfo.channel = 1;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer2");
    digitalWrite(RED_LED_PIN, HIGH);
    return;
  }

  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

  digitalWrite(BLUE_LED_PIN, HIGH);

  xTaskCreatePinnedToCore(gyro_task, "gyro_task"  , 2048 //4096 //2048
    , NULL, 7         // Priority High to Low=0.
    , &gyro_task_handle, ARDUINO_RUNNING_CORE);

}
 
void loop() {
}



void gyro_task(void *arg) {
float corrected_yaw = 0;
double _position = 0;
BNO08x_RVC_Data heading;

  volatile bool ok_to_Send = false;
  Serial.printf("Gyro Task Started\n\r");
  digitalWrite(BLUE_LED_PIN, HIGH);
  
  while(1) {

	  if (!rvc.read(&heading)) {
	    // Data not available or parsable, keep trying
		  vTaskDelay(50 / portTICK_RATE_MS);
	  } else {

//	  Serial.print(heading.yaw);Serial.println(F(","));
	  //Add 'magnetic north' offset for current location.
	  double declinationAngle = 0.05235988; //0.22;  //LAKE DALLAS - TX on 7/27/21 jab
	  corrected_yaw = (heading.yaw + declinationAngle);
	  //              //Send it to the Filter.  _position will now contains the filtered collection of headings
	  //              double temp_pos = filter_heading(heading.yaw);
	  //              _heading = temp_pos;
	  //              _position = nearbyint(temp_pos);  // Remove the decimal degrees (rounded out)
	  //              Input = _position;//'Input comes out of the filter_heading() filter and goes to the PID


	  if(corrected_yaw < 0)
	  	_position = (360 + corrected_yaw);
	  else
	  	_position = corrected_yaw;


      outgoingReadings.bearing = heading.yaw;
      outgoingReadings.heading = _position;

//	  Serial.print(heading.pitch);Serial.println(F(","));
//	  Serial.print(heading.roll);Serial.println(F(","));
//	  Serial.print(heading.x_accel);Serial.print(F(","));
//	  Serial.print(heading.y_accel);Serial.print(F(","));
//	  Serial.print(heading.z_accel);
//	  Serial.println("");
      digitalWrite(RED_LED_PIN, LOW);
      digitalWrite(GREEN_LED_PIN, LOW);
//
              // Correct for when signs are reversed.
//              if(heading.yaw < 0)
//                corrected_yaw = heading.yaw += 2*PI;
//
//              // Check for wrap due to addition of declination.
//              if(heading.yaw > 2*PI)
//            	  corrected_yaw = heading.yaw -= 2*PI;


     	  	  //Send it to the Filter.  _position will now contains the filtered collection of headings
              //_position = nearbyint(corrected_yaw);  // Remove the decimal degrees (rounded out)



              Serial.printf("corrected_yaw = %lf\n\r",_position);

              //
//
//            // Send message via ESP-NOW
            outgoingReadings.sensorID = MOWJOE_GYRO;
//
            ok_to_Send = start_sending;

            if(ok_to_Send == true) {
                digitalWrite(RED_LED_PIN, LOW);
                digitalWrite(BLUE_LED_PIN, LOW);
                digitalWrite(GREEN_LED_PIN, HIGH);

            	esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoingReadings, sizeof(outgoingReadings));		//Send to MowJoe_Master
            	if (result == ESP_OK) {
            	  Serial.println("Sent with success");
            	}
            	else {
            	  Serial.println("Error sending the data");
            	}
///*
//            	esp_err_t result2 = esp_now_send(broadcastAddress2, (uint8_t *) &outgoingReadings, sizeof(outgoingReadings));	//Send to MowJoe_MotorControl
//            	if (result2 == ESP_OK) {
//            	  Serial.println("Sent with success");
//            	}
//            	else {
//            	  Serial.println("Error sending the data");
//            	}
//*/
//
////            	Serial.printf("SENT -> outgoingReadings.heading: %lf\n\r", outgoingReadings.heading);
//            } else {
//                digitalWrite(RED_LED_PIN, HIGH);
//                digitalWrite(BLUE_LED_PIN, LOW);
//                digitalWrite(GREEN_LED_PIN, LOW);
//            }
            vTaskDelay(50 / portTICK_RATE_MS); //Normal/default max sample rate (continious mode of HMC5883L) is 66ms.
            }
	  	  }
      } // End of while(1) loop

    //Should NEVER get here..
    vTaskDelete( gyro_task_handle );

} // End of gyro_task
