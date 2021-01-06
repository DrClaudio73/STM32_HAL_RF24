
/*
  Getting Started example sketch for nRF24L01+ radios
  This is a very basic example of how to send data from one node to another
  Updated: Dec 2014 by TMRh20
*/

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"


/****************** User Config ***************************/
/***      Set this radio as radio number 0 or 1         ***/
bool radioNumber = 0;

/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8 */
RF24 radio(7, 8);
//HardwareSerial pc(PA10,PA9); //CE, CSN

/**********************************************************/

byte addresses[][6] = {"1Node", "2Node"};

// Used to control whether this node is sending or receiving
bool role = 0;
#define NUM_BYTES 32
uint8_t tx[NUM_BYTES];

void setup() {
  for (int i = 0 ; i < NUM_BYTES; i++) {
    tx[i] = 0;
  }

  Serial.begin(115200);
  printf_begin();
  

  radio.begin();

  // Set the PA Level low to prevent power supply related issues since this is a
  // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  radio.setAutoAck(1);                    // Ensure autoACK is enabled
  //radio.enableAckPayload();               // Allow optional ack payloads
  radio.setDataRate(RF24_2MBPS);
  radio.setPALevel(RF24_PA_HIGH);
  
  radio.setRetries(15, 15);                // Smallest time between retries, max no. of retries
  radio.setPayloadSize(NUM_BYTES);                // Here we are sending 1-byte payloads to test the call-response speed

  // Open a writing and reading pipe on each radio, with opposite addresses
  if (radioNumber) {
    radio.openWritingPipe(addresses[1]);
    radio.openReadingPipe(1, addresses[0]);
  } else {
    radio.openWritingPipe(addresses[0]);
    radio.openReadingPipe(1, addresses[1]);
  }

  // Start the radio listening for data
  radio.startListening();
  radio.printDetails();                   // Dump the configuration of the rf unit for debugging

  Serial.println(F("RF24/examples/GettingStarted on NANO"));
  Serial.println(F("*** PRESS 'T' to begin transmitting to the other node"));
  radio.printPrettyDetails();
}


void printdata(uint8_t* dati) {
  for (int i = 0 ; i < NUM_BYTES; i++) {
    Serial.print(dati[i]);
    if (i < (NUM_BYTES - 1)) {
      Serial.print(" -");
    }
  }
  Serial.println(" ");
}

void loop() {


  /****************** Ping Out Role ***************************/
  if (role == 1)  {

    radio.stopListening();                                    // First, stop listening so we can talk.


    Serial.println(F("Now sending"));
    tx[0] = tx[0] + 1;
    for (int i = 1; i < NUM_BYTES; i++) {
      tx[i] = tx[i - 1] + 1;
    }
    printdata(tx);

    unsigned long start_time = micros();                             // Take the time, and send it.  This will block until complete

    if (!radio.write( &tx, NUM_BYTES )) {
      Serial.println(F("failed"));
    } else {
      Serial.println(F("sent OK!"));
    }

    radio.startListening();                                    // Now, continue listening

    unsigned long started_waiting_at = micros();               // Set up a timeout period, get the current microseconds
    boolean timeout = false;                                   // Set up a variable to indicate if a response was received or not

    while ( ! radio.available() ) {                            // While nothing is received
      if (micros() - started_waiting_at > 200000 ) {           // If waited longer than 200ms, indicate timeout and exit while loop
        timeout = true;
        break;
      }
    }

    if ( timeout ) {                                            // Describe the results
      Serial.println(F("Failed, response timed out."));
    } else {
      printf("Success, time out has not been triggered. \r\n");
      uint8_t rx[NUM_BYTES];                                 // Grab the response, compare, and send to debugging spew
      for (int i = 0 ; i < NUM_BYTES; i++) {
        rx[i] = 0;
      }
      radio.read( &rx, NUM_BYTES );
      printf("Sent:\t\t");
      printdata(tx);
      printf("Got response:\t");
      printdata(rx);
    }

    // Try again 1s later
    //delay(100);
  }



  /****************** Pong Back Role ***************************/

  if ( role == 0 )
  {
    unsigned long got_time;
    uint8_t rx[NUM_BYTES];                                 // Grab the response, compare, and send to debugging spew
    for (int i = 0 ; i < NUM_BYTES; i++) {
      rx[i] = 0;
    }

    if ( radio.available()) {
      // Variable for the received timestamp
      while (radio.available()) {                                   // While there is data ready
         radio.read( &rx, NUM_BYTES );       // Get the payload
        }
        radio.stopListening();                                        // First, stop listening so we can talk
        radio.write( &rx, NUM_BYTES );              // Send the final one back.
        radio.startListening();                                       // Now, resume listening so we catch the next packets.
        printf("Sent response: ");
        printdata(rx);
      }
  }        
        
    char c=toupper(Serial.read());
    if ( c == 'T' && role == 0 ) {
      radio.flush_tx();
      radio.flush_rx();
      Serial.println(F("*** CHANGING TO TRANSMIT ROLE -- PRESS 'R' TO SWITCH BACK"));
      role = 1;                  // Become the primary transmitter (ping out)
      tx[0] = 0;

    } else if ( c == 'R' && role == 1 ) {
      radio.flush_tx();
      radio.flush_rx();
      Serial.println(F("*** CHANGING TO RECEIVE ROLE -- PRESS 'T' TO SWITCH BACK"));
      role = 0;                // Become the primary receiver (pong back)
      tx[0] = 0;
      radio.startListening();
    } else {
      if ( c == 'P'  ) {
        radio.flush_tx();
        radio.flush_rx();
        Serial.println(F("*** PRINTIG DEBUG INFO ***"));
        radio.printDetails();                   // Dump the configuration of the rf unit for debugging
        Serial.println(F("*** ***** ***"));
        radio.printPrettyDetails();

      }
    }
} // Loop
