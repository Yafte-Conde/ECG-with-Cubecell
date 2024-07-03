#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <Wire.h>
#include "ADS1X15.h"

#ifndef LoraWan_RGB
#define LoraWan_RGB 0
#endif

#define RF_FREQUENCY                                915000000 // Hz

#define TX_OUTPUT_POWER                             14        // dBm

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       12         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false


#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 30 // Define the payload size here

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];
ADS1115 ADS(0x48);

static RadioEvents_t RadioEvents;
void OnTxDone( void );
void OnTxTimeout( void );

typedef enum
{
  //LOWPOWER,
  //RX,
  ReadECG,
  TX
}States_t;

bool lora_idle=true;

int16_t txnumber;
States_t state;
bool sleepmode = false;
int16_t RSSI,rxSize;

void setup() {
  Serial.begin(2400);
  Wire.begin();
  ADS.begin();
  // txnumber=0;
  
  RSSI=0;
  
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;

    Radio.Init( &RadioEvents );
    Radio.SetChannel( RF_FREQUENCY );
    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );

    state=TX;
  
}

void loop() {
  //appDataSize = 0;
  ADS.setGain(0);
  float f = ADS.toVoltage(1);  //  voltage factor
  int16_t val_0 = ADS.readADC(0);

  switch(state){
    case TX:

      sprintf(txpacket,"%s","val_0: ");
      sprintf(txpacket+strlen(txpacket),"%d",val_0);
      // sprintf(txpacket+strlen(txpacket),"%s"," rssi : ");
      // sprintf(txpacket+strlen(txpacket),"%d",RSSI);
      turnOnRGB(COLOR_SEND,0);
      
      Serial.printf("\r\nsending packet \"%s\" , length %d\r\n",txpacket, strlen(txpacket));

      Radio.Send( (uint8_t *)txpacket, strlen(txpacket) );
		  state=ReadECG;
		break;
/*
    case LOWPOWER:
			lowPowerHandler();
      Serial.println("Low Power");
      delay(100);
      state = ReadECG;
		break;
*/
    case ReadECG:
      int16_t val_ECG = val_0;
      Serial.println(" "); Serial.print("The signal is: "); Serial.print(val_0 * f); Serial.println(" ");
      Serial.print("\tAnalog0: "); Serial.print(val_ECG); Serial.print('\t'); Serial.println(val_ECG);
      state = TX;
    break; 

    //default:
    //      break;
  }
}

void OnTxDone( void )
{
	Serial.print("TX done......");
	turnOnRGB(0,0);
	state=TX;
}

void OnTxTimeout( void )
{
    Radio.Sleep( );
    Serial.print("TX Timeout......");
    state=TX;
}
