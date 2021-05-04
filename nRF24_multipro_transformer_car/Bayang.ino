/*
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License.
 If not, see <http://www.gnu.org/licenses/>.
 */

#define BAYANG_BIND_COUNT       1000
#define BAYANG_PACKET_PERIOD    1000
#define BAYANG_INITIAL_WAIT    500  
#define BAYANG_PACKET_SIZE      15
#define BAYANG_RF_NUM_CHANNELS  4
#define BAYANG_RF_BIND_CHANNEL  0
#define BAYANG_ADDRESS_LENGTH   5

#define CH_FLIP 0
#define CH_EXPERT 1
#define CH_HEADFREE 2
#define CH_RTH 3
#define CH_INV 4

static uint8_t Bayang_rf_chan = 0;
static uint8_t Bayang_rf_channels[BAYANG_RF_NUM_CHANNELS] = {0,};
static uint8_t Bayang_rx_tx_addr[BAYANG_ADDRESS_LENGTH];
//char trims[4];
uint32_t lastRxTime;
bool timingFail;
uint8_t skipChannel;

enum{
    // flags going to packet[2]
    BAYANG_FLAG_RTH      = 0x01,
    BAYANG_FLAG_HEADLESS = 0x02,
    BAYANG_FLAG_FLIP     = 0x08,
    BAYANG_FLAG_VIDEO    = 0x10,
    BAYANG_FLAG_SNAPSHOT = 0x20,
};

enum{
    // flags going to packet[3]
    BAYANG_FLAG_INVERT   = 0x80,
};

void Bayang_init()
{
    const u8 bind_address[] = {0,0,0,0,0};
    NRF24L01_Initialize();
    NRF24L01_SetTxRxMode(RX_EN);
    XN297_SetTXAddr(bind_address, BAYANG_ADDRESS_LENGTH);
    XN297_SetRXAddr(bind_address, BAYANG_ADDRESS_LENGTH);
    NRF24L01_FlushTx();
    NRF24L01_FlushRx();
    NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);      // No Auto Acknowldgement on all data pipes
    NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x01);
    NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, BAYANG_PACKET_SIZE); // rx pipe 0
    NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x03);   // address size
    NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0x00); // no retransmits
    NRF24L01_SetBitrate(NRF24L01_BR_1M);             // 1Mbps
    NRF24L01_SetPower(RF_POWER);
    NRF24L01_Activate(0x73);                         // Activate feature register
    NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 0x00);      // Disable dynamic payload length on all pipes
    NRF24L01_WriteReg(NRF24L01_1D_FEATURE, 0x01);
    NRF24L01_Activate(0x73);
    delay(150);
    XN297_Configure(_BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO) | _BV(NRF24L01_00_PWR_UP) | _BV(NRF24L01_00_PRIM_RX));
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
    NRF24L01_FlushRx();
}

void Bayang_bind()
{
    digitalWrite(LED_pin, LOW);

    int bind_count = 0;
    uint8_t bind_packet[BAYANG_PACKET_SIZE] = {0};
    uint32_t timeout;

    NRF24L01_WriteReg(NRF24L01_05_RF_CH, BAYANG_RF_BIND_CHANNEL);

    while(bind_count < 10) {
        timeout = millis()+5;        

        while(millis()<timeout) {
            delay(1);
            if(NRF24L01_ReadReg(NRF24L01_07_STATUS) & _BV(NRF24L01_07_RX_DR)) { // data received from tx
                digitalWrite(LED_pin, LOW);
                XN297_ReadPayload(packet, BAYANG_PACKET_SIZE);
                
                NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
                NRF24L01_FlushRx();
                
                if( packet[0] == 0xA4)
                {
                  if (0 == bind_count)
                  {
                    memcpy(bind_packet, packet, BAYANG_PACKET_SIZE);
                    ++bind_count;
                    digitalWrite(LED_pin, HIGH);
                  }
                  else
                  {
                    if (0 == memcmp(bind_packet, packet, BAYANG_PACKET_SIZE))
                    {
                      ++bind_count;
                      digitalWrite(LED_pin, HIGH);
                    }
                  }
                }
                break;
            }
        }
    }
    digitalWrite(LED_pin, LOW);
    memcpy(Bayang_rx_tx_addr, &packet[1], 5);
    memcpy(Bayang_rf_channels, &packet[6], 4);
    transmitterID[0] = packet[10];
    transmitterID[1] = packet[11];

    XN297_SetTXAddr(Bayang_rx_tx_addr, BAYANG_ADDRESS_LENGTH);
    XN297_SetRXAddr(Bayang_rx_tx_addr, BAYANG_ADDRESS_LENGTH);

    nextChannel();
  
    delay(300);
    for(int i=0; i<5; i++) {
      digitalWrite(LED_pin, HIGH);
      delay(300);
      digitalWrite(LED_pin, LOW);
      delay(100);
    }
}

void Bayang_recv_packet(TrxData* data)
{  
    data->throttle = data->yaw = data->pitch = data->roll = 0;

    if(NRF24L01_ReadReg(NRF24L01_07_STATUS) & _BV(NRF24L01_07_RX_DR)) 
    { 
      // data received from tx
      uint32_t recTime = micros();
      int sum = 0;
      XN297_ReadPayload(packet, BAYANG_PACKET_SIZE);

      if (packet[0] == 0xA5)
      {
        // data packet
        for(int i=0; i<14; i++) 
        {
          sum += packet[i];
        }	
        
        if ((sum&0xFF) == packet[14])
        {
          // checksum OK
          data->roll = (packet[4] & 0x0003) * 256 + packet[5];
          data->pitch = (packet[6] & 0x0003) * 256 + packet[7];
          data->yaw = (packet[10] & 0x0003) * 256 + packet[11];
          data->throttle = (packet[8] & 0x0003) * 256 + packet[9];

          data->trim1 = packet[6] >> 2;
/*  
          trims[0] = packet[6] >> 2;
          trims[1] = packet[4] >> 2;
          trims[2] = packet[8] >> 2;
          trims[3] = packet[10] >> 2;
*/  
          data->invert = (packet[3] & 0x80)?1:0; // inverted flag
          data->flip = (packet[2] & 0x08) ? 1 : 0;
          data->expert = (packet[1] == 0xfa) ? 1 : 0;
          data->headfree = (packet[2] & 0x02) ? 1 : 0;
          data->rth = (packet[2] & 0x01) ? 1 : 0; // rth channel

          //all good
          lastRxTime = recTime;
          timingFail = 0;
          skipChannel = 0;

          nextChannel();
        }
        else
        {
          nextChannel();
          //check sum failed
        }
      }

      if (skipChannel < 5)
      {
        uint32_t delta = recTime - lastRxTime ;
        if (!timingFail && delta > 500 && (delta - 250)/3000 >= (skipChannel + 1) ) 
        {
          nextChannel();
          skipChannel++;
        }
      }
/*
      // sequence period 12000
      if (delta > 13000)
      {     
          //  channel with no reception   
          lastRxTime = recTime;
          // set channel to last with reception
          if (!timingFail) chan = lastrxchan;
          // advance to next channel
          nextchannel();
          // set flag to discard packet timing
          timingfail = 1;
    
      } 
      */    
   }  
}

void nextChannel(void) 
{
  Bayang_rf_chan++;
  Bayang_rf_chan %= sizeof(Bayang_rf_channels);
  NRF24L01_WriteReg(NRF24L01_05_RF_CH, Bayang_rf_channels[Bayang_rf_chan]);
  NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
  NRF24L01_FlushRx();  
}

