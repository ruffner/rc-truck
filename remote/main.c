#include <util/delay.h>
#include <avr/io.h> 
#include <math.h>
#include <inttypes.h>
#include "nRF24L01.h"
#include "../adxl362_lib/adxl362.h"
#include "../adxl362_lib/adxl362.c"

#define NODE_CONFIG ( (1<<EN_CRC) | (1<<CRCO) )

#define NRF_CE  PB0
#define NRF_CSN PB1
#define NRF_MOSI  PA5
#define NRF_MISO  PA6
#define NRF_SCK PA4

#define NRF_CE_HIGH PORTB |=  _BV(NRF_CE)
#define NRF_CE_LOW  PORTB &= ~_BV(NRF_CE)
#define NRF_CS_HIGH PORTB |= _BV(NRF_CSN)
#define NRF_CS_LOW PORTB &= ~_BV(NRF_CSN)

uint8_t my_addr[3] = {0x56, 0x34, 0x12}; // sent lsb first = 0x123456
uint8_t tx_addr[3] = {0xcc, 0xbb, 0xaa}; // sent lsb first = 0xccbbaa
int8_t ts[3];
uint8_t PTX = 0;

void report(uint8_t value);
void setup_nrf();
void setup_spi();
void send(uint8_t * value);
void get_data(uint8_t * data);
uint8_t get_status();
void flush_rx();
void power_up_rx();
void power_up_tx();
uint8_t data_ready();
uint8_t rx_fifo_empty();
void write_register(uint8_t reg, uint8_t value);
void write_register_n(uint8_t reg, uint8_t * data, uint8_t len);
uint8_t spi_transfer(uint8_t data);
void spi_write(uint8_t * data, uint8_t len);
void spi_write_read(uint8_t * data, uint8_t len);

int main() {
  uint8_t resp[4];
  int8_t last[2][2];

  setup_spi();
  setup_nrf();
  adxl362_begin();
  config_measure_mode(MODE_MEASURE);

  DDRA |= (1 << PA0);

  _delay_ms(10);

  power_up_rx();

  while(!data_ready())
    power_up_rx();
  
  get_data(resp);
  _delay_ms(3);
  if(resp[3] = 0xaa) {
    ts[0] = 0xbb;
    send(ts);
  }

  read_all_axes_short(ts);
  last[0][0] = ts[0];
  last[0][1] = ts[1];
  last[0][2] = ts[2];
  _delay_ms(100);
  read_all_axes_short(ts);
  last[1][0] = ts[0];
  last[1][1] = ts[1];
  last[1][2] = ts[2];
  _delay_ms(100);

  while(42) {
    uint8_t ctrl = 0;
    read_all_axes_short(ts);
    if( fabs((ts[0] - last[1][0]) + (last[1][0] - last[0][0])) < 4 ) {
      if(ts[0] > 30)
	ctrl |= 0x80;
      if(ts[0] < -30)
	ctrl |= 0x40;
    }
    if( fabs((ts[1] - last[1][1]) + (last[1][1] - last[0][1])) < 4 ) {
      if(ts[1] > 30)
	ctrl |= 0x01;
      if(ts[1] < -30)
	ctrl |= 0x02;
    }

    last[0][0] = last[1][0];
    last[0][1] = last[1][1];
    last[1][0] = ts[0];
    last[1][1] = ts[1];

    ts[0] = ctrl;
    send(ts);

    PORTA ^= (1 << PA0);
    _delay_ms(100);
  }
}

void setup_nrf() {
  NRF_CE_LOW;
  NRF_CS_HIGH;

  write_register(EN_AA, 0x00);           // no shockburst
  write_register(SETUP_AW, 0x01);        // 3 byte address
  write_register(RF_CH, 53);             // channel 53   
  write_register(RF_SETUP, 0x06);        // 1Mbps - max power  
  write_register(SETUP_RETR, 0x00);      // no auto-retry
  write_register_n(RX_ADDR_P0, my_addr, 3); // my_addr 
  write_register_n(TX_ADDR, tx_addr, 3);    // where were sending to
  write_register(RX_PW_P0, 0x04);        // receiving payload size of 2
  flush_rx();
  power_up_rx();
}

void setup_spi() {
  DDRB |= _BV(NRF_CE);
  DDRB |= _BV(NRF_CSN);
  DDRA |= _BV(NRF_MOSI);
  DDRA |= _BV(NRF_SCK);
  DDRA &= ~_BV(NRF_MISO);
  PORTA |= _BV(NRF_MISO);
}

void send(uint8_t * value) {
  uint8_t status;
  status = get_status();
  while (PTX) {
    status = get_status();
    if((status & ((1 << TX_DS)  | (1 << MAX_RT)))){
      PTX = 0;
      break;
    }
  }
  NRF_CE_LOW;
  power_up_tx();

  NRF_CS_LOW;
  spi_transfer( FLUSH_TX ); 
  NRF_CS_HIGH;

  uint8_t data[4];
  data[0] = W_TX_PAYLOAD;
  data[1] = value[0];
  data[2] = value[1];
  data[3] = value[2];

  NRF_CS_LOW;
  spi_write(data, 4);
  NRF_CS_HIGH;

  NRF_CE_HIGH;
}

void get_data(uint8_t * data) {
  NRF_CS_LOW;
  spi_transfer(R_RX_PAYLOAD);
  spi_write_read(data, 2);
  NRF_CS_HIGH;

  write_register(STATUS, (1<<RX_DR));
}

void power_up_rx() {
  PTX = 0;
  NRF_CE_LOW;
  write_register(CONFIG, NODE_CONFIG | ( (1<<PWR_UP) | (1<<PRIM_RX) ) );
  NRF_CE_HIGH;
  write_register(STATUS,(1 << TX_DS) | (1 << MAX_RT));
}

void power_up_tx() {
  PTX = 1;
  write_register(CONFIG, NODE_CONFIG | ( (1<<PWR_UP) | (0<<PRIM_RX) ) );
}

void flush_rx() {
  NRF_CS_LOW;
  spi_transfer(FLUSH_RX);
  NRF_CS_HIGH;
}

uint8_t data_ready() {
  uint8_t status = get_status();
  if ( status & (1 << RX_DR) ) return 1;
  return !rx_fifo_empty();
}

uint8_t rx_fifo_empty() {
  uint8_t status = 0x00;
  NRF_CS_LOW;
  spi_transfer(FIFO_STATUS);
  status = spi_transfer(0);
  NRF_CS_HIGH;
  return (status & (1 << RX_EMPTY));
}

uint8_t get_status() {
  uint8_t val;

  NRF_CS_LOW;
  val =  spi_transfer(0x00);
  NRF_CS_HIGH;

  return val;
}

void write_register(uint8_t reg, uint8_t value) {
  NRF_CS_LOW;
  uint8_t data[2] = {W_REGISTER | (REGISTER_MASK & reg), \
		  value};
  spi_write(data, 2);
  NRF_CS_HIGH;
}

void write_register_n(uint8_t reg, uint8_t * values, uint8_t len) {
  NRF_CS_LOW;
  uint8_t data[len+1];
  data[0] = W_REGISTER | (REGISTER_MASK & reg);
  uint8_t i;
  for(i = 1; i < len+1; i++)
    data[i] = values[i-1];
  spi_write(data, len+1);
  NRF_CS_HIGH;
}

void spi_write(uint8_t * data, uint8_t len) {
  uint8_t i;
  for(i = 0; i < len; i++)
    spi_transfer(data[i]);
}

void spi_write_read(uint8_t * data, uint8_t len) {
  uint8_t i;
  for(i = 0; i < len; i++)
    data[i] = spi_transfer(data[i]);
}

uint8_t spi_transfer(uint8_t data) {
  USIDR = data;
  USISR = _BV(USIOIF); // clear flag
 
  while ( (USISR & _BV(USIOIF)) == 0 ) {
    USICR = (1<<USIWM0)|(1<<USICS1)|(1<<USICLK)|(1<<USITC);
  }
  return USIDR;
}
