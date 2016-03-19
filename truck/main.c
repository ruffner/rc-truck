#include <util/delay.h>
#include <avr/io.h> 
#include <inttypes.h>
#include <avr/interrupt.h>
#include "nRF24L01.h"

#define STD_CONFIG ( (1<<EN_CRC) | (1<<CRCO) )

#define CHANNEL 53

#define PAYLOAD_SIZE 3

#define DIR_LEFT   0
#define DIR_CENTER 1
#define DIR_RIGHT  2
#define LIM_LEFT   PINA & (1 << PA0)
#define LIM_RIGHT  PINA & (1 << PA1)

#define SETUP_LEFT    PORTB |= (1 << PB0)
#define SETUP_RIGHT   PORTB &= ~(1 << PB0)
#define SETUP_FORWARD PORTB |= (1 << PB1)
#define SETUP_REVERSE PORTB &= ~(1 << PB1)

#define CTRL_DRIVE PB2
#define CTRL_TURN  PA7

#define NRF_CE  PA2
#define NRF_CSN PA3
#define NRF_DO  PA5
#define NRF_DI  PA6
#define NRF_SCK PA4

#define CE_HIGH PORTA |=  _BV(NRF_CE)
#define CE_LOW  PORTA &= ~_BV(NRF_CE)
#define CSN_HIGH PORTA |= _BV(NRF_CSN)
#define CSN_LOW PORTA &= ~_BV(NRF_CSN)

uint8_t tx_addr[3] = {0x56, 0x34, 0x12}; // sent lsb first = 0x123456
uint8_t my_addr[3] = {0xcc, 0xbb, 0xaa}; // sent lsb first = 0xccbbaa
uint8_t PTX = 0;
uint8_t cur_pos = DIR_CENTER;

// ----------------- high level
void turn(uint8_t dir);
void send(uint8_t data);
void get_data(uint8_t * data);
// ----------------- spi and barebones
void setup();
void setup_nrf();
void setup_spi();
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
  uint8_t com[PAYLOAD_SIZE];
  uint8_t ctrl;

  setup();
  setup_spi();
  setup_nrf();

  _delay_ms(100);

  while(com[0] != 0xbb) {
    while(!data_ready()) {
      send(0xaa);
      power_up_rx();
      _delay_ms(500);
    }
    get_data(com);
  }

  while(42) {
    power_up_rx();
    while(!data_ready())
      power_up_rx();
    
    get_data(com);
    ctrl = com[0];
    _delay_ms(50);

    // foreard/reverse
    if(ctrl & 0x40) {
      SETUP_FORWARD;
      PORTB |= (1 << CTRL_DRIVE);
    }
    else if(ctrl & 0x80) {
      SETUP_REVERSE;
      PORTB |= (1 << CTRL_DRIVE);
    } else {
      PORTB &= ~(1 << CTRL_DRIVE);
    }
    
    // turning

    if(ctrl & 0x02)
      turn(DIR_LEFT);
    else if(ctrl & 0x01)
      turn(DIR_RIGHT);
    else 
      turn(DIR_CENTER);
    
    _delay_ms(20);
  }
}

void drive(uint8_t dir) {

}

void turn(uint8_t dir) {
  if(cur_pos == dir)
    return;

  switch(dir) {
  case DIR_LEFT:
    SETUP_LEFT;
    PORTB |= (1 << CTRL_TURN);
    //while(LIMIT_LEFT);
    if(cur_pos == DIR_RIGHT)
      _delay_ms(1000);
    if(cur_pos == DIR_CENTER)
      _delay_ms(500);
    PORTB &= ~(1 << CTRL_TURN);
    cur_pos = DIR_LEFT;
    break;
  case DIR_CENTER:
    switch(cur_pos) {
    case DIR_LEFT:
      SETUP_RIGHT;
      PORTB |= (1 << CTRL_TURN);
      //while(LIMIT_LEFT);
      _delay_ms(500);
      PORTB &= ~(1 << CTRL_TURN);
      break;
    case DIR_RIGHT:
      SETUP_LEFT;
      PORTB |= (1 << CTRL_TURN);
      //while(LIMIT_LEFT);
      _delay_ms(500);
      PORTB &= ~(1 << CTRL_TURN);
      break;
    }
    cur_pos = DIR_CENTER;
    break;
  case DIR_RIGHT:
    SETUP_RIGHT;
    PORTB |= (1 << CTRL_TURN);
    //while(LIMIT_LEFT);
    if(cur_pos == DIR_LEFT)
      _delay_ms(1000);
    if(cur_pos == DIR_CENTER)
      _delay_ms(500);
    PORTB &= ~(1 << CTRL_TURN);
    cur_pos = DIR_RIGHT;
    break;
  }
}

void setup() {
  DDRA &= ~_BV(PA0); // limit left
  DDRA &= ~_BV(PA1); // limit right
  PORTA |= (1 << PA0); // pullup
  PORTA |= (1 << PA1); //pullup
  DDRB |= (1 << CTRL_DRIVE);
  DDRA |= (1 << CTRL_TURN);
  DDRB |= (1 << PB0); // forward/rev control
  DDRB |= (1 << PB1); // left/right control
}

void setup_nrf() {
  CE_LOW;
  CSN_HIGH;

  write_register(EN_AA, 0x00);           // no shockburst
  write_register(SETUP_AW, 0x01);        // 3 byte address
  write_register(RF_CH, CHANNEL);        // channel    
  write_register(RF_SETUP, 0x06);        // 1Mbps - max power  
  write_register(SETUP_RETR, 0x00);      // no auto-retry
  write_register_n(RX_ADDR_P0, my_addr, 3); // my_addr (made-up)
  write_register_n(TX_ADDR, tx_addr, 3);    // where were sending to
  write_register(RX_PW_P0, 0x03);        // receiving payload size of 3
  flush_rx();
  power_up_rx();
}

void setup_spi() {
  DDRA |= _BV(NRF_CE);
  DDRA |= _BV(NRF_CSN);
  DDRA |= _BV(NRF_DO);
  DDRA |= _BV(NRF_SCK);
  DDRA &= ~_BV(NRF_DI);
  PORTA |= _BV(NRF_DI);
}

void send(uint8_t val) {
  uint8_t status;
  status = get_status();
  while (PTX) {
    status = get_status();
    if((status & ((1 << TX_DS)  | (1 << MAX_RT)))){
      PTX = 0;
      break;
    }
  }
  CE_LOW;
  power_up_tx();

  CSN_LOW;
  spi_transfer( FLUSH_TX ); 
  CSN_HIGH;

  uint8_t data[5];
  data[0] = W_TX_PAYLOAD;
  data[1] = 'k';
  data[2] = 'k';
  data[3] = 'k';
  data[4] = val;

  CSN_LOW;
  spi_write(data, 5);
  CSN_HIGH;

  CE_HIGH;
}

void get_data(uint8_t * data) {
  CSN_LOW;
  spi_transfer(R_RX_PAYLOAD);
  spi_write_read(data, PAYLOAD_SIZE);
  CSN_HIGH;

  write_register(STATUS, (1<<RX_DR));
}

void power_up_rx() {
  PTX = 0;
  CE_LOW;
  write_register(CONFIG, STD_CONFIG | ( (1<<PWR_UP) | (1<<PRIM_RX) ) );
  CE_HIGH;
  write_register(STATUS,(1 << TX_DS) | (1 << MAX_RT));
}

void power_up_tx() {
  PTX = 1;
  write_register(CONFIG, STD_CONFIG | ( (1<<PWR_UP) | (0<<PRIM_RX) ) );
}

void flush_rx() {
  CSN_LOW;
  spi_transfer(FLUSH_RX);
  CSN_HIGH;
}

uint8_t data_ready() {
  uint8_t status = get_status();
  if ( status & (1 << RX_DR) ) return 1;
  return !rx_fifo_empty();
}

uint8_t rx_fifo_empty() {
  uint8_t status = 0x00;
  CSN_LOW;
  spi_transfer(FIFO_STATUS);
  status = spi_transfer(0);
  CSN_HIGH;
  return (status & (1 << RX_EMPTY));
}

uint8_t get_status() {
  uint8_t val;

  CSN_LOW;
  val =  spi_transfer(NOP);
  CSN_HIGH;

  return val;
}

void write_register(uint8_t reg, uint8_t value) {
  CSN_LOW;
  uint8_t data[2] = {W_REGISTER | (REGISTER_MASK & reg), \
		  value};
  spi_write(data, 2);
  CSN_HIGH;
}

void write_register_n(uint8_t reg, uint8_t * values, uint8_t len) {
  CSN_LOW;
  uint8_t data[len+1];
  data[0] = W_REGISTER | (REGISTER_MASK & reg);
  uint8_t i;
  for(i = 1; i < len+1; i++)
    data[i] = values[i-1];
  spi_write(data, len+1);
  CSN_HIGH;
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
