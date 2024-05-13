#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"

#include "hardware/clocks.h"
#include "hardware/pwm.h"

#define PROG_FLICKER true

// Configure SPI clock (in Hz).
// E.g. for an ATtiny @ 128 kHz: the datasheet states that both the high and low
// SPI clock pulse must be > 2 CPU cycles, so take 3 cycles i.e. divide target
// f_cpu by 6:
//     #define SPI_CLOCK            (128000/6)
//
// A clock slow enough for an ATtiny85 @ 1 MHz, is a reasonable default:

#define SPI_CLOCK (1000000 / 6)

// Select hardware or software SPI, depending on SPI clock.
// Currently only for AVR, for other architectures (Due, Zero,...), hardware SPI
// is probably too fast anyway.

// Configure which pins to use:

// The standard pin configuration.

#define LED_HB 25
#define LED_ERR 20
#define LED_PMODE 22
#define PIN_RESET 3 // Use pin 10 to reset the target rather than SS
#define PIN_SCK 2
#define PIN_MISO 1
#define PIN_MOSI 0

#define HWVER 2
#define SWMAJ 1
#define SWMIN 18

// STK Definitions
#define STK_OK 0x10
#define STK_FAILED 0x11
#define STK_UNKNOWN 0x12
#define STK_INSYNC 0x14
#define STK_NOSYNC 0x15
#define CRC_EOP 0x20 // ok it is a space...

#define SPI_MODE0 0x00

typedef struct {
  uint32_t clock_freq;
} spi_settings;

spi_settings current_spi_settings;

uint32_t spi_pulse_width;

void pulse(int pin, int times);

void init_bitbanged_spi() {
  gpio_init(PIN_MISO);
  gpio_init(PIN_MOSI);
  gpio_init(PIN_SCK);
  gpio_init(PIN_RESET);
  gpio_init(LED_ERR);
  gpio_init(LED_PMODE);

  gpio_set_dir(PIN_MISO, false);
  gpio_set_dir(PIN_MOSI, true);
  gpio_set_dir(PIN_SCK, true);
  gpio_set_dir(PIN_RESET, true);
  gpio_set_dir(LED_ERR, true);
  gpio_set_dir(LED_PMODE, true);

  gpio_put(PIN_SCK, false);
  gpio_put(PIN_MOSI, false);

  gpio_set_function(LED_HB, GPIO_FUNC_PWM);
  uint slice_num = pwm_gpio_to_slice_num(LED_HB);

  pwm_clear_irq(slice_num);
  pwm_set_irq_enabled(slice_num, false);

  pwm_config config = pwm_get_default_config();
  pwm_config_set_clkdiv(&config, 8.f);
  pwm_config_set_wrap(&config, 256);
  pwm_init(slice_num, &config, true);
  pwm_set_gpio_level(LED_HB, 128);
}

void begin_spi_transaction(spi_settings sett) {
  spi_pulse_width = (500000 + sett.clock_freq - 1) / sett.clock_freq;
  if (spi_pulse_width == 0) {
    spi_pulse_width = 1;
  }
}

void end_spi_transaction() { return; }

uint8_t spi_transfer(uint8_t b) {
  for (int i = 0; i < 8; i++) {
    gpio_put(PIN_MOSI, (b & 0x80) ? true : false);
    gpio_put(PIN_SCK, true);
    busy_wait_us_32(spi_pulse_width);
    b = (b << 1) | gpio_get(PIN_MISO);
    gpio_put(PIN_SCK, false);
    busy_wait_us_32(spi_pulse_width);
  }
  return b;
}

void setup() {
  init_bitbanged_spi();
  pulse(LED_PMODE, 2);
  pulse(LED_ERR, 2);
  pulse(LED_HB, 2);
}

int ISPError = 0;
int pmode = 0;
// address for reading and writing, set by 'U' command
unsigned int here;
uint8_t buff[256]; // global block storage

#define beget16(addr) (*addr * 256 + *(addr + 1))
typedef struct param {
  uint8_t devicecode;
  uint8_t revision;
  uint8_t progtype;
  uint8_t parmode;
  uint8_t polling;
  uint8_t selftimed;
  uint8_t lockbytes;
  uint8_t fusebytes;
  uint8_t flashpoll;
  uint16_t eeprompoll;
  uint16_t pagesize;
  uint16_t eepromsize;
  uint32_t flashsize;
} parameter;

parameter param;

// this provides a heartbeat on pin 9, so you can tell the software is running.
uint8_t hbval = 128;
int8_t hbdelta = 8;
void heartbeat() {
  static uint64_t last_time = 0;
  uint64_t now = ((uint64_t)get_absolute_time()) / 1000;
  if ((now - last_time) < 40) {
    return;
  }
  last_time = now;
  if (hbval > 192) {
    hbdelta = -hbdelta;
  }
  if (hbval < 32) {
    hbdelta = -hbdelta;
  }
  hbval += hbdelta;
  pwm_set_gpio_level(LED_HB, hbval);
}

static bool rst_active_high;

void reset_target(bool reset) {
  gpio_put(PIN_RESET,
               ((reset && rst_active_high) || (!reset && !rst_active_high))
                   ? true
                   : false);
}

uint8_t getch() {
  return getchar();
}

void fill(int n) {
  for (int x = 0; x < n; x++) {
    buff[x] = getch();
  }
}

#define PTIME 30
void pulse(int pin, int times) {
  do {
    gpio_put(pin, true);
    busy_wait_ms(PTIME);
    gpio_put(pin, false);
    busy_wait_ms(PTIME);
  } while (times--);
}

void prog_lamp(int state) {
  if (PROG_FLICKER) {
    gpio_put(LED_PMODE, state);
  }
}

uint8_t spi_transaction(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
  spi_transfer(a);
  spi_transfer(b);
  spi_transfer(c);
  return spi_transfer(d);
}

void empty_reply() {
  if (CRC_EOP == getch()) {
    putchar((char)STK_INSYNC);
    putchar((char)STK_OK);
  } else {
    ISPError++;
    putchar((char)STK_NOSYNC);
  }
}

void breply(uint8_t b) {
  if (CRC_EOP == getch()) {
    putchar((char)STK_INSYNC);
    putchar((char)b);
    putchar((char)STK_OK);
  } else {
    ISPError++;
    putchar((char)STK_NOSYNC);
  }
}

void get_version(uint8_t c) {
  switch (c) {
  case 0x80:
    breply(HWVER);
    break;
  case 0x81:
    breply(SWMAJ);
    break;
  case 0x82:
    breply(SWMIN);
    break;
  case 0x93:
    breply('S'); // serial programmer
    break;
  default:
    breply(0);
  }
}

void set_parameters() {
  // call this after reading parameter packet into buff[]
  param.devicecode = buff[0];
  param.revision = buff[1];
  param.progtype = buff[2];
  param.parmode = buff[3];
  param.polling = buff[4];
  param.selftimed = buff[5];
  param.lockbytes = buff[6];
  param.fusebytes = buff[7];
  param.flashpoll = buff[8];
  // ignore buff[9] (= buff[8])
  // following are 16 bits (big endian)
  param.eeprompoll = beget16(&buff[10]);
  param.pagesize = beget16(&buff[12]);
  param.eepromsize = beget16(&buff[14]);

  // 32 bits flashsize (big endian)
  param.flashsize = buff[16] * 0x01000000 + buff[17] * 0x00010000 +
                    buff[18] * 0x00000100 + buff[19];

  // AVR devices have active low reset, AT89Sx are active high
  rst_active_high = (param.devicecode >= 0xe0);
}

void start_pmode() {

  // Reset target before driving PIN_SCK or PIN_MOSI

  spi_settings sett;
  sett.clock_freq = SPI_CLOCK;
  gpio_set_dir(PIN_RESET, true);
  reset_target(true);
  init_bitbanged_spi();
  begin_spi_transaction(sett);

  // See AVR datasheets, chapter "SERIAL_PRG Programming Algorithm":

  // Pulse PIN_RESET after PIN_SCK is low:
  gpio_put(PIN_SCK, false);
  busy_wait_ms(20); // discharge PIN_SCK, value arbitrarily chosen
  reset_target(false);
  // Pulse must be minimum 2 target CPU clock cycles so 100 usec is ok for CPU
  // speeds above 20 KHz
  busy_wait_us_32(100);
  reset_target(true);

  // Send the enable programming command:
  busy_wait_ms(50); // datasheet: must be > 20 msec
  spi_transaction(0xAC, 0x53, 0x00, 0x00);
  pmode = 1;
}

void end_pmode() {
  end_spi_transaction();
  // We're about to take the target out of reset so configure SPI pins as input
  //gpio_set_dir(PIN_MOSI, false);
  //gpio_set_dir(PIN_SCK, false);
  reset_target(false);
  //gpio_set_dir(PIN_RESET, false);
  pmode = 0;
}

void universal() {
  uint8_t ch;

  fill(4);
  ch = spi_transaction(buff[0], buff[1], buff[2], buff[3]);
  breply(ch);
}

void flash(uint8_t hilo, unsigned int addr, uint8_t data) {
  spi_transaction(0x40 + 8 * hilo, addr >> 8 & 0xFF, addr & 0xFF, data);
}
void commit(unsigned int addr) {
  if (PROG_FLICKER) {
    prog_lamp(false);
  }
  spi_transaction(0x4C, (addr >> 8) & 0xFF, addr & 0xFF, 0);
  if (PROG_FLICKER) {
    busy_wait_ms(PTIME);
    prog_lamp(true);
  }
}

unsigned int current_page() {
  if (param.pagesize == 32) {
    return here & 0xFFFFFFF0;
  }
  if (param.pagesize == 64) {
    return here & 0xFFFFFFE0;
  }
  if (param.pagesize == 128) {
    return here & 0xFFFFFFC0;
  }
  if (param.pagesize == 256) {
    return here & 0xFFFFFF80;
  }
  return here;
}

uint8_t write_flash_pages(int length);

void write_flash(int length) {
  fill(length);
  if (CRC_EOP == getch()) {
    putchar((char)STK_INSYNC);
    putchar((char)write_flash_pages(length));
  } else {
    ISPError++;
    putchar((char)STK_NOSYNC);
  }
}

uint8_t write_flash_pages(int length) {
  int x = 0;
  unsigned int page = current_page();
  while (x < length) {
    if (page != current_page()) {
      commit(page);
      page = current_page();
    }
    flash(false, here, buff[x++]);
    flash(true, here, buff[x++]);
    here++;
  }

  commit(page);

  return STK_OK;
}

uint8_t write_eeprom_chunk(unsigned int start, unsigned int length);

#define EECHUNK (32)
uint8_t write_eeprom(unsigned int length) {
  // here is a word address, get the byte address
  unsigned int start = here * 2;
  unsigned int remaining = length;
  if (length > param.eepromsize) {
    ISPError++;
    return STK_FAILED;
  }
  while (remaining > EECHUNK) {
    write_eeprom_chunk(start, EECHUNK);
    start += EECHUNK;
    remaining -= EECHUNK;
  }
  write_eeprom_chunk(start, remaining);
  return STK_OK;
}
// write (length) bytes, (start) is a byte address
uint8_t write_eeprom_chunk(unsigned int start, unsigned int length) {
  // this writes byte-by-byte, page writing may be faster (4 bytes at a time)
  fill(length);
  prog_lamp(false);
  for (unsigned int x = 0; x < length; x++) {
    unsigned int addr = start + x;
    spi_transaction(0xC0, (addr >> 8) & 0xFF, addr & 0xFF, buff[x]);
    busy_wait_ms(45);
  }
  prog_lamp(true);
  return STK_OK;
}

void program_page() {
  char result = (char)STK_FAILED;
  unsigned int length = 256 * getch();
  length += getch();
  char memtype = getch();
  // flash memory @here, (length) bytes
  if (memtype == 'F') {
    write_flash(length);
    return;
  }
  if (memtype == 'E') {
    result = (char)write_eeprom(length);
    if (CRC_EOP == getch()) {
      putchar((char)STK_INSYNC);
      putchar(result);
    } else {
      ISPError++;
      putchar((char)STK_NOSYNC);
    }
    return;
  }
  putchar((char)STK_FAILED);
  return;
}

uint8_t flash_read(uint8_t hilo, unsigned int addr) {
  return spi_transaction(0x20 + hilo * 8, (addr >> 8) & 0xFF, addr & 0xFF, 0);
}

char flash_read_page(int length) {
  for (int x = 0; x < length; x += 2) {
    uint8_t low = flash_read(false, here);
    putchar((char)low);
    uint8_t high = flash_read(true, here);
    putchar((char)high);
    here++;
  }
  return STK_OK;
}

char eeprom_read_page(int length) {
  // here again we have a word address
  int start = here * 2;
  for (int x = 0; x < length; x++) {
    int addr = start + x;
    uint8_t ee = spi_transaction(0xA0, (addr >> 8) & 0xFF, addr & 0xFF, 0xFF);
    putchar((char)ee);
  }
  return STK_OK;
}

void read_page() {
  char result = (char)STK_FAILED;
  int length = 256 * getch();
  length += getch();
  char memtype = getch();
  if (CRC_EOP != getch()) {
    ISPError++;
    putchar((char)STK_NOSYNC);
    return;
  }
  putchar((char)STK_INSYNC);
  if (memtype == 'F') {
    result = flash_read_page(length);
  }
  if (memtype == 'E') {
    result = eeprom_read_page(length);
  }
  putchar(result);
}

void read_signature() {
  if (CRC_EOP != getch()) {
    ISPError++;
    putchar((char)STK_NOSYNC);
    return;
  }
  putchar((char)STK_INSYNC);
  uint8_t high = spi_transaction(0x30, 0x00, 0x00, 0x00);
  putchar((char)high);
  uint8_t middle = spi_transaction(0x30, 0x00, 0x01, 0x00);
  putchar((char)middle);
  uint8_t low = spi_transaction(0x30, 0x00, 0x02, 0x00);
  putchar((char)low);
  putchar((char)STK_OK);
}
//////////////////////////////////////////
//////////////////////////////////////////

////////////////////////////////////
////////////////////////////////////
void avrisp(uint8_t ch, bool has_ch) {
  if(!has_ch){
    ch = getch();
  }
  switch (ch) {
  case '0': // signon
    ISPError = 0;
    empty_reply();
    break;
  case '1':
    if (getch() == CRC_EOP) {
      putchar((char)STK_INSYNC);
      printf("AVR ISP");
      putchar((char)STK_OK);
    } else {
      ISPError++;
      putchar((char)STK_NOSYNC);
    }
    break;
  case 'A':
    get_version(getch());
    break;
  case 'B':
    fill(20);
    set_parameters();
    empty_reply();
    break;
  case 'E': // extended parameters - ignore for now
    fill(5);
    empty_reply();
    break;
  case 'P':
    if (!pmode) {
      start_pmode();
    }
    empty_reply();
    break;
  case 'U': // set address (word)
    here = getch();
    here += 256 * getch();
    empty_reply();
    break;

  case 0x60: // STK_PROG_FLASH
    getch(); // low addr
    getch(); // high addr
    empty_reply();
    break;
  case 0x61: // STK_PROG_DATA
    getch(); // data
    empty_reply();
    break;

  case 0x64: // STK_PROG_PAGE
    program_page();
    break;

  case 0x74: // STK_READ_PAGE 't'
    read_page();
    break;

  case 'V': // 0x56
    universal();
    break;
  case 'Q': // 0x51
    ISPError = 0;
    end_pmode();
    empty_reply();
    break;

  case 0x75: // STK_READ_SIGN 'u'
    read_signature();
    break;

  // expecting a command, not CRC_EOP
  // this is how we can get back in sync
  case CRC_EOP:
    ISPError++;
    putchar((char)STK_NOSYNC);
    break;

  // anything else we will return STK_UNKNOWN
  default:
    ISPError++;
    if (CRC_EOP == getch()) {
      putchar((char)STK_UNKNOWN);
    } else {
      putchar((char)STK_NOSYNC);
    }
  }
}

int main() {
  stdio_usb_init();
  stdio_set_translate_crlf(&stdio_usb, false); // By default this is true

  setup();

  // Should enable 12MHz clock on pin 21
  clock_gpio_init(21, CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_VALUE_CLK_USB, 4);

  while(true) {
    if (pmode) {
      gpio_put(LED_PMODE, true);
    } else {
      gpio_put(LED_PMODE, false);
    }
    // is there an error?
    if (ISPError) {
      gpio_put(LED_ERR, true);
    } else {
      gpio_put(LED_ERR, false);
    }

    // light the heartbeat LED
    heartbeat();
    int tmp_ch = getchar_timeout_us(10);
    if (tmp_ch != PICO_ERROR_TIMEOUT) {
      avrisp((char)(tmp_ch & 0xff), true);
    }
  }

  return 0;
}
