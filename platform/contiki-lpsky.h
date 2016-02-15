#ifndef CONTIKI_LPSKY_H
#define CONTIKI_LPSKY_H

void lpsky_lpm3();
void lpsky_lpm4();
void lpsky_cc2420_on();
void lpsky_cc2420_off();
int lpsky_xmem_on();
void lpsky_xmem_off();
void lspky_adc_on();
void lpsky_adc_off();
void lpsky_spi_on();
void lpsky_spi_off();
void lpsky_exit(uint8_t error_code);

#define LPSKY_ERROR_NODEID 1

#endif /* CONTIKI_LPSKY_H */
