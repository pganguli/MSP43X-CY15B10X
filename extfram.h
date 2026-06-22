// SPI FRAM driver API for MSP430FR5994 and MSP432P401R.
//
// The external FRAM chip (8 Mbit) is accessed over SPI using DMA to avoid
// tying up the CPU during transfers.  All NVM reads/writes in the platform
// layer call into this API.
//
// FRAM_FREQ_DIVIDER: SPI clock = SMCLK / DIVIDER.  MSP430 uses /4 (~4 MHz
//   at 16 MHz SMCLK); MSP432 uses /6 to work around DMA read errors that
//   occurred at higher SPI frequencies — likely a setup/hold time issue on
//   the specific board layout.
//
// SPI_ADDR: 24-bit FRAM address packed into a union so individual bytes can
//   be sent as the 3-byte opcode+address SPI preamble without shifting.
//
// SPI_READ(A, dst, len): initiates a DMA-backed read; call SPI_WAIT_DMA()
//   afterwards to block until the transfer completes.
//
// SPI_WRITE(A, src, len): synchronous write (busy-waits internally).
//
// SPI_WRITE2(A, src, len, timer_delay): same as SPI_WRITE but inserts a
//   timer_delay between the chip-select assert and the first data byte.
//   Used when writing to HAWAII footprints to allow the energy-harvesting
//   capacitor to partially recharge between writes.
//
// controlTable: μDMA channel control table; must be 1024-byte aligned as
//   required by the MSP432 μDMA controller.

#ifndef JAPARILIB_EXTFRAM_H_
#define JAPARILIB_EXTFRAM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define EXTFRAM_USE_DMA

#define FRAM_8Mb

// SPI clock divider — see file header for rationale
#ifdef __MSP430__
#define FRAM_FREQ_DIVIDER 4
#elif defined(__MSP432__)
// /6 avoids DMA read corruption at higher SPI frequencies on this board
#define FRAM_FREQ_DIVIDER 6
#endif

extern uint8_t controlTable[1024] __attribute__((aligned(1024)));
extern uint32_t curDMATransmitChannelNum, curDMAReceiveChannelNum;

// 24-bit FRAM address; byte[3]=MSB, byte[0]=LSB (sent byte-by-byte over SPI)
typedef union Data {
  unsigned char byte[4];
  unsigned long L;
} SPI_ADDR;

void eraseFRAM(void);
void eraseFRAM2(uint8_t init_val);  // Erase to init_val instead of 0xFF
void initSPI(void);
void SPI_READ(SPI_ADDR* A, uint8_t* dst, unsigned long len);
void SPI_WAIT_DMA();  // Block until in-flight DMA read completes
void SPI_WRITE(SPI_ADDR* A, const uint8_t* src, unsigned long len);
void SPI_WRITE2(SPI_ADDR* A, const uint8_t* src, unsigned long len,
                uint16_t timer_delay);
uint8_t testSPI(void);  // Basic FRAM self-test; returns non-zero on failure

#ifdef __cplusplus
}
#endif

#endif /* JAPARILIB_EXTFRAM_H_ */
