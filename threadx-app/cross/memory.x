/* memory.x - Linker script for the STM32F1XXX */
MEMORY
{
  RAM    (xrw)   : ORIGIN = 0x20000000,   LENGTH = 128K
  FLASH   (rx)   : ORIGIN = 0x8000000,    LENGTH = 1024K
  CCMRAM (rw)    : ORIGIN = 0x10000000,   LENGTH = 64K
}

