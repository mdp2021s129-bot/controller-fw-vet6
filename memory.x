/*
 * Linker script for the STM32F103VET6.
 * We don't know the exact part number, so we're assuming the lowest
 * FLASH & RAM configuration.
 */
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 256K
  RAM : ORIGIN = 0x20000000, LENGTH = 48K
}
