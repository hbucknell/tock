/* Memory layout for the STM32L476RG
 * rom = 1024KB (LENGTH = 0x00100000)
 * kernel = 256KB
 * user = 768KB
 * ram = 128KB, but split into SRAM1 = 96KB and SRAM2 = 32KB */

MEMORY
{
  rom (rx)  : ORIGIN = 0x08000000, LENGTH = 0x00040000
  prog (rx) : ORIGIN = 0x08040000, LENGTH = 0x000C0000
  ram (rwx) : ORIGIN = 0x20000000, LENGTH = 0x00018000
}

MPU_MIN_ALIGN = 8K;
