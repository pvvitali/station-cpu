MEMORY
{
  /* Прошивка в Секторах 5-6 (256 КБ) */
  FLASH            : ORIGIN = 0x08020000, LENGTH = 256K
  BOOTLOADER_STATE : ORIGIN = 0x08010000, LENGTH = 64K
  DFU              : ORIGIN = 0x08060000, LENGTH = 384K
  RAM              : ORIGIN = 0x20000000, LENGTH = 256K
}

/* Относительно bank1_region2 (старт 0x08010000) */
__bootloader_state_start = 0;
__bootloader_state_end = LENGTH(BOOTLOADER_STATE);

/* Относительно bank1_region3 (старт 0x08020000) */
__bootloader_dfu_start = ORIGIN(DFU) - 0x08020000;
__bootloader_dfu_end = ORIGIN(DFU) + LENGTH(DFU) - 0x08020000;