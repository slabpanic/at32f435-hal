/* AT32F435CGT7 with the default 384 KiB SRAM configuration.
 * Reserve the last 2 KiB flash sector for the flash example. */
MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 1022K
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 384K
}

_flash_scratch_start = ORIGIN(FLASH) + LENGTH(FLASH);
