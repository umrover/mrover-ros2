### Ever wondered what that BOOT0 pin does in the STM32Cube IDE .ioc file or stm32 chip? Want to set the BOOT0 pin to an alternate function such as an I2C CLK? Here is what you need to know before doing so.

## The BOOT0 pin
The BOOT0 pin only really matters for the first 4 clock cycles of the boot sequence after reset has been released. However, it decides where the chip will boot from: Main Flash, System Memory, or Embedded SRAM. (Maybe more or less options depending on the chip). By default, holding BOOT0 LOW will make the chip boot from the main flash - this is most likely where your code is. (When using CubeIDE, this is where it is typically flashed)

If it is held high, for example let's say you are using it as an I2C CLK line with an external pull up resistor, this can cause make the chip boot from a region other than flash, which could lead to erratic behavior and floating pins.

## Alternate Boot Options
As you can see from the table below, there are actually other ways of selecting the boot region. In fact, there is one option (row 2) that completely bypasses the BOOT0 pin in order to boot from flash. This is the option we want if using the BOOT0 pin for its alternate functions.\
![image](https://github.com/umrover/mrover-ros/assets/55666129/a1fa6c0d-9d32-4765-aa1d-42b851ef1ac6)\
_[STM32 G4 Series Manual](https://www.st.com/resource/en/reference_manual/dm00355726-stm32g4-series-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)_

## Configuring Boot Options
The following code was taken from a great [stack overflow post](https://stackoverflow.com/questions/30449912/how-to-boot-stm32-from-user-flash). It configures the boot option register and sets nBOOT0 to 1 and nSWBOOT0 to 0.

```   
if ((FLASH->OPTR & FLASH_OPTR_nSWBOOT0_Msk) != 0x0 || ((FLASH->OPTR & FLASH_OPTR_nBOOT0_Msk) == 0x0))
{
	while ((FLASH->SR & FLASH_SR_BSY_Msk) != 0x0) { ; }
	FLASH->KEYR = 0x45670123;
	while ((FLASH->SR & FLASH_SR_BSY_Msk) != 0x0) { ; }
	FLASH->KEYR = 0xCDEF89AB;

	while ((FLASH->SR & FLASH_SR_BSY_Msk) != 0x0) { ; }
	FLASH->OPTKEYR = 0x08192A3B;
	while ((FLASH->SR & FLASH_SR_BSY_Msk) != 0x0) { ; }
	FLASH->OPTKEYR = 0x4C5D6E7F;

	while ((FLASH->SR & FLASH_SR_BSY_Msk) != 0x0) { ; }
	FLASH->OPTR = (FLASH->OPTR & ~(FLASH_OPTR_nSWBOOT0_Msk)) | FLASH_OPTR_nBOOT0_Msk;

	while ((FLASH->SR & FLASH_SR_BSY_Msk) != 0x0) { ; }
	FLASH->CR = FLASH->CR | FLASH_CR_OPTSTRT;

	while ((FLASH->SR & FLASH_SR_BSY_Msk) != 0x0) { ; }
}
```

Here I explain it with some more detail.\
`while ((FLASH->SR & FLASH_SR_BSY_Msk) != 0x0) { ; }` Polls the flash status register and waits until it is no longer busy.

`FLASH->KEYR = 0x45670123;`\
`FLASH->KEYR = 0xCDEF89AB;` Is the sequence of register writes needed to unlock flash for writing/reading.

`FLASH->OPTKEYR = 0x08192A3B;`\
`FLASH->OPTKEYR = 0x4C5D6E7F;` Is the sequence of register writes needed to unlock the Options register.

`FLASH->OPTR = (FLASH->OPTR & ~(FLASH_OPTR_nSWBOOT0_Msk)) | FLASH_OPTR_nBOOT0_Msk;` Sets nBOOT0 to 1 and nSWBOOT0 to 0.

## Other Resources
STM has other methods of programming these option bits, including using HAL functions. See the following resources for more info. Also make sure to look at the STM reference manual for your particular chip regarding boot options.\
[How to program STM32 Option Bytes with the HAL API](https://community.st.com/t5/stm32-mcus/how-to-program-stm32-option-bytes-with-the-hal-api/ta-p/49660)\
[What are option bytes in STM32 and how do I use them?](https://community.st.com/t5/stm32-mcus/what-are-option-bytes-in-stm32-and-how-do-i-use-them/ta-p/49451)