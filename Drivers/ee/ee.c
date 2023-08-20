
#include "ee.h"
#include "eeConfig.h"
#include <string.h>

#define   PAGE                  0
#define   SECTOR                1
#define   PAGE_NUM              2


#if defined(STM32G031xx)
#define   _EE_SIZE              2048
#define   _EE_ADDR_INUSE        (((uint32_t)0x08000000) | (_EE_SIZE * _EE_USE_FLASH_PAGE_OR_SECTOR))
#define   _EE_PAGE_OR_SECTOR    PAGE_NUM
#if (_EE_USE_FLASH_PAGE_OR_SECTOR > 15)
#error  "Please Enter correct address, maximum is (31)"
#endif
#endif


//##########################################################################################################
bool ee_init(void)
{
  return true;
}
//##########################################################################################################
bool ee_format(bool keepRamData)
{
  uint32_t error;
  HAL_FLASH_Unlock();
  FLASH_EraseInitTypeDef flashErase;
#if _EE_PAGE_OR_SECTOR == PAGE_NUM
  flashErase.NbPages = 1;
  flashErase.Page = _EE_USE_FLASH_PAGE_OR_SECTOR;
  flashErase.TypeErase = FLASH_TYPEERASE_PAGES;
#endif
  if (HAL_FLASHEx_Erase(&flashErase, &error) == HAL_OK)
  {
    HAL_FLASH_Lock();
    if (error != 0xFFFFFFFF)
    {
    	return false;
    }
    else
    {
      return true;
    }
  }
  HAL_FLASH_Lock();
  return false;
}
//##########################################################################################################
bool ee_read(uint32_t startVirtualAddress, uint32_t len, uint8_t* data)
{
  if ((startVirtualAddress + len) > _EE_SIZE)
    return false;
  for (uint32_t i = startVirtualAddress; i < len + startVirtualAddress; i++)
  {
    if (data != NULL)
    {
      *data = (*(__IO uint8_t*) (i + _EE_ADDR_INUSE));
      data++;
    }
  }
  return true;
}
//##########################################################################################################
bool ee_write(uint32_t startVirtualAddress, uint32_t len, uint8_t *data)
{
  if ((startVirtualAddress + len) > _EE_SIZE)
    return false;
  if (data == NULL)
    return false;
  HAL_FLASH_Unlock();

#ifdef FLASH_TYPEPROGRAM_DOUBLEWORD
  for (uint32_t i = 0; i < len; i += 8)
  {
  	uint64_t DoubleWord = 0;
  	uint8_t* ptr = (uint8_t*)&DoubleWord;
  	ptr[0] = data[i + 0];
  	ptr[1] = data[i + 1];
  	ptr[2] = data[i + 2];
  	ptr[3] = data[i + 3];
  	ptr[4] = data[i + 4];
  	ptr[5] = data[i + 5];
  	ptr[6] = data[i + 6];
  	ptr[7] = data[i + 7];
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, ((i + startVirtualAddress)) + _EE_ADDR_INUSE, DoubleWord) != HAL_OK)
    {
      HAL_FLASH_Lock();

      return false;
    }
  }
#endif

  HAL_FLASH_Lock();

  return true;
}
//##########################################################################################################
uint32_t  ee_maxVirtualAddress(void)
{
  return (_EE_SIZE);  
}
//##########################################################################################################
