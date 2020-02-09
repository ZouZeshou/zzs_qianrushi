/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include "string.h"
#include "drv_flash.h"
/********** from calibrate.c start***********************/
cali_sys_t cali_param;

cali_sys_t *get_cali_param(void)
{
  return &cali_param;
}

/**
  * @brief save calibrate data cali_param, write this structure in chip flash
  * @usage called when calibrate data be changed in calibrate loop
  */
void save_cali_data(void)
{
  BSP_FLASH_Write((uint8_t*)&cali_param, sizeof(cali_sys_t));
}
/**
  * @brief read calibrate data cali_param from chip flash
  * @usage called after cali_param_init() in main() initialize part.
  */
void cali_data_read(void)
{
  memcpy((void*)&cali_param, (void*)PARAM_SAVED_START_ADDRESS, sizeof(cali_sys_t));
} 

/**
  * @brief read gimbal pitch and yaw center point offset
  *        the offset is absolute encoder value
  * @usage called in init task
  */
void cali_param_init(void)
{
  cali_data_read();
}

/**
  * @brief save gimbal pitch and yaw center point offset
  *        the offset is absolute encoder value
  * @usage called in gimbal task loop 
  */
void gimbal_save_data(uint16_t yaw_ecd, uint16_t pitch_ecd)
{
  cali_param.gim_cali_data.yaw_offset   = yaw_ecd;
  cali_param.gim_cali_data.pitch_offset = pitch_ecd;
  //cali_param.gim_cali_data.calied_done  = CALIED_FLAG;
  save_cali_data();
}
/**
  * @brief save shoot data
  * @usage called in shoot task 
  */
void save_shoot_data(uint16_t err)
{
	cali_param.shoot_cali_data.err = err;
  save_cali_data();
}
/********** from calibrate.c end***********************/

static uint32_t GetSector(uint32_t Address);

/**
  * @brief  Gets the sector of a given address
  * @param  None
  * @retval The sector of a given address
  */
static uint32_t GetSector(uint32_t Address)
{
  uint32_t sector = 0;

  if ((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
  {
    sector = FLASH_SECTOR_0;
  }
  else if ((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
  {
    sector = FLASH_SECTOR_1;
  }
  else if ((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
  {
    sector = FLASH_SECTOR_2;
  }
  else if ((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
  {
    sector = FLASH_SECTOR_3;
  }
  else if ((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
  {
    sector = FLASH_SECTOR_4;
  }
  else if ((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
  {
    sector = FLASH_SECTOR_5;
  }
  else if ((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
  {
    sector = FLASH_SECTOR_6;
  }
  else if ((Address < ADDR_FLASH_SECTOR_8) && (Address >= ADDR_FLASH_SECTOR_7))
  {
    sector = FLASH_SECTOR_7;
  }
  else if ((Address < ADDR_FLASH_SECTOR_9) && (Address >= ADDR_FLASH_SECTOR_8))
  {
    sector = FLASH_SECTOR_8;
  }
  else if ((Address < ADDR_FLASH_SECTOR_10) && (Address >= ADDR_FLASH_SECTOR_9))
  {
    sector = FLASH_SECTOR_9;
  }
  else if ((Address < ADDR_FLASH_SECTOR_11) && (Address >= ADDR_FLASH_SECTOR_10))
  {
    sector = FLASH_SECTOR_10;
  }
  else if ((Address < ADDR_FLASH_SECTOR_12) && (Address >= ADDR_FLASH_SECTOR_11))
  {
    sector = FLASH_SECTOR_11;
  }
  else if ((Address < ADDR_FLASH_SECTOR_13) && (Address >= ADDR_FLASH_SECTOR_12))
  {
    sector = FLASH_SECTOR_12;
  }
  else if ((Address < ADDR_FLASH_SECTOR_14) && (Address >= ADDR_FLASH_SECTOR_13))
  {
    sector = FLASH_SECTOR_13;
  }
  else if ((Address < ADDR_FLASH_SECTOR_15) && (Address >= ADDR_FLASH_SECTOR_14))
  {
    sector = FLASH_SECTOR_14;
  }
  else if ((Address < ADDR_FLASH_SECTOR_16) && (Address >= ADDR_FLASH_SECTOR_15))
  {
    sector = FLASH_SECTOR_15;
  }
  else if ((Address < ADDR_FLASH_SECTOR_17) && (Address >= ADDR_FLASH_SECTOR_16))
  {
    sector = FLASH_SECTOR_16;
  }
  else if ((Address < ADDR_FLASH_SECTOR_18) && (Address >= ADDR_FLASH_SECTOR_17))
  {
    sector = FLASH_SECTOR_17;
  }
  else if ((Address < ADDR_FLASH_SECTOR_19) && (Address >= ADDR_FLASH_SECTOR_18))
  {
    sector = FLASH_SECTOR_18;
  }
  else if ((Address < ADDR_FLASH_SECTOR_20) && (Address >= ADDR_FLASH_SECTOR_19))
  {
    sector = FLASH_SECTOR_19;
  }
  else if ((Address < ADDR_FLASH_SECTOR_21) && (Address >= ADDR_FLASH_SECTOR_20))
  {
    sector = FLASH_SECTOR_20;
  }
  else if ((Address < ADDR_FLASH_SECTOR_22) && (Address >= ADDR_FLASH_SECTOR_21))
  {
    sector = FLASH_SECTOR_21;
  }
  else if ((Address < ADDR_FLASH_SECTOR_23) && (Address >= ADDR_FLASH_SECTOR_22))
  {
    sector = FLASH_SECTOR_22;
  }
  else /* (Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_23) */
  {
    sector = FLASH_SECTOR_23;
  }
  return sector;
}

/**
  * @brief  write bytes to flash  
  *         start addr  = PARAM_SAVED_START_ADDRESS
  * @retval flash write status
  */
uint32_t SectorError;
uint8_t BSP_FLASH_Write(uint8_t *pbuff, uint32_t len)
{
  FLASH_EraseInitTypeDef EraseInitStruct;
  /*erase flash before program */
  uint32_t WriteSector = FLASH_SECTOR_23;
  uint32_t start_addr = PARAM_SAVED_START_ADDRESS;
  HAL_FLASH_Unlock();

  EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  EraseInitStruct.Sector = WriteSector;
  EraseInitStruct.NbSectors = 1;
  while (HAL_OK != HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError))
    ;

  /*start program flash*/
  while (len--)
  {
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, start_addr, *pbuff);
    start_addr++;
    pbuff++;
  }
  /*write data end*/
  HAL_FLASH_Lock();
  return 0;
}
