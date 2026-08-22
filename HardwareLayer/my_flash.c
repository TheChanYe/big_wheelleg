/**
  ******************************************************************************
  * @file    flash_driver.c
  * @brief   AT32F403ARGT7 Flash Driver with interrupt protection
  ******************************************************************************
  * 注意事项：
  * 1. 操作前确保地址已按2字节对齐（半字操作）
  * 2. 写入前必须保证目标区域已被擦除
  * 3. 擦除操作以2KB页为单位
  * 4. 操作期间CPU时钟必须保持稳定
  * 5. 不要操作正在执行代码的Flash区域
  * 6. 写操作前建议关闭所有中断
  ******************************************************************************
  */

#include "my_flash.h"
#define MODULE_NAME       "my_flash"

#ifdef  MODE_LOG_TAG
#undef  MODE_LOG_TAG
#endif
#define MODE_LOG_TAG          MODULE_NAME

/* 芯片特定参数 */
#define FLASH_BASE_ADDR         0x08000000  /*flash起始地址*/
#define FLASH_SIZE              (1024 * 1024)  /* flashz总大小1024KB */
#define FLASH_PAGE_SIZE         2048          /* 2KB per page */
#define FLASH_TIMEOUT           1000000

/* 内部缓冲区 */
static uint16_t page_buffer[FLASH_PAGE_SIZE/2] __attribute__((aligned(4)));

int flash_write_nocheck(uint32_t write_addr, uint16_t *p_buffer, uint16_t size)
{
  uint16_t i;
  flash_status_type status = FLASH_OPERATE_DONE; 

    if((write_addr < FLASH_BASE_ADDR) || 
       (write_addr + size > FLASH_BASE_ADDR + FLASH_SIZE))
    {
        return E_PARAM;
    }

		for(i = 0; i < size; i++)
		{
			status = flash_halfword_program(write_addr, p_buffer[i]);
			if(status != FLASH_OPERATE_DONE)
				return E_ERROR;
			write_addr += 2;
		}
		return E_OK;
}


/**
  * @brief  数据读取
  */
int flash_read(uint32_t addr, void *buffer, uint16_t size)
{
    if((addr < FLASH_BASE_ADDR) || 
       (addr + size > FLASH_BASE_ADDR + FLASH_SIZE))
    {
        return E_PARAM;
    }

    /* 直接内存访问读取 */
    memcpy(buffer, (void*)addr, size);
    return E_OK;
}


/**
  * @brief  带中断保护的页擦除
  */
int flash_erasepage(uint32_t page_address)
{
    flash_status_type status;
    int ret = 0;
    uint32_t primask;
    
    /* 计算实际页地址 */
    page_address = FLASH_BASE_ADDR + 
                 ((page_address - FLASH_BASE_ADDR) / FLASH_PAGE_SIZE) * FLASH_PAGE_SIZE;

    primask = __get_PRIMASK();  /* 保存中断状态 */
    __disable_irq();            /* 关中断 */

    /* 执行擦除操作 */
    status = flash_sector_erase(page_address);
    if(status == FLASH_OPERATE_DONE)
    {
        status = flash_operation_wait_for(FLASH_TIMEOUT);//等待flash炒作
         if (status == FLASH_OPERATE_DONE)
					ret = E_OK;
    }
    
    /* 清除可能的错误标志 */
    if(flash_flag_get(FLASH_PRGMERR_FLAG | FLASH_EPPERR_FLAG))
    {
        flash_flag_clear(FLASH_PRGMERR_FLAG | FLASH_EPPERR_FLAG);
        ret = E_ERROR;
    }

    __set_PRIMASK(primask);  /* 恢复中断状态 */
    return ret;
}

/**
  * @brief  带中断保护的数据写入
  */
/* Flash写入函数（带中断保护）*/
int flash_write(uint32_t addr, uint16_t *data, uint16_t size)
{
    uint32_t primask;          // 用于保存中断状态寄存器
    int ret = E_OK;           // 返回状态
    // 局部变量用于存储地址偏移、扇区位置、偏移量、剩余空间等信息
    uint32_t offset_addr;
    uint32_t sector_position;
    uint16_t sector_offset;
    uint16_t sector_remain;
    uint16_t i;
    flash_status_type status;

    /*---------- 参数有效性检查 ----------*/
    // 检查地址是否在Flash范围内
    // 检查写入范围是否超出Flash容量
    // 检查写入数据长度是否为0
    if((addr < FLASH_BASE_ADDR) || (addr + size > FLASH_BASE_ADDR + FLASH_SIZE) || (size == 0))
    {
        return E_PARAM;  // 参数非法立即返回
    }

    /*---------- 解锁Flash操作权限 ----------*/
    flash_unlock();  // 必须解锁才能进行擦写操作

   // 计算写入地址相对于闪存基地址的偏移量
    offset_addr = addr - FLASH_BASE;
    // 计算写入地址所在的扇区位置
    sector_position = offset_addr / FLASH_PAGE_SIZE;
    // 计算写入地址在扇区内的偏移量（以半字为单位）
    sector_offset = (offset_addr % FLASH_PAGE_SIZE) / 2;
    // 计算当前扇区剩余的可写入半字数量
    sector_remain = FLASH_PAGE_SIZE / 2 - sector_offset;
    // 如果要写入的数量小于等于当前扇区剩余空间，则更新剩余空间为要写入的数量
    if (size <= sector_remain)
		{
        sector_remain = size;
    }
   // 循环处理要写入的数据，直到全部写入完成
    while (1)
		{
        // 读取当前扇区的数据到缓冲区
        /* page_buffer 容量为整页；读取长度单位为字节，必须完整备份 2KB 页。 */
        flash_read(sector_position * FLASH_PAGE_SIZE + FLASH_BASE, page_buffer, FLASH_PAGE_SIZE);
        // 检查扇区内是否有非 0xFFFF 的数据
        for (i = 0; i < sector_remain; i++)
				{	
            if (page_buffer[sector_offset + i] != 0xFFFF)
						{
                break;
            }
        }
        // 如果扇区内有非 0xFFFF 的数据，需要擦除该扇区
        if (i < sector_remain) 
				{
						/*---------- 中断保护处理 ----------*/
					primask = __get_PRIMASK();  // 保存当前中断状态
					__disable_irq();            // 关闭全局中断
					
					// 等待擦除操作完成
					status = flash_operation_wait_for(ERASE_TIMEOUT);
					if (status == FLASH_PROGRAM_ERROR || status == FLASH_EPP_ERROR)
					{
						// 清除错误标志
						flash_flag_clear(FLASH_PRGMERR_FLAG | FLASH_EPPERR_FLAG);
					} 
					else if (status == FLASH_OPERATE_TIMEOUT)
					{
						// 操作超时，锁定闪存并返回错误
						flash_lock();
						__set_PRIMASK(primask);  // 恢复中断状态
						return ERROR;
					}

					// 擦除当前扇区
					status = flash_sector_erase(sector_position * FLASH_PAGE_SIZE + FLASH_BASE);
					if (status != FLASH_OPERATE_DONE) 
					{
							// 擦除失败，锁定闪存并返回错误
							flash_lock();
							__set_PRIMASK(primask);  // 恢复中断状态
							return ERROR;
					}
					// 将数据复制到缓冲区
					for (i = 0; i < sector_remain; i++)
					{
							page_buffer[i + sector_offset] = data[i];
					}

					// 无检查写入整个扇区的数据
					ret = flash_write_nocheck(sector_position * FLASH_PAGE_SIZE + FLASH_BASE, page_buffer, FLASH_PAGE_SIZE / 2);			
					if (ret!= E_OK) 
					{
							// 写入失败，锁定闪存并返回错误
							flash_lock();
							__set_PRIMASK(primask);  // 恢复中断状态
							return ERROR;
					}
       } 
				else
				{
						/*---------- 中断保护处理 ----------*/
						primask = __get_PRIMASK();  // 保存当前中断状态
						__disable_irq();            // 关闭全局中断
					
						// 如果扇区内全是 0xFFFF，直接写入数据
						ret = flash_write_nocheck(addr, data, sector_remain);
						if (ret!= E_OK) 
						{
							// 写入失败，锁定闪存并返回错误
							flash_lock();
							__set_PRIMASK(primask);  // 恢复中断状态
										return ERROR;
						}
				}

				// 如果已经写入了所有数据，退出循环
				if (size == sector_remain) 
				{
						break;
				} 
				else
				{
					// 处理下一个扇区
					sector_position++;
					sector_offset = 0;
					data += sector_remain;
					addr += (sector_remain * 2);
					size -= sector_remain;
					if (size > (FLASH_PAGE_SIZE / 2))
					{
							sector_remain = FLASH_PAGE_SIZE / 2;
					} 
					else 
					{
						 sector_remain = size;
					}
				}
    }
    /*---------- 收尾处理 ----------*/
    flash_lock();          // 重新锁定Flash
    __set_PRIMASK(primask);  // 恢复中断状态
    return E_OK;            // 返回最终操作状态
}


