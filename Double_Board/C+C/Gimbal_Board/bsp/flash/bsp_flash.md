
# bsp_flash

<p align='right'>LSJhero@example.com</p>

## 模块概述

bsp_flash模块提供了STM32微控制器Flash存储器的基本操作功能，包括Flash扇区擦除、单页写入、多页写入和数据读取等操作。该模块基于STM32 HAL库实现，为上层应用提供简单易用的Flash操作接口。

## 使用说明

在使用Flash相关功能前，请确保了解以下注意事项：

1. Flash写入前必须先擦除对应扇区
2. Flash具有写入次数限制，应避免频繁写入
3. 写入地址必须是4字节对齐的
4. 写入操作会消耗较大电流，在低功耗应用中需注意

## 代码结构

.h文件包含了Flash相关的宏定义、外部接口声明和类型定义；
.c文件实现了所有外部接口和必要的私有辅助函数。

## 类型定义与宏定义

```c
/* Flash扇区基地址定义 */
#define ADDR_FLASH_SECTOR_0 ((uint32_t)0x08000000)  /* 扇区0基地址，16 Kbytes   */
#define ADDR_FLASH_SECTOR_1 ((uint32_t)0x08004000)  /* 扇区1基地址，16 Kbytes   */
#define ADDR_FLASH_SECTOR_2 ((uint32_t)0x08008000)  /* 扇区2基地址，16 Kbytes   */
#define ADDR_FLASH_SECTOR_3 ((uint32_t)0x0800C000)  /* 扇区3基地址，16 Kbytes   */
#define ADDR_FLASH_SECTOR_4 ((uint32_t)0x08010000)  /* 扇区4基地址，64 Kbytes   */
#define ADDR_FLASH_SECTOR_5 ((uint32_t)0x08020000)  /* 扇区5基地址，128 Kbytes  */
#define ADDR_FLASH_SECTOR_6 ((uint32_t)0x08040000)  /* 扇区6基地址，128 Kbytes  */
#define ADDR_FLASH_SECTOR_7 ((uint32_t)0x08060000)  /* 扇区7基地址，128 Kbytes  */
#define ADDR_FLASH_SECTOR_8 ((uint32_t)0x08080000)  /* 扇区8基地址，128 Kbytes  */
#define ADDR_FLASH_SECTOR_9 ((uint32_t)0x080A0000)  /* 扇区9基地址，128 Kbytes  */
#define ADDR_FLASH_SECTOR_10 ((uint32_t)0x080C0000) /* 扇区10基地址，128 Kbytes */
#define ADDR_FLASH_SECTOR_11 ((uint32_t)0x080E0000) /* 扇区11基地址，128 Kbytes */

#define FLASH_END_ADDR ((uint32_t)0x08100000)       /* Flash结束地址 */
```

- 上述宏定义了STM32 Flash各扇区的基地址和Flash存储器的结束地址
- 不同扇区的大小不同：扇区0-3为16KB，扇区4为64KB，扇区5-11为128KB

## 外部接口

### 1. Flash擦除函数

```c
void flash_erase_address(uint32_t address, uint16_t len);
```

- **功能**：擦除指定地址开始的连续Flash扇区
- **参数**：
  - `address`：擦除的起始Flash地址
  - `len`：要擦除的扇区数量
- **返回值**：无
- **说明**：该函数会解锁Flash、配置擦除参数、执行擦除操作，最后锁定Flash

### 2. 单页Flash写入函数

```c
int8_t flash_write_single_address(uint32_t start_address, uint32_t *buf, uint32_t len);
```

- **功能**：向Flash的单个页面写入数据
- **参数**：
  - `start_address`：写入的起始Flash地址
  - `buf`：指向要写入数据的缓冲区指针（32位字数组）
  - `len`：要写入的数据长度（以32位字为单位）
- **返回值**：成功返回0，失败返回-1
- **说明**：
  - 写入前必须确保目标Flash扇区已被擦除
  - 写入操作不会跨Flash页面边界
  - 使用32位字编程模式，每次写入4字节

### 3. 多页Flash写入函数

```c
int8_t flash_write_muli_address(uint32_t start_address, uint32_t end_address, uint32_t *buf, uint32_t len);
```

- **功能**：向Flash的多个页面写入数据
- **参数**：
  - `start_address`：写入的起始Flash地址
  - `end_address`：写入的结束Flash地址
  - `buf`：指向要写入数据的缓冲区指针
  - `len`：要写入的数据长度（以32位字为单位）
- **返回值**：成功返回0，失败返回-1
- **说明**：允许跨多个Flash页面写入数据，但写入前需要确保所有涉及的扇区已被擦除

### 4. Flash读取函数

```c
void flash_read(uint32_t address, uint32_t *buf, uint32_t len);
```

- **功能**：从Flash读取数据
- **参数**：
  - `address`：读取的起始Flash地址
  - `buf`：用于存储读取数据的缓冲区指针
  - `len`：要读取的数据长度（以32位字为单位）
- **返回值**：无
- **说明**：使用memcpy直接读取Flash内容，不需要解锁Flash

### 5. 获取下一页Flash地址函数

```c
uint32_t get_next_flash_address(uint32_t address);
```

- **功能**：获取指定地址所在Flash页面的下一个页面的起始地址
- **参数**：
  - `address`：Flash地址
- **返回值**：下一个Flash页面的起始地址
- **说明**：用于确定Flash页面边界，避免写入操作跨页面

## 私有函数

在.c文件内定义的static函数，仅供模块内部使用：

```c
static uint32_t ger_sector(uint32_t address);
```

- **功能**：根据给定的Flash地址获取对应的扇区号
- **参数**：
  - `address`：Flash地址
- **返回值**：对应的Flash扇区号
- **说明**：在flash_erase_address函数中使用，用于确定要擦除的扇区

## 实现细节

### Flash写入流程

1. 调用HAL_FLASH_Unlock()解锁Flash
2. 初始化写入地址和缓冲区指针
3. 循环执行Flash字编程操作：
   - 调用HAL_FLASH_Program()写入一个32位字
   - 成功则更新地址和指针，继续写入
   - 失败则锁定Flash并返回错误
4. 写入完成后调用HAL_FLASH_Lock()重新锁定Flash

### Flash擦除流程

1. 根据起始地址确定要擦除的扇区
2. 配置FLASH_EraseInitTypeDef结构体
3. 调用HAL_FLASH_Unlock()解锁Flash
4. 调用HAL_FLASHEx_Erase()执行扇区擦除
5. 调用HAL_FLASH_Lock()重新锁定Flash

## 注意事项

1. **擦除操作注意事项**：
   - Flash只能按扇区擦除，不能按字节擦除
   - 擦除会将整个扇区的所有位设置为1（0xFF）

2. **写入操作注意事项**：
   - 写入前必须确保目标区域已被擦除（值为0xFFFFFFFF）
   - 写入操作只能将1变为0，不能将0变为1
   - 地址必须是4字节对齐的

3. **安全考虑**：
   - 完成操作后务必调用HAL_FLASH_Lock()锁定Flash
   - 避免写入到包含程序代码的扇区

4. **性能考虑**：
   - Flash操作（尤其是擦除）比较耗时，应避免在实时性要求高的任务中执行
   - 可以考虑批量操作减少擦除和写入次数

5. **错误处理**：
   - 写入失败时会立即停止操作并返回错误码
   - 建议在调用写入函数后检查返回值以确保操作成功
        