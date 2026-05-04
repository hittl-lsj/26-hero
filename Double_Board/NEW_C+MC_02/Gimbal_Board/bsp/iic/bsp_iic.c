#include "bsp_iic.h"
#include "memory.h"
#include "stdlib.h"

static uint8_t idx = 0; // 配合中断以及初始化
static IICInstance *iic_instance[IIC_DEVICE_CNT] = {NULL};
// @brief 注册IIC实例,返回IICInstance指针
// @param conf IIC初始化配置结构体指针
// @return IICInstance* IIC实例指针
IICInstance *IICRegister(IIC_Init_Config_s *conf)
{
    if (idx >= MX_IIC_SLAVE_CNT) // 超过最大实例数
        while (1)                // 酌情增加允许的实例上限,也有可能是内存泄漏
            ;
    // 申请到的空间未必是0, 所以需要手动初始化
    IICInstance *instance = (IICInstance *)malloc(sizeof(IICInstance));
    instance = (IICInstance *)malloc(sizeof(IICInstance));
    memset(instance, 0, sizeof(IICInstance));

    instance->dev_address = conf->dev_address << 1; // 地址左移一位,最低位为读写位
    instance->callback = conf->callback;
    instance->work_mode = conf->work_mode;
    instance->handle = conf->handle;
    instance->id = conf->id;

    iic_instance[idx++] = instance;
    return instance;
}
// @brief 设置IIC设备工作模式
// @param iic IIC实例指针
// @param mode IIC工作模式(IIC_BLOCK_MODE, IIC_IT_MODE, IIC_DMA_MODE)
void IICSetMode(IICInstance *iic, IIC_Work_Mode_e mode)
{ // HAL自带重入保护,不需要手动终止或等待传输完成
    if (iic->work_mode != mode)
    {
        iic->work_mode = mode; // 如果不同才需要修改
    }
}
// @brief 向IIC设备传输数据
// @param iic IIC实例指针
// @param data 指向要传输数据的缓冲区指针
// @param size 要传输的数据长度(单位为字节)
// @param seq_mode 序列模式, 决定是否在传输完成后释放总线(IIC_SEQ_RELEASE)或保持总线(IIC_SEQ_HOLDON)
void IICTransmit(IICInstance *iic, uint8_t *data, uint16_t size, IIC_Seq_Mode_e seq_mode)
{
    if (seq_mode != IIC_SEQ_RELEASE && seq_mode != IIC_SEQ_HOLDON)
        while (1)
            ; // 未知传输模式, 程序停止

    // 根据不同的工作模式进行不同的传输
    switch (iic->work_mode)
    {
    case IIC_BLOCK_MODE:
        if (seq_mode != IIC_SEQ_RELEASE)
            while (1)
                ;                                                                // 阻塞模式下不支持HOLD ON模式!!!只能传输完成后立刻释放总线
        HAL_I2C_Master_Transmit(iic->handle, iic->dev_address, data, size, 100); // 默认超时时间100ms
        break;
    case IIC_IT_MODE:
        if (seq_mode == IIC_SEQ_RELEASE)
            HAL_I2C_Master_Seq_Transmit_IT(iic->handle, iic->dev_address, data, size, I2C_OTHER_AND_LAST_FRAME);
        else if (seq_mode == IIC_SEQ_HOLDON)
            HAL_I2C_Master_Seq_Transmit_IT(iic->handle, iic->dev_address, data, size, I2C_OTHER_FRAME);
        break;
    case IIC_DMA_MODE:
        if (seq_mode == IIC_SEQ_RELEASE)
            HAL_I2C_Master_Seq_Transmit_DMA(iic->handle, iic->dev_address, data, size, I2C_OTHER_AND_LAST_FRAME);
        else if (seq_mode == IIC_SEQ_HOLDON)
            HAL_I2C_Master_Seq_Transmit_DMA(iic->handle, iic->dev_address, data, size, I2C_OTHER_FRAME);
        break;
    default:
        while (1)
            ; // 未知传输模式, 程序停止
    }
}
// @brief 从IIC设备接收数据
// @param iic IIC实例指针
// @param data 指向要接收数据的缓冲区指针
// @param size 要接收的数据长度(单位为字节)
// @param seq_mode 序列模式, 决定是否在接收完成后释放总线(IIC_SEQ_RELEASE)或保持总线(IIC_SEQ_HOLDON)
void IICReceive(IICInstance *iic, uint8_t *data, uint16_t size, IIC_Seq_Mode_e seq_mode)
{
    if (seq_mode != IIC_SEQ_RELEASE && seq_mode != IIC_SEQ_HOLDON)
        while (1)
            ; // 未知传输模式, 程序停止,请检查指针越界

    // 初始化接收缓冲区地址以及接受长度, 用于中断回调函数
    iic->rx_buffer = data;
    iic->rx_len = size;

    switch (iic->work_mode)
    {
    case IIC_BLOCK_MODE:
        if (seq_mode != IIC_SEQ_RELEASE)
            while (1)
                ;                                                               // 阻塞模式下不支持HOLD ON模式!!!
        HAL_I2C_Master_Receive(iic->handle, iic->dev_address, data, size, 100); // 默认超时时间100ms
        break;
    case IIC_IT_MODE:
        if (seq_mode == IIC_SEQ_RELEASE)
            HAL_I2C_Master_Seq_Receive_IT(iic->handle, iic->dev_address, data, size, I2C_OTHER_AND_LAST_FRAME);
        else if (seq_mode == IIC_SEQ_HOLDON)
            HAL_I2C_Master_Seq_Receive_IT(iic->handle, iic->dev_address, data, size, I2C_OTHER_FRAME);
        break;
    case IIC_DMA_MODE:
        if (seq_mode == IIC_SEQ_RELEASE)
            HAL_I2C_Master_Seq_Receive_DMA(iic->handle, iic->dev_address, data, size, I2C_OTHER_AND_LAST_FRAME);
        else if (seq_mode == IIC_SEQ_HOLDON)
            HAL_I2C_Master_Seq_Receive_DMA(iic->handle, iic->dev_address, data, size, I2C_OTHER_FRAME);
        break;
    default:
        while (1)
            ; // 未知传输模式, 程序停止
        break;
    }
}
// @brief 访问IIC设备的内存空间
// @param iic IIC实例指针
// @param mem_addr 内存地址
// @param data 指向要传输数据的缓冲区指针(仅在写操作时使用)
// @param size 要传输的数据长度(单位为字节)
// @param mem_mode 内存操作模式(IIC_WRITE_MEM或IIC_READ_MEM)
// @param mem8bit_flag 内存地址是否为8位(I2C_MEMADD_SIZE_8BIT或I2C_MEMADD_SIZE_16BIT)
void IICAccessMem(IICInstance *iic, uint16_t mem_addr, uint8_t *data, uint16_t size, IIC_Mem_Mode_e mem_mode, uint8_t mem8bit_flag)
{
    uint16_t bit_flag = mem8bit_flag ? I2C_MEMADD_SIZE_8BIT : I2C_MEMADD_SIZE_16BIT;
    if (mem_mode == IIC_WRITE_MEM)
    {
        HAL_I2C_Mem_Write(iic->handle, iic->dev_address, mem_addr, bit_flag, data, size, 100);
    }
    else if (mem_mode == IIC_READ_MEM)
    {
        HAL_I2C_Mem_Read(iic->handle, iic->dev_address, mem_addr, bit_flag, data, size, 100);
    }
    else
    {
        while (1)
            ; // 未知模式, 程序停止
    }
}

/**
 * @brief IIC接收完成回调函数
 *
 * @param hi2c handle
 */
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    // 如果是当前i2c硬件发出的complete,且dev_address和之前发起接收的地址相同,同时回到函数不为空, 则调用回调函数
    for (uint8_t i = 0; i < idx; i++)
    {
        if (iic_instance[i]->handle == hi2c && hi2c->Devaddress == iic_instance[i]->dev_address)
        {
            if (iic_instance[i]->callback != NULL) // 回调函数不为空
                iic_instance[i]->callback(iic_instance[i]);
            return;
        }
    }
}

/**
 * @brief 内存访问回调函数,仅做形式上的封装,仍然使用HAL_I2C_MasterRxCpltCallback
 *
 * @param hi2c handle
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    HAL_I2C_MasterRxCpltCallback(hi2c);
}
