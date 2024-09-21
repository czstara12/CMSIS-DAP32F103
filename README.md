# CMSIS-DAP

这是CMSIS-DAP在STM32F103上的移植,尽量参考官方实现,使用原版方案同款RL-USB,RTX5和最新编译器AC6.

使用HAL库外加RTE导入包

## 已知问题

最新的包对于F103这款老芯片的支持不佳 需要使用固定版本的方案才能成功编译 推荐使用的MDK版本5.36

HAL框架和RTE框架在F103上有很大的冲突 比如两个框架各有一套CMSIS-Device驱动 但是头文件名和内容有很大差别 并且硬编码在各自的驱动库中 而中间件又依赖一些驱动库 
另外两个框架也各自有各自的启动文件

不太可能塞到32kROM 10kRAM的C6T6上了

## 备注

针对USBD_STM32F10x.c添加了

```
__packed=__unaligned
```

宏定义 用于解决AC6编译错误

USB配置页面的代码根本没有用

```
//           <h>Data
//             <s.1024>Unicode String
//             <i>Property Data in case Data Type is selected as Unicode String.
#define USBD_CUSTOM_CLASS0_IF0_OS_EXT_PROP0_DATA_STR_RAW           "{CDB3B5AD-293B-4663-AA36-1AAE46463776}"
//{CDB3B5AD-293B-4663-AA36-1AAE46463776}
//{7D9ADCFC-E570-4B38-BF4E-8F81F68964E0}
```

使用c11

删除包含目录../Drivers/CMSIS/Include

这个函数必须写成这样

```c
/** SWDIO I/O pin: Set Output (used in SWD mode only).
\param bit Output value for the SWDIO DAP hardware I/O pin.
*/
__STATIC_FORCEINLINE void     PIN_SWDIO_OUT     (uint32_t bit) {
  HAL_GPIO_WritePin(SWDIO_GPIO_Port, SWDIO_Pin, (bit & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
```

而不能写成这样

```c
/** SWDIO I/O pin: Set Output (used in SWD mode only).
\param bit Output value for the SWDIO DAP hardware I/O pin.
*/
__STATIC_FORCEINLINE void     PIN_SWDIO_OUT     (uint32_t bit) {
  HAL_GPIO_WritePin(SWDIO_GPIO_Port, SWDIO_Pin, (bit == 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
```

必须同时启动USB CDC dap才能正常工作 PID/VID也需要精心挑选(使用示例中的)

## TODO

SPI驱动SW电平逻辑

顺势加入虚拟串口

SWO

JTAG