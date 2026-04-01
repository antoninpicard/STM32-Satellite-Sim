# Satellite Simulator Project

## Arborescence du projet

ESP32-FLOOR-BASE
├── esp32_floor_base.ino

LICENSE

STM32-SIM
├── Core
│   ├── Inc
│   │   ├── FreeRTOSConfig.h
│   │   ├── main.h
│   │   ├── ssd1306.h
│   │   ├── ssd1306_conf.h
│   │   ├── ssd1306_fonts.h
│   │   ├── stm32f4xx_hal_conf.h
│   │   └── stm32f4xx_it.h
│   ├── Src
│   │   ├── freertos.c
│   │   ├── main.c
│   │   ├── ssd1306.c
│   │   ├── ssd1306_fonts.c
│   │   ├── stm32f4xx_hal_msp.c
│   │   ├── stm32f4xx_hal_timebase_tim.c
│   │   ├── stm32f4xx_it.c
│   │   ├── syscalls.c
│   │   ├── sysmem.c
│   │   └── system_stm32f4xx.c
│   └── Startup
│       └── startup_stm32f446retx.s
├── Drivers
│   ├── CMSIS
│   │   ├── Device/ST/STM32F4xx
│   │   │   ├── Include
│   │   │   │   ├── stm32f446xx.h
│   │   │   │   ├── stm32f4xx.h
│   │   │   │   └── system_stm32f4xx.h
│   │   │   ├── LICENSE.txt
│   │   │   └── Source/Templates
│   │   ├── Include
│   │   │   └── [fichiers CMSIS divers]
│   │   └── LICENSE.txt
│   └── STM32F4xx_HAL_Driver
│       ├── Inc
│       │   ├── Legacy/stm32_hal_legacy.h
│       │   └── [fichiers HAL divers]
│       ├── Src
│       │   └── [fichiers HAL source divers]
│       └── LICENSE.txt
├── Middlewares
│   └── Third_Party/FreeRTOS/Source
│       ├── CMSIS_RTOS_V2
│       │   ├── cmsis_os.h
│       │   ├── cmsis_os2.c
│       │   ├── cmsis_os2.h
│       │   ├── freertos_mpool.h
│       │   └── freertos_os2.h
│       ├── LICENSE
│       ├── croutine.c
│       ├── event_groups.c
│       ├── include
│       │   ├── FreeRTOS.h
│       │   ├── StackMacros.h
│       │   ├── atomic.h
│       │   ├── croutine.h
│       │   ├── deprecated_definitions.h
│       │   ├── event_groups.h
│       │   ├── list.h
│       │   ├── message_buffer.h
│       │   ├── mpu_prototypes.h
│       │   ├── mpu_wrappers.h
│       │   ├── portable.h
│       │   ├── projdefs.h
│       │   ├── queue.h
│       │   ├── semphr.h
│       │   ├── stack_macros.h
│       │   ├── stream_buffer.h
│       │   ├── task.h
│       │   └── timers.h
│       ├── list.c
│       ├── portable/GCC/ARM_CM4F
│       │   ├── port.c
│       │   └── portmacro.h
│       ├── portable/MemMang/heap_4.c
│       ├── queue.c
│       ├── stream_buffer.c
│       ├── tasks.c
│       └── timers.c
├── STM32F446RETX_FLASH.ld
├── STM32F446RETX_RAM.ld
└── Satelite.ioc