# STM32 Satellite Simulator And ESP32 Base Station

## Project Structure

ESP32-FLOOR-BASE/  
├── esp32_floor_base.ino  

LICENSE  

STM32-SIM/  
├── Core/  
│   ├── Inc/            # Main headers  
│   ├── Src/            # Main sources  
│   └── Startup/        # Startup files  
├── Drivers/  
│   ├── CMSIS/          # CMSIS device and cores  
│   └── STM32F4xx_HAL_Driver/  # HAL and peripheral drivers  
├── Middlewares/  
│   └── FreeRTOS/       # RTOS and related files  
├── STM32F446RETX_FLASH.ld  
├── STM32F446RETX_RAM.ld  
└── Satelite.ioc        # CubeMX project  
