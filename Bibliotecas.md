## Bibliotecas necessárias

### 1. Comunicação I²C (MPU6050 e SSD1306)
- **driver/i2c.h**  
- **esp_log.h**  

### 2. Comunicação SPI e SD-Card
- **driver/spi_master.h**  
- **sdmmc_cmd.h** _ou_ **driver/sdspi_host.h**  
- **esp_vfs_fat.h**  

### 3. Controle de GPIO (Botões e Buzzer)
- **driver/gpio.h**  
- **driver/ledc.h**  

### 4. Multithreading e Sincronização (FreeRTOS)
- **freertos/FreeRTOS.h**  
- **freertos/task.h**  
- **freertos/queue.h**  
- **freertos/semphr.h**  

### 5. Utilitários e Logging
- **esp_system.h**  
- **esp_err.h**  
- **esp_log.h**  
- **esp_timer.h**  
