RS485 Bootloader для stm32f103c8t6, после подтверждения коннекта с загрузчиком приложение пересылает микроконтроллеру hex-файл прошивки, микроконтроллер сохраняет его в i2c EEPROM, а после записывает из EEPROM в свою Flash. В случае сбоя при перезагрузке видит новую прошивку в EEPROM и обновляет Flash.