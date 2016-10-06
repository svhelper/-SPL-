Архитектура MetaLLL
===================

Типы объектов (ObjType)
=======================

- System Objects
  - MCU          - root object - bundle of all other objects
                   contains logic for verification of configuration at compile-time

  - sysclock     - configuration of system clock
                   contains MCU clock, clocks of Periphery busses, etc

- Low-Level
  - GPIO[]       - direct control, event / interrupt
  - ADC[]        - self calibration, analog inpit / mcu voltage / mcu temperature / analog watchdog
  - DAC[]        - 
  - DMA[]        - 
  - TIM[]        - timer (interrupt) / input capture / quadrature encoder / hall-sensor / output compare / PWM / complementary PWM / One-pulse mode output
  - RTC[]        - clock / alart (interrupt, wakeup)
  - WD[]         - IWD, WWD
  - FSMC[]       - SRAM, ROM, NOR Flash, NAND Flash, PSRAM
  - SDIO[]       - SD-card / MMC / CE-ATA
  - USB[]        - USB-DEV / USB-HOST / USB-OTG
  - CAN[]        - CAN-A / CAN-B
  - SPI[]        - master / multimaster / slave, full-duplex 3-lines / simplex 2-lines, transmitter / receiver / bi-directional (transiver)
  - SQI[]        - 
  - I2S[]        - master / slave
  - I2C[]        - master / multimaster / slave, I2C / PEC / SMBus / PMBus
  - UART[]       - master / slave, UART / USART / LIN / IrDA SIR / SMARTCARD / half-duplex (single wire) / multiprocessor / addressable slave
  - ETH[]        - Ethernet media access control (MAC) 
  - etc

- Middle Level
  - CLOCK[]      - TIM[] + period_ms
  - RTC_CLOCK[]  - RTC[]
  - RTC_ALARM[]  - RTC[]
  - Vmcu[]       - ADC[] + mcu_voltage
  - TEMP[]       - ADC[] + mcu_temperature
  - ADC_CAL[]    - ADC[] + self_calibration
  - SPI_TARGET[] - GPIO[] | SPI[]
  - I2C_TARGET[] - I2C[] + ADDR_ID
  - LCD_CMD[]    - GPIO[] | SPI_TARGET[] | I2C_TARGET[]
  - LCD_DATA[]   - GPIO[] | SPI_TARGET[] | I2C_TARGET[]
  - BTN_ARR[]    - GPIO[] | SPI_TARGET[] | I2C_TARGET[]
  - SDCARD[]     - GPIO[] | SDIO[]
  - FLASH[]      - GPIO[] | SPI_TARGET[] | I2C_TARGET[] | FSMC[]
  - PWM[]        - GPIO[] | TIM[]
  - USB-DEV[]    - GPIO[] | USB[]
  - USB-HOST[]   - GPIO[] | USB[]
  - USB-OTG[]    - GPIO[] | USB[]
  - CAN-OPEN[]   - GPIO[] | CAN[]
  - USBH_MSC[]   - USB-HOST[] + usb-pipe[] | USB-OTG[] + usb-pipe[]
  - etc

- High Level
  - LCD/OLED[]   - LCD_CFG[] | LCD_DATA[]
  - KEYPAD[]     - BTN_ARR[] + CLOCK[]
  - LED[]        - GPIO[] | PWM[]
  - FAT_FS[]     - SDCARD[] | FLASH[] | MCS[]
  - etc



Содержимое объектов
===================

- Идентификация
  - `ObjType` - Тип объекта
  - `ID`      - ID модуля / ID пина / адрес устройства итд

- Ссылки
  - `{ObjType, ID}`   - тип и ID родителя

- Ресурсы
  - `{ObjType, ID}[]` - базовый модуль, GPIO пины, DMA каналы / стримы
  
- Конфигурационные параметры
  - `_cfg_`    - структура с константами или типами, содержимое зависит от ObjType

- Параметры объкта
  - `_set_`    - структура с константами или типами, содержимое зависит от самого объекта и целевой платформы (MCU)

- Анализ используемых ресурсов:
  - метафункция для обхода всех ресурсов для поиска объекта по его идентификационной информации

- Служебные функции (статические методы):
  - `init<sysclock>` - инициализация объекта
         `sysclock`  - настройки системы (тактирование), отсутствует для объекта "sysclock"
  
  - `deinit`         - деинициализация объекта
  
  - `on_sysclock_changing<sysclock> :: starting`   - временное блокирование операций, ожидание завершения активной
  - `on_sysclock_changing<sysclock> :: finished`   - перенастройка тактовой частоты, разблокирование операций
  
  - `on_irq<IRQ_LEVEL>` - уведомление о событии


Иерархия объектов
=================

Пример иерархии зависимостей

~~~
MCU<sysclock>
 +----> High Level Module 1
 |                        +--> Low-Level Module 1
 +--> Low-Level Module 2  +--> Middle Level Module 1
 |                             ^                   +--> Low-Level Module 3
 |                             |                   +--> Low-Level Module 4
 +--> High Level Module 2      |                   |                     +--> Low-Level Module 5
 |                      +------+                   |                     +--> Low-Level Module 6
 |                      +--> Low-Level Module 7    |
 |                                                 +--> Low-Level Module 8
 |                                                      ^
 +--> Middle Level Module 2                             |
 |                        +-----------------------------+
 |                        +--> Low-Level Module 9
 |
 +--> Low-Level Module 10
~~~

Как можно видеть, подобную нетривиальную иерархию может оказаться не просто описать даже.
Вручную ее вполне возможно скофигурировать единожды.
Вот только поддерживать ее в актуальном состоянии без Стандартной Библиотеки Периферии проблематично.
Скажем большое спасибо производителям, за предоставление подобных библиотек разработчикам!

Но есть задачи, которые практически нереально выполнить, даже используя Стандартную Библиотеку Периферии - это:
- смена тактовой частоты шины периферии с сохранением настроек режима тактирования самой периферии или с допустимой погрешностью
- динамическое включение/отключение периферии, при смене режима работы микроконтроллера
- абстрагирование подключения внешней периферии, автоматическое формирование максимально оптимизированного конфигурирования периферии, близкой к атомарной операции
- контроль адекватности и целостности системы на стадии компиляции
- контроль возможности переключения режимов, смены тактовой частоты на стадии компиляции
- при этом сохранить скорость управления периферией, соизмеримой прямому управлению ей


