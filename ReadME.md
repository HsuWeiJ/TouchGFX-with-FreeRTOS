
# TouchGFX with FreeRTOS

在此專案中，使用STM32F429ZIT6 Discovery進行開發，該開發板內嵌觸控式面板，可藉由ST官方提供的TouchGFX Designer進行UI的開發，並使用Free RTOS建立一個多工處理的整合系統，融合RFID門禁管理、溫溼度感測、PWM 控制電壓轉換器、電壓感測、SD卡儲存、Flash讀寫以及CPU資源管理等功能。

本專案使用CubeIDE搭配CubeMX進行開發，硬體外設的接腳可自定義調整。

## ****Project Description****

此系統使用RFID進行登入的管控，在登入完成後UI會切換至主頁面，而主頁面為一實時時鐘以及顯示當下的CPU使用率，並且使用者可以自定義當前的時間。在其他的頁面中，使用者可以獲取當前的溫濕度，以及控制外接的電壓轉換器，並即時獲取當前電壓轉換器的輸出。而在登入的期間，SD卡會每秒將當前獲取的資料儲存在SD卡之中。在登出之後會將最新的資料儲存在開發板內部的Flash中，等待下次登入時將資料從Flash讀回做使用。

## Brief **Architecture**

![image](https://github.com/HsuWeiJ/TouchGFX-with-FreeRTOS/blob/master/RTOS_SideProject.drawio.png)

將獨立的功能切分成各個任務，總共分成6個Tasks以及1個Periodic Software Timer

- **Log Task** : 管理登入或是登出的功能，需要處理RFID Access的資料，利用Queue通知其他Task有沒有通過access
    - 登入:
        1. 接受RFID_UART_ISR傳送的資料，判斷是否Access成功
        2. 啟動software timer，如果Access成功，將在兩秒後，進入操作頁面 ; 失敗則在兩秒後，消除授權結果並等待下次RFID掃描
        3. 登入成功後，Resume其他的tasks，並讀取NOR Flash中，溫濕度、電壓以及duty資料，來傳給各自的tasks
    - 登出:
        1. 檢查EventGroups的事件，確認是在哪個頁面登出
        2. 登出後返回Home Screen，並suspend其他Tasks以及將最後一筆的溫濕度、電壓和duty資料存入NOR Flash，提供下次登入使用
- **SD Task** : 定期儲存當前時間、溫濕度、電壓和duty資料
    - 使用Queue接收其他tasks傳送的儲存資料
- **PWM Task** : 管理PWM的duty cycle來控制Buck Converter
    - 使用Queue接受GUI Task的資料，並以此更改TIM2→CCR1的設定值來改變Duty
- **GUI Task** : 管理觸控屏的顯示以及觸控的所有事件，並透過IPC API與其他任務做通訊
    - 使用Queue接受顯示資料並顯示在螢幕上
        1. Home Screen 接收RTC時間以及CPU Usage
        2. Home Setting Screen 根據捲動選單的資料，設定RTC的日期以及時間
        3. Temperature Screen 接收溫溼度資料
        4. Voltage Screen  接收電壓資料
        5. Log Screen 接收 RFID 以及 Access result
    - 使用Queue傳送在Voltage Screen  更改的duty
    - 使用EventGroup 記錄最後是哪個頁面登出，事件觸發後會傳送給Log Task
- **Tem Task** : 定期量測溫濕度資料，並使用DMA傳輸做CPU資源的優化
    - 使用Queue傳送溫溼度資料給觸控屏以及SD卡使用
- **ADC Task** : 定期量測Buck Converter的電壓output
    - 使用Queue傳送電壓資料給觸控屏以及SD卡使用FLASH_SECTOR_F4
- **Periodic Software Timer** : 設定每1000ms，讀取RTC寄存器的時間，以及計算CPU使用率，並傳送給GUI Task顯示

## Files

- Application / User : 使用者自定義的函式
    - user_include : 內含使用者新增的標頭檔
    - cpu_utils.c : 計算CPU使用率
    - FLASH_SECTOR_F4.c : Flash檔案讀寫
    - SD_SPI_FATFS.c : SD卡底層IO API
    - File_Handling_RTOS.c : FATFS檔案讀寫
    - SHT31.c : 溫溼度感測
- Core / Src
    - main .c : 主程式，RTOS的所有任務皆寫在此
    - stm32f4xx_it.c : 設定Interrupt Handler
- Driver : ST官方提供的Hal函式庫
- FATFS /Target :
    - user_diskio.c : 需將SD_SPI_FATFS.c完成的檔案操作底層IO API移植進此檔案
- Middlewares / Third_Party
    - FATFS : File Allocation Table File System 的source code
    - FreeRTOS : FreeRTOS 的source code
- TouchGFX : 由TouchGFX Designer產生
    - generated : 由TouchGFX Designer產生，且不可變更裡面的程式
    - gui : 使用者能自定義修改的程式區間，進行GUI Task 跟其他 Task的溝通
- STM32F429I-DISCO.ioc : 開發板的腳位、時脈設定

## Notes

- [Notion](https://www.notion.so/TouchGFX-with-FreeRTOS-ecd2d226f7b54d118a934fad421d8d8a)
- [PDF](https://drive.google.com/file/d/1pdhXr2TmjLs4nE3cDYEdf_HihZ9Uv4F7/view?usp=sharing) (Notion匯出的版本，可能會跑板)
