# Kontrol PSU Anodize - STM32CubeIDE Project

## Deskripsi Proyek
Firmware ini adalah sistem kontrol PSU (Power Supply Unit) anodize berbasis STM32F4, yang mengelola beberapa timer, display, input, buzzer, dan komunikasi Modbus. Firmware menggunakan FreeRTOS untuk multitasking dan mendukung perangkat eksternal seperti LCD I2C, MAX7219 (7-segment), 74HC595 (shift register), serta komunikasi UART dan SPI.

---

## Struktur File Utama

### 1. `main.c`
- Inisialisasi periferal: I2C, SPI, UART, Timer, GPIO, dan FreeRTOS.
- Definisi struktur data utama (`TimerInfo_t`, enum mode operasi, dsb).
- Implementasi multitasking (task RTOS):
  - Timer manager
  - Display manager
  - Input handler
  - Buzzer
  - Modbus master
- Driver perangkat: MAX7219, 74HC595, LCD I2C
- Fungsi Modbus (CRC16, pengiriman register)
- Sinkronisasi data antar task (mutex, semaphore)
- Helper: format waktu, inisialisasi timer, dsb.

### 2. `main.h`
- Definisi struktur data (`TimerInfo_t` dengan field arus, waktu, status)
- Konstanta & macro (jumlah timer, pin GPIO, dsb)
- Deklarasi fungsi utama
- Include file HAL, FreeRTOS, dan library eksternal

---

## Fitur Utama
- Multi timer (hingga 8 timer independen)
- Display ganda: LCD I2C & 7-segment (MAX7219)
- Input encoder & tombol
- Buzzer untuk notifikasi
- Modbus master (pengiriman data timer)
- Penyimpanan flash (dengan `flash_storage.h/c`)

---

## Struktur Data Penting
```c
// TimerInfo_t dari main.h
typedef struct {
  uint32_t remaining_seconds; // Waktu tersisa (detik)
  uint16_t current_mA;        // Arus (mA)
  bool is_running;            // Status berjalan
  bool is_finished;           // Status selesai
} TimerInfo_t;
```

---

## Cara Kerja Singkat
1. Semua periferal dan task diinisialisasi di `main()`
2. Task berjalan paralel sesuai fungsinya
3. Data timer dilindungi mutex
4. Data timer dikirim ke slave Modbus secara periodik
5. User dapat mengatur timer via encoder/tombol, status tampil di LCD/7-segment

---

## Catatan
- Pastikan konfigurasi pin di CubeMX sesuai dengan definisi di `main.h`
- Untuk menambah/mengurangi jumlah timer, ubah `NUM_TIMERS` di kedua file
- Library eksternal seperti `i2c-lcd.h` dan `flash_storage.h` harus tersedia di folder `Inc/` dan `Src/`
