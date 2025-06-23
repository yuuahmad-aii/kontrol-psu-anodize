#ifndef FLASH_STORAGE_H
#define FLASH_STORAGE_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h" // Diperlukan untuk tipe data TimerInfo_t
#include <stdbool.h>

/*
 * ==============================================================================
 * PERINGATAN: Memilih sektor yang salah dapat menimpa kode program Anda
 * dan membuat perangkat tidak dapat di-boot. Harap verifikasi alamat sektor
 * dan ukurannya dari datasheet mikrokontroler Anda.
 *
 * Untuk STM32F446RE (512KB Flash):
 * - Sektor 0-3: 16KB each
 * - Sektor 4: 64KB
 * - Sektor 5-7: 128KB each
 *
 * Alamat awal:
 * ...
 * Sektor 5: 0x08020000
 * Sektor 6: 0x08040000
 * Sektor 7: 0x08060000 -> Sektor ini dipilih untuk data pengguna.
 * ==============================================================================
 */
#define FLASH_USER_SECTOR_ADDR 0x08060000U
#define FLASH_USER_SECTOR FLASH_SECTOR_7

    /**
     * @brief Menulis data pengaturan dari semua timer ke flash memory.
     * @note Fungsi ini akan menghapus seluruh sektor sebelum menulis.
     * @param timers_data Pointer ke array data timer.
     * @param num_timers Jumlah timer dalam array.
     * @retval true jika penulisan berhasil, false jika gagal.
     */
    bool Flash_Write_Data(TimerInfo_t *timers_data, uint16_t num_timers);

    /**
     * @brief Membaca data pengaturan semua timer dari flash memory.
     * @param timers_data Pointer ke array untuk menyimpan data yang dibaca.
     * @param num_timers Jumlah timer untuk dibaca.
     */
    void Flash_Read_Data(TimerInfo_t *timers_data, uint16_t num_timers);

#ifdef __cplusplus
}
#endif

#endif /* FLASH_STORAGE_H */
