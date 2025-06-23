#include "flash_storage.h"

/**
 * @brief Menghapus sektor flash yang ditentukan untuk data pengguna.
 * @retval true jika berhasil, false jika gagal.
 */
static bool Flash_Erase_Sector(void)
{
    HAL_StatusTypeDef status;
    FLASH_EraseInitTypeDef erase_init;
    uint32_t sector_error = 0;

    // Inisialisasi struktur penghapusan flash
    erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3; // Khas untuk STM32F4
    erase_init.Sector = FLASH_USER_SECTOR;
    erase_init.NbSectors = 1;

    // Lakukan penghapusan sektor
    status = HAL_FLASHEx_Erase(&erase_init, &sector_error);

    return (status == HAL_OK);
}

/**
 * @brief Menulis data pengaturan dari semua timer ke flash memory.
 */
bool Flash_Write_Data(TimerInfo_t *timers_data, uint16_t num_timers)
{
    // 1. Buka kunci flash memory untuk operasi penulisan
    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return false;
    }

    // 2. Hapus sektor sebelum menulis data baru
    if (!Flash_Erase_Sector())
    {
        HAL_FLASH_Lock(); // Kunci kembali flash jika penghapusan gagal
        return false;
    }

    // 3. Tulis data baru ke flash.
    // Ukuran TimerInfo_t adalah 8 byte (2 word), jadi kita bisa menulis word by word (32-bit).
    uint32_t address = FLASH_USER_SECTOR_ADDR;
    uint32_t *data_ptr = (uint32_t *)timers_data;
    uint16_t num_words = (sizeof(TimerInfo_t) * num_timers) / 4;

    for (uint16_t i = 0; i < num_words; i++)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, data_ptr[i]) != HAL_OK)
        {
            HAL_FLASH_Lock(); // Kunci kembali flash jika terjadi error
            return false;     // Gagal menulis
        }
        address += 4; // Pindah ke word berikutnya
    }

    // 4. Kunci kembali flash memory setelah selesai
    HAL_FLASH_Lock();

    return true; // Penulisan berhasil
}

/**
 * @brief Membaca data pengaturan semua timer dari flash memory.
 */
void Flash_Read_Data(TimerInfo_t *timers_data, uint16_t num_timers)
{
    uint32_t address = FLASH_USER_SECTOR_ADDR;
    uint8_t *dest_ptr = (uint8_t *)timers_data;
    uint16_t total_bytes = sizeof(TimerInfo_t) * num_timers;

    for (uint16_t i = 0; i < total_bytes; i++)
    {
        // Membaca dari flash semudah membaca dari memori biasa
        dest_ptr[i] = *(__IO uint8_t *)(address + i);
    }
}
