
/** Put this in the src folder **/

#include "i2c-lcd.h"
#include "string.h"	  // Diperlukan untuk fungsi strlen()
#include "cmsis_os.h" // WAJIB: Sertakan header CMSIS-OS untuk osDelay()

// Pastikan handler I2C ini sesuai dengan yang ada di proyek Anda
extern I2C_HandleTypeDef hi2c1;

// Alamat slave I2C untuk modul LCD Anda. 0x4E atau 0x7E adalah alamat umum.
// Mungkin juga 0x27 atau 0x3F tergantung pada chip I/O expander yang digunakan.
#define SLAVE_ADDRESS_LCD 0x4E

// Definisikan panjang baris LCD Anda di sini agar mudah diubah
#define LCD_LINE_LENGTH 16

void lcd_send_cmd(char cmd)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd & 0xf0);
	data_l = ((cmd << 4) & 0xf0);
	data_t[0] = data_u | 0x0C; // en=1, rs=0 -> bxxxx1100
	data_t[1] = data_u | 0x08; // en=0, rs=0 -> bxxxx1000
	data_t[2] = data_l | 0x0C; // en=1, rs=0 -> bxxxx1100
	data_t[3] = data_l | 0x08; // en=0, rs=0 -> bxxxx1000
	HAL_I2C_Master_Transmit(&hi2c1, SLAVE_ADDRESS_LCD, (uint8_t *)data_t, 4, 100);
}

void lcd_send_data(char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data & 0xf0);
	data_l = ((data << 4) & 0xf0);
	data_t[0] = data_u | 0x0D; // en=1, rs=0 -> bxxxx1101
	data_t[1] = data_u | 0x09; // en=0, rs=0 -> bxxxx1001
	data_t[2] = data_l | 0x0D; // en=1, rs=0 -> bxxxx1101
	data_t[3] = data_l | 0x09; // en=0, rs=0 -> bxxxx1001
	HAL_I2C_Master_Transmit(&hi2c1, SLAVE_ADDRESS_LCD, (uint8_t *)data_t, 4, 100);
}

void lcd_clear(void)
{
	lcd_send_cmd(0x80);
	for (int i = 0; i < 70; i++)
	{
		lcd_send_data(' ');
	}
}

void lcd_put_cur(int row, int col)
{
	switch (row)
	{
	case 0:
		col |= 0x80;
		break;
	case 1:
		col |= 0xC0;
		break;
	}

	lcd_send_cmd(col);
}

void lcd_init(void)
{
	osDelay(50);
	lcd_send_cmd(0x30);
	osDelay(5);
	lcd_send_cmd(0x30);
	osDelay(1);
	lcd_send_cmd(0x30);
	osDelay(10);
	lcd_send_cmd(0x20);
	osDelay(10);
	lcd_send_cmd(0x28);
	osDelay(1);
	lcd_send_cmd(0x08);
	osDelay(1);
	lcd_send_cmd(0x01);
	osDelay(2);
	lcd_send_cmd(0x06);
	osDelay(1);
	lcd_send_cmd(0x0C);
}

void lcd_send_string(char *str)
{
	int i;
	// Hitung panjang string yang sebenarnya
	int len = strlen(str);

	// Kirim setiap karakter dari string
	for (i = 0; i < len; i++)
	{
		lcd_send_data(str[i]);
	}

	// Isi sisa baris dengan spasi untuk menghapus karakter lama
	for (i = len; i < LCD_LINE_LENGTH; i++)
	{
		lcd_send_data(' ');
	}
}
