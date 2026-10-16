
#include "grove_bme280.h"
#include "board_pins.h"
 
extern I2C_HandleTypeDef GROVE_I2C_HANDLE;
 
// REGISTRES
 
#define BME280_CHIP_ID          0x60
#define BME280_REG_ID           0xD0
#define BME280_REG_RESET        0xE0
#define BME280_RESET_CMD        0xB6
 
#define BME280_REG_CALIB1       0x88  // 26 octets (0x88..0xA1)
#define BME280_REG_CALIB2       0xE1  // 7 octets  (0xE1..0xE7)
#define BME280_CALIB1_LEN       26
#define BME280_CALIB2_LEN       7
 
#define BME280_REG_CTRL_HUM     0xF2
#define BME280_REG_STATUS       0xF3
#define BME280_REG_CTRL_MEAS    0xF4
#define BME280_REG_CONFIG       0xF5
#define BME280_REG_DATA         0xF7  // 8 octets : pression (3), température (3), humidité (2)
 
// Suréchantillonnage : température x2, pression x4, humidité x1
#define BME280_OSRS_T           2U
#define BME280_OSRS_P           3U
#define BME280_OSRS_H           1U
#define BME280_MODE_FORCED      1U
 
#define BME280_STATUS_IM_UPDATE 0x01  // copie de la calibration en cours
#define BME280_MEAS_TIMEOUT_MS  100U  // une mesure dure environ 20 ms avec ces réglages
 
// Coefficients de calibration (propres à chaque capteur, lus au démarrage)
typedef struct
{
    uint16_t dig_t1; int16_t dig_t2; int16_t dig_t3;
    uint16_t dig_p1; int16_t dig_p2; int16_t dig_p3;
    int16_t  dig_p4; int16_t dig_p5; int16_t dig_p6;
    int16_t  dig_p7; int16_t dig_p8; int16_t dig_p9;
    uint8_t  dig_h1; int16_t dig_h2; uint8_t dig_h3;
    int16_t  dig_h4; int16_t dig_h5; int8_t  dig_h6;
} BME280_Calib;
 
static uint16_t bme280_addr = (0x76 << 1);
static BME280_Calib calib;
static bool bme280_pret = false;
 
// ACCÈS I2C
 
static bool read_reg(uint8_t reg, uint8_t *data, uint16_t len)
{
    return HAL_I2C_Mem_Read(&GROVE_I2C_HANDLE, bme280_addr, reg, I2C_MEMADD_SIZE_8BIT,
                            data, len, 100) == HAL_OK;
}
 
static bool write_reg(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(&GROVE_I2C_HANDLE, bme280_addr, reg, I2C_MEMADD_SIZE_8BIT,
                             &value, 1, 100) == HAL_OK;
}
 
// CALIBRATION
 
static bool lire_calibration(void)
{
    uint8_t c[BME280_CALIB1_LEN];
    uint8_t e[BME280_CALIB2_LEN];
 
    if (!read_reg(BME280_REG_CALIB1, c, BME280_CALIB1_LEN)) return false;
    if (!read_reg(BME280_REG_CALIB2, e, BME280_CALIB2_LEN)) return false;
 
    calib.dig_t1 = (uint16_t)((c[1] << 8) | c[0]);
    calib.dig_t2 = (int16_t)((c[3] << 8) | c[2]);
    calib.dig_t3 = (int16_t)((c[5] << 8) | c[4]);
 
    calib.dig_p1 = (uint16_t)((c[7] << 8) | c[6]);
    calib.dig_p2 = (int16_t)((c[9] << 8) | c[8]);
    calib.dig_p3 = (int16_t)((c[11] << 8) | c[10]);
    calib.dig_p4 = (int16_t)((c[13] << 8) | c[12]);
    calib.dig_p5 = (int16_t)((c[15] << 8) | c[14]);
    calib.dig_p6 = (int16_t)((c[17] << 8) | c[16]);
    calib.dig_p7 = (int16_t)((c[19] << 8) | c[18]);
    calib.dig_p8 = (int16_t)((c[21] << 8) | c[20]);
    calib.dig_p9 = (int16_t)((c[23] << 8) | c[22]);
 
    calib.dig_h1 = c[25]; // registre 0xA1 (0xA0 n'est pas utilisé)
    calib.dig_h2 = (int16_t)((e[1] << 8) | e[0]);
    calib.dig_h3 = e[2];
    calib.dig_h4 = (int16_t)(((int8_t)e[3] * 16) | (e[4] & 0x0F));
    calib.dig_h5 = (int16_t)(((int8_t)e[5] * 16) | (e[4] >> 4));
    calib.dig_h6 = (int8_t)e[6];
 
    return true;
}
 
// COMPENSATIONS (formules Bosch, datasheet BME280 §8.1, version flottante)
 
static double compenser_temperature(int32_t adc_t, double *t_fine)
{
    double var1 = (((double)adc_t / 16384.0) - ((double)calib.dig_t1 / 1024.0)) * (double)calib.dig_t2;
    double var2 = ((double)adc_t / 131072.0) - ((double)calib.dig_t1 / 8192.0);
    var2 = (var2 * var2) * (double)calib.dig_t3;
 
    *t_fine = var1 + var2;
    return *t_fine / 5120.0;
}
 
static double compenser_pression(int32_t adc_p, double t_fine)
{
    double var1 = (t_fine / 2.0) - 64000.0;
    double var2 = var1 * var1 * ((double)calib.dig_p6 / 32768.0);
    var2 = var2 + (var1 * (double)calib.dig_p5 * 2.0);
    var2 = (var2 / 4.0) + ((double)calib.dig_p4 * 65536.0);
    var1 = (((double)calib.dig_p3 * var1 * var1 / 524288.0) + ((double)calib.dig_p2 * var1)) / 524288.0;
    var1 = (1.0 + (var1 / 32768.0)) * (double)calib.dig_p1;
 
    if (var1 == 0.0) return 0.0; // évite une division par zéro
 
    double p = 1048576.0 - (double)adc_p;
    p = (p - (var2 / 4096.0)) * 6250.0 / var1;
    var1 = (double)calib.dig_p9 * p * p / 2147483648.0;
    var2 = p * (double)calib.dig_p8 / 32768.0;
    p = p + (var1 + var2 + (double)calib.dig_p7) / 16.0;
 
    return p; // en Pa
}
 
static double compenser_humidite(int32_t adc_h, double t_fine)
{
    double h = t_fine - 76800.0;
 
    h = ((double)adc_h - (((double)calib.dig_h4 * 64.0) + ((double)calib.dig_h5 / 16384.0) * h))
        * (((double)calib.dig_h2 / 65536.0)
        * (1.0 + ((double)calib.dig_h6 / 67108864.0) * h
        * (1.0 + ((double)calib.dig_h3 / 67108864.0) * h)));
    h = h * (1.0 - ((double)calib.dig_h1 * h / 524288.0));
 
    if (h > 100.0) h = 100.0;
    if (h < 0.0)   h = 0.0;
 
    return h;
}
 
// FONCTIONS PUBLIQUES
 
bool GroveBME280_ReadChipId(uint8_t *chip_id)
{
    if (chip_id == 0) return false;
    return read_reg(BME280_REG_ID, chip_id, 1);
}
 
bool GroveBME280_Init(void)
{
    /*
     * BME280 chip ID = 0x60.
     * Adresse : 0x76 (Grove Seeed par défaut) ou 0x77.
     */
    uint8_t id = 0;
    bme280_pret = false;
 
    bme280_addr = (0x76 << 1);
    if (!(GroveBME280_ReadChipId(&id) && id == BME280_CHIP_ID))
    {
        bme280_addr = (0x77 << 1);
        if (!(GroveBME280_ReadChipId(&id) && id == BME280_CHIP_ID))
            return false;
    }
 
    // Réinitialisation logicielle puis attente de la copie de la calibration
    if (!write_reg(BME280_REG_RESET, BME280_RESET_CMD)) return false;
    HAL_Delay(5);
 
    uint8_t status = BME280_STATUS_IM_UPDATE;
    uint32_t debut = HAL_GetTick();
    while ((status & BME280_STATUS_IM_UPDATE) && (HAL_GetTick() - debut) < 50U)
    {
        if (!read_reg(BME280_REG_STATUS, &status, 1)) return false;
    }
 
    if (!lire_calibration()) return false;
 
    // Filtre IIR désactivé (config ne s'écrit qu'en mode sommeil, ce qui est le cas après le reset)
    if (!write_reg(BME280_REG_CONFIG, 0x00)) return false;
 
    bme280_pret = true;
    return true;
}
 
bool GroveBME280_ReadEnvironment(Env_Data *data)
{
    if (data == 0 || !bme280_pret) return false;
 
    // ctrl_hum n'est pris en compte qu'après l'écriture de ctrl_meas
    if (!write_reg(BME280_REG_CTRL_HUM, BME280_OSRS_H)) return false;
    if (!write_reg(BME280_REG_CTRL_MEAS,
                   (uint8_t)((BME280_OSRS_T << 5) | (BME280_OSRS_P << 2) | BME280_MODE_FORCED)))
        return false;
 
    // En mode forcé, le capteur repasse en sommeil (mode = 00) une fois la mesure terminée
    uint8_t ctrl = BME280_MODE_FORCED;
    uint32_t debut = HAL_GetTick();
 
    do
    {
        HAL_Delay(5);
        if (!read_reg(BME280_REG_CTRL_MEAS, &ctrl, 1)) return false;
 
    } while ((ctrl & 0x03) != 0 && (HAL_GetTick() - debut) < BME280_MEAS_TIMEOUT_MS);
 
    if ((ctrl & 0x03) != 0) return false; // timeout
 
    uint8_t d[8];
    if (!read_reg(BME280_REG_DATA, d, sizeof(d))) return false;
 
    int32_t adc_p = (int32_t)(((uint32_t)d[0] << 12) | ((uint32_t)d[1] << 4) | (d[2] >> 4));
    int32_t adc_t = (int32_t)(((uint32_t)d[3] << 12) | ((uint32_t)d[4] << 4) | (d[5] >> 4));
    int32_t adc_h = (int32_t)(((uint32_t)d[6] << 8) | d[7]);
 
    double t_fine;
    data->temperature_c      = (float)compenser_temperature(adc_t, &t_fine);
    data->pressure_hpa       = (float)(compenser_pression(adc_p, t_fine) / 100.0);
    data->humidity_percent   = (float)compenser_humidite(adc_h, t_fine);
    data->gas_resistance_ohm = 0.0f; // pas de capteur de gaz sur le BME280
 
    return true;
}