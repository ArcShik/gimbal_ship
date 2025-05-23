/**
 ******************************************************************************
 * @file    BMI088driver.c
 * @author
 * @version V1.2.0
 * @date    2022/3/8
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "BMI088driver.h"
#include "BMI088reg.h"
#include "BMI088Middleware.h"
#include "drv_dwt.h"
#include <math.h>
#include "rm_module.h"

#define SEND       //如果仅用上板发送传感器原始数据而不在上板进行欧拉角解算时使用，现在已经将发送注释部分注释掉
//#define RECEIVE    //启动下板接收传感器原始数据

float BMI088_ACCEL_SEN = BMI088_ACCEL_6G_SEN;
float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;

static uint8_t res = 0;
static uint8_t write_reg_num = 0;
static uint8_t error = BMI088_NO_ERROR;
float gyroDiff[3], gNormDiff;

uint8_t caliOffset = 1;
int16_t caliCount = 0;

ImuDataTypeDef BMI088;

extern CAN_HandleTypeDef hcan2;

#if defined(BMI088_USE_SPI)

#define BMI088_accel_write_single_reg(reg, data) \
    {                                            \
        BMI088_ACCEL_NS_L();                     \
        BMI088_write_single_reg((reg), (data));  \
        BMI088_ACCEL_NS_H();                     \
    }
#define BMI088_accel_read_single_reg(reg, data) \
    {                                           \
        BMI088_ACCEL_NS_L();                    \
        BMI088_read_write_byte((reg) | 0x80);   \
        BMI088_read_write_byte(0x55);           \
        (data) = BMI088_read_write_byte(0x55);  \
        BMI088_ACCEL_NS_H();                    \
    }
//#define BMI088_accel_write_muli_reg( reg,  data, len) { BMI088_ACCEL_NS_L(); BMI088_write_muli_reg(reg, data, len); BMI088_ACCEL_NS_H(); }
#define BMI088_accel_read_muli_reg(reg, data, len) \
    {                                              \
        BMI088_ACCEL_NS_L();                       \
        BMI088_read_write_byte((reg) | 0x80);      \
        BMI088_read_muli_reg(reg, data, len);      \
        BMI088_ACCEL_NS_H();                       \
    }

#define BMI088_gyro_write_single_reg(reg, data) \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_write_single_reg((reg), (data)); \
        BMI088_GYRO_NS_H();                     \
    }
#define BMI088_gyro_read_single_reg(reg, data)  \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_read_single_reg((reg), &(data)); \
        BMI088_GYRO_NS_H();                     \
    }
//#define BMI088_gyro_write_muli_reg( reg,  data, len) { BMI088_GYRO_NS_L(); BMI088_write_muli_reg( ( reg ), ( data ), ( len ) ); BMI088_GYRO_NS_H(); }
#define BMI088_gyro_read_muli_reg(reg, data, len)   \
    {                                               \
        BMI088_GYRO_NS_L();                         \
        BMI088_read_muli_reg((reg), (data), (len)); \
        BMI088_GYRO_NS_H();                         \
    }

static void
BMI088_write_single_reg(uint8_t reg, uint8_t data);
static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data);
// static void BMI088_write_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len );
static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);

#elif defined(BMI088_USE_IIC)

#endif

static uint8_t write_BMI088_accel_reg_data_error[BMI088_WRITE_ACCEL_REG_NUM][3] =
    {
        {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
        {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
        {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
        {BMI088_ACC_RANGE, BMI088_ACC_RANGE_6G, BMI088_ACC_RANGE_ERROR},
        {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR},
        {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}

};

static uint8_t write_BMI088_gyro_reg_data_error[BMI088_WRITE_GYRO_REG_NUM][3] =
    {
        {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
        {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_2000_230_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
        {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
        {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}

};

static void Calibrate_MPU_Offset(ImuDataTypeDef *bmi088);

//void BMI088_Init(SPI_HandleTypeDef *bmi088_SPI)
//{
//    while (BMI088_init(bmi088_SPI))
//    {
//        ;
//    }
//}

uint8_t BMI088_init(SPI_HandleTypeDef *bmi088_SPI)
{
    BMI088_SPI = bmi088_SPI;
    error = BMI088_NO_ERROR;

    error |= bmi088_accel_init();
    error |= bmi088_gyro_init();

    Calibrate_MPU_Offset(&BMI088);

    return error;
}

// 较准零飘
void Calibrate_MPU_Offset(ImuDataTypeDef *bmi088)
{
    static float startTime;
    static uint16_t CaliTimes = 5000;   // 需要足够多的数据才能得到有效陀螺仪零偏校准结果
    uint8_t buf[8] = {0, 0, 0, 0, 0, 0};
    int16_t bmi088_raw_temp;
    float gyroMax[3], gyroMin[3];
    float gNormTemp, gNormMax, gNormMin;
#ifndef BSP_BMI088_CALI  // 更换开发板或定期校准，以便快速启动
    static uint8_t cali_dt_max = 0;
#elif
    static uint8_t cali_dt_max = 10;
#endif /* BSP_BMI088_CALI */

    startTime = dwt_get_time_s();
    do
    {
        if (dwt_get_time_s() - startTime > cali_dt_max)
        {
            // 校准超时
            bmi088->gyro_offset[0] = GxOFFSET;
            bmi088->gyro_offset[1] = GyOFFSET;
            bmi088->gyro_offset[2] = GzOFFSET;
            bmi088->g_norm = gNORM;
            bmi088->temp_when_cali = 40;
            break;
        }

        dwt_delay_s(0.005);
        bmi088->g_norm = 0;
        bmi088->gyro_offset[0] = 0;
        bmi088->gyro_offset[1] = 0;
        bmi088->gyro_offset[2] = 0;

        for (uint16_t i = 0; i < CaliTimes; i++)
        {
            BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);
            bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
            bmi088->accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN;
            bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
            bmi088->accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN;
            bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
            bmi088->accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN;
            gNormTemp = sqrtf(bmi088->accel[0] * bmi088->accel[0] +
                              bmi088->accel[1] * bmi088->accel[1] +
                              bmi088->accel[2] * bmi088->accel[2]);
            bmi088->g_norm += gNormTemp;

            BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
            if (buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
            {
                bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
                bmi088->gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
                bmi088->gyro_offset[0] += bmi088->gyro[0];
                bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
                bmi088->gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
                bmi088->gyro_offset[1] += bmi088->gyro[1];
                bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
                bmi088->gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
                bmi088->gyro_offset[2] += bmi088->gyro[2];
            }

            // 记录数据极差
            if (i == 0)
            {
                gNormMax = gNormTemp;
                gNormMin = gNormTemp;
                for (uint8_t j = 0; j < 3; j++)
                {
                    gyroMax[j] = bmi088->gyro[j];
                    gyroMin[j] = bmi088->gyro[j];
                }
            }
            else
            {
                if (gNormTemp > gNormMax)
                    gNormMax = gNormTemp;
                if (gNormTemp < gNormMin)
                    gNormMin = gNormTemp;
                for (uint8_t j = 0; j < 3; j++)
                {
                    if (bmi088->gyro[j] > gyroMax[j])
                        gyroMax[j] = bmi088->gyro[j];
                    if (bmi088->gyro[j] < gyroMin[j])
                        gyroMin[j] = bmi088->gyro[j];
                }
            }

            // 数据差异过大认为收到外界干扰，需重新校准
            gNormDiff = gNormMax - gNormMin;
            for (uint8_t j = 0; j < 3; j++)
                gyroDiff[j] = gyroMax[j] - gyroMin[j];
            if (gNormDiff > 0.3f ||
                gyroDiff[0] > 0.1f ||
                gyroDiff[1] > 0.1f ||
                gyroDiff[2] > 0.1f)
                break;
            dwt_delay_s(0.0005);
        }

        // 取平均值得到标定结果
        bmi088->g_norm /= (float)CaliTimes;
        for (uint8_t i = 0; i < 3; i++)
            bmi088->gyro_offset[i] /= (float)CaliTimes;

        // 记录标定时IMU温度
        BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);
        bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));
        if (bmi088_raw_temp > 1023)
            bmi088_raw_temp -= 2048;
        bmi088->temp_when_cali = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;

        caliCount++;
    } while (gNormDiff > 0.3f ||
             fabsf(bmi088->g_norm - 9.8f) > 0.5f ||
             gyroDiff[0] > 0.1f ||
             gyroDiff[1] > 0.1f ||
             gyroDiff[2] > 0.1f ||
             fabsf(bmi088->gyro_offset[0]) > 0.01f ||
             fabsf(bmi088->gyro_offset[1]) > 0.01f ||
             fabsf(bmi088->gyro_offset[2]) > 0.01f);

    // 根据标定结果校准加速度计标度因数
    bmi088->accel_scale = 9.81f / bmi088->g_norm;
}

uint8_t bmi088_accel_init(void)
{
    // check commiunication
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    HAL_Delay(1);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    HAL_Delay(1);

    // accel software reset
    BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    HAL_Delay(BMI088_LONG_DELAY_TIME);

    // check commiunication is normal after reset
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    HAL_Delay(1);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    HAL_Delay(1);

    // check the "who am I"
    if (res != BMI088_ACC_CHIP_ID_VALUE)
        return BMI088_NO_SENSOR;
    // do
    // {
    //     BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    // } while (res != BMI088_ACC_CHIP_ID_VALUE);

    // set accel sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++)
    {

        BMI088_accel_write_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], write_BMI088_accel_reg_data_error[write_reg_num][1]);
        HAL_Delay(1);

        BMI088_accel_read_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], res);
        HAL_Delay(1);

        if (res != write_BMI088_accel_reg_data_error[write_reg_num][1])
        {
//             write_reg_num--;
            // return write_BMI088_accel_reg_data_error[write_reg_num][2];
            error |= write_BMI088_accel_reg_data_error[write_reg_num][2];
        }
    }
    return BMI088_NO_ERROR;
}

uint8_t bmi088_gyro_init(void)
{
    // check commiunication
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    HAL_Delay(1);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    HAL_Delay(1);

    // reset the gyro sensor
    BMI088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    HAL_Delay(BMI088_LONG_DELAY_TIME);
    // check commiunication is normal after reset
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    HAL_Delay(1);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    HAL_Delay(1);

    // check the "who am I"
    if (res != BMI088_GYRO_CHIP_ID_VALUE)
        return BMI088_NO_SENSOR;

    // set gyro sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++)
    {

        BMI088_gyro_write_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], write_BMI088_gyro_reg_data_error[write_reg_num][1]);
        HAL_Delay(1);

        BMI088_gyro_read_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], res);
        HAL_Delay(1);

        if (res != write_BMI088_gyro_reg_data_error[write_reg_num][1])
        {
            write_reg_num--;
            // return write_BMI088_gyro_reg_data_error[write_reg_num][2];
            error |= write_BMI088_accel_reg_data_error[write_reg_num][2];
        }
    }

    return BMI088_NO_ERROR;
}


void BMI088_Read(ImuDataTypeDef *bmi088)
{
    static uint8_t buf_accel[8] = {0, 0, 0, 0, 0, 0};
    static uint8_t buf_gyro[8] = {0, 0, 0, 0, 0, 0};
    static uint8_t buf_temperature[8]={0,};
    static int16_t bmi088_raw_temp;


    BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf_accel, 6);

    bmi088_raw_temp = (int16_t)((buf_accel[1]) << 8) | buf_accel[0];
    bmi088->accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN * bmi088->accel_scale;
    bmi088_raw_temp = (int16_t)((buf_accel[3]) << 8) | buf_accel[2];
    bmi088->accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN * bmi088->accel_scale;
    bmi088_raw_temp = (int16_t)((buf_accel[5]) << 8) | buf_accel[4];
    bmi088->accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN * bmi088->accel_scale;

    BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf_gyro, 8);

    if (buf_gyro[0] == BMI088_GYRO_CHIP_ID_VALUE)
    {
        if (caliOffset)
        {
            bmi088_raw_temp = (int16_t)((buf_gyro[3]) << 8) | buf_gyro[2];
            bmi088->gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN - bmi088->gyro_offset[0];
            bmi088_raw_temp = (int16_t)((buf_gyro[5]) << 8) | buf_gyro[4];
            bmi088->gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN - bmi088->gyro_offset[1];
            bmi088_raw_temp = (int16_t)((buf_gyro[7]) << 8) | buf_gyro[6];
            bmi088->gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN - bmi088->gyro_offset[2];
        }
        else
        {
            bmi088_raw_temp = (int16_t)((buf_gyro[3]) << 8) | buf_gyro[2];
            bmi088->gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
            bmi088_raw_temp = (int16_t)((buf_gyro[5]) << 8) | buf_gyro[4];
            bmi088->gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
            bmi088_raw_temp = (int16_t)((buf_gyro[7]) << 8) | buf_gyro[6];
            bmi088->gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
        }
    }

    BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf_temperature, 2);

    bmi088_raw_temp = (int16_t)((buf_temperature[0] << 3) | (buf_temperature[1] >> 5));

    if (bmi088_raw_temp > 1023)
    {
        bmi088_raw_temp -= 2048;
    }

    bmi088->temperature = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

#if defined(BMI088_USE_SPI)

static void BMI088_write_single_reg(uint8_t reg, uint8_t data)
{
    BMI088_read_write_byte(reg);
    BMI088_read_write_byte(data);
}

static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data)
{
    BMI088_read_write_byte(reg | 0x80);
    *return_data = BMI088_read_write_byte(0x55);
}

// static void BMI088_write_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len )
//{
//     BMI088_read_write_byte( reg );
//     while( len != 0 )
//     {

//        BMI088_read_write_byte( *buf );
//        buf ++;
//        len --;
//    }

//}

static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    BMI088_read_write_byte(reg | 0x80);

    while (len != 0)
    {

        *buf = BMI088_read_write_byte(0x55);
        buf++;
        len--;
    }
}
#elif defined(BMI088_USE_IIC)

#endif
