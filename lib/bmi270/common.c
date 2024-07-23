/**
 * Copyright (C) 2023 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "common.h"
//#include "bmi2_defs.h"

//#include "driver/i2c.h"
//#include <driver/spi_master.h>
//#include "driver/gpio.h"
//#include "sdkconfig.h"

/******************************************************************************/
/*!                 Macro definitions                                         */
#define BMI2XY_SHUTTLE_ID  UINT16_C(0x1B8)

/*! Macro that defines read write length */
#define READ_WRITE_LEN     UINT8_C(46)

/******************************************************************************/
/*!                Static variable definition                                 */

/*! Variable that holds the I2C device address or SPI chip selection */
static uint8_t dev_addr;

/*! Variable that holds the I2C or SPI bus instance */
static uint8_t bus_inst;

/*! Structure to hold interface configurations */
static struct coines_intf_config intf_conf;

struct bmi2_dev Bmi270;
struct bmi2_dev *pBmi270=&Bmi270;

i2c_cmd_handle_t i2chandle;

// SPIデバイスハンドラーを使って通信する
spi_device_handle_t spidev;



i2c_port_t i2c_port=1;

uint8_t Bmi270_address = 0x69; 

uint8_t _I2CBuffer[256];

struct bmi2_sens_config config;

void bmi270_dev_init(void)
{
  Bmi270.intf = BMI2_SPI_INTF;
  //Bmi270.chip_id = 0x24;
  Bmi270.read = bmi2_spi_read;
  Bmi270.write =bmi2_spi_write;
  Bmi270.delay_us = bmi2_delay_us;
  Bmi270.dummy_byte = 1;
  Bmi270.gyro_en = 1;
}

void getI2cBus(void)
{
    i2chandle = i2c_cmd_link_create();
}

void putI2cBus(void)
{
    i2c_master_stop(i2chandle);
    i2c_master_cmd_begin(i2c_port, i2chandle, 1 / portTICK_RATE_MS);
    i2c_cmd_link_delete(i2chandle);
}

int _i2cWrite(uint8_t slave_address, uint8_t *pdata, uint32_t count) {
    int status;
    
    i2c_master_start(i2chandle);
    status = i2c_master_write_byte(i2chandle, (slave_address<<1)|I2C_MASTER_WRITE, I2C_MASTER_ACK);
    status = i2c_master_write(i2chandle, pdata, count, I2C_MASTER_ACK);
    return status;
}

int _i2cRead(uint8_t slave_address, uint8_t *pdata, uint32_t count) {
    int status;

    i2c_master_start(i2chandle);
    status = i2c_master_write_byte(i2chandle, (slave_address<<1)|I2C_MASTER_READ, I2C_MASTER_ACK);
    if (count>1)
    {
        status = i2c_master_read(i2chandle, pdata, count-1, I2C_MASTER_ACK);
    }
    status = i2c_master_read_byte(i2chandle, pdata+count-1, I2C_MASTER_NACK);
    return status;
}

/******************************************************************************/
/*!                User interface functions                                   */

/*!
 * I2C read function (map to StampFly)
 */
BMI2_INTF_RETURN_TYPE bmi2_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    int8_t Status = 0;
    int32_t status_int;

    _I2CBuffer[0] = reg_addr;
    getI2cBus();
    status_int = _i2cWrite(Bmi270_address, _I2CBuffer, 1);
    if (status_int != 0) {
        Status = 0x55;
        goto done;
    }
    status_int = _i2cRead(Bmi270_address, reg_data, len);
    if (status_int != 0) {
        Status = 0x66;
    }
done:
    putI2cBus();
    return Status;
}

/*!
 * I2C write function map to StampFly
 */
BMI2_INTF_RETURN_TYPE bmi2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t status_int;
    uint8_t Status = 0;
    if (len > sizeof(_I2CBuffer) - 1) {
        return -1;
    }
    _I2CBuffer[0] = reg_addr;
    memcpy(&_I2CBuffer[1], reg_data, len);
    getI2cBus();
    status_int = _i2cWrite(Bmi270_address, _I2CBuffer, len + 1);
    if (status_int != 0) {
        Status = 0x55;
    }
    putI2cBus();
    return Status;
}

//SPIバスの設定
spi_bus_config_t buscfg = {
    .mosi_io_num = PIN_NUM_MOSI,
    .miso_io_num = PIN_NUM_MISO,
    .sclk_io_num = PIN_NUM_CLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 4096*2,
};

// SPIデバイスの設定
spi_device_interface_config_t devcfg = {
    .command_bits = 0,
    .address_bits = 0,
    .dummy_bits = 0,
    .mode = 0,
    .duty_cycle_pos = 128,  // default 128 = 50%/50% duty
    .cs_ena_pretrans = 0, // 0 not used
    .cs_ena_posttrans = 0,  // 0 not used
    .clock_speed_hz = SPI_MASTER_FREQ_8M,// 8,9,10,11,13,16,20,26,40,80
    .spics_io_num = 46,
    .flags = 0,  // 0 not used
    .queue_size = 10,// transactionのキュー数。1以上の値を入れておく。
    .pre_cb = NULL,// transactionが始まる前に呼ばれる関数をセットできる
    .post_cb = NULL,// transactionが完了した後に呼ばれる関数をセットできる
};


esp_err_t spi_init(void)
{


    //Initialize the SPI bus
    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if(ret != ESP_OK) return ret;

    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spidev);
    return ret;
}


/*!
 * SPI read function
 */
BMI2_INTF_RETURN_TYPE bmi2_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    // 読み込み
    spi_transaction_t trans;
    esp_err_t ret=0;

    memset(&trans, 0, sizeof(trans)); // 構造体をゼロで初期化
    
    _I2CBuffer[0]=reg_addr|0x80;
    //3Byte書き込み、読み込み
    //trans.flags = SPI_TRANS_CS_KEEP_ACTIVE|SPI_TRANS_USE_TXDATA;
    //trans.flags = SPI_TRANS_USE_TXDATA;
    trans.tx_buffer =_I2CBuffer;
    trans.rx_buffer =reg_data;
    trans.length = 8+len*8;
    ret=spi_device_polling_transmit(spidev, &trans);
    uint16_t index = 0;
    while(index<len)
    {
        reg_data[index]=reg_data[index+1];
        index++;
    }
    assert(ret==ESP_OK);
    return ret;
}

/*!
 * SPI write function map to COINES platform
 */
BMI2_INTF_RETURN_TYPE bmi2_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    spi_transaction_t trans;
    esp_err_t ret;
    //uint8_t buffer[3];
    uint8_t tmp;

    _I2CBuffer[0] = reg_addr&0b01111111;
    memcpy(&_I2CBuffer[1], reg_data, len);
    memset(&trans, 0, sizeof(trans)); // 構造体をゼロで初期化
    
    //buffer[0]=0x7C;
    //buffer[1]=0;
    //buffer[2]=4;

    // アドレス+データの書き込み
    trans.tx_buffer = _I2CBuffer;
    trans.rx_buffer = NULL;
    trans.length = 8+len*8;
    trans.rxlength = 0;

    //書き込み
    ret = spi_device_polling_transmit(spidev, &trans);
    assert(ret==ESP_OK);
    
    //spi_device_release_bus(spidev);

    return ret;
}

/*!
 * Delay function map to COINES platform
 */
void bmi2_delay_us(uint32_t period, void *intf_ptr)
{
    //coines_delay_usec(period);
    ets_delay_us(period);
}

/*!
 *  @brief Function to initialize coines platform
 */
int16_t board_init(void)
{
    #if 0
    struct coines_board_info board_info;

    int16_t result = coines_open_comm_intf(intf_type, NULL);

    if (result < COINES_SUCCESS)
    {
        printf(
            "\n Unable to connect with Application Board ! \n" " 1. Check if the board is connected and powered on. \n" " 2. Check if Application Board USB driver is installed. \n"
            " 3. Check if board is in use by another application. (Insufficient permissions to access USB) \n");
        exit(result);
    }

    if (get_board_info)
    {

        result = coines_get_board_info(&board_info);

#if defined(PC)
        setbuf(stdout, NULL);
#endif

        if (result == COINES_SUCCESS)
        {
            if ((board_info.shuttle_id != BMI2XY_SHUTTLE_ID))
            {
                printf("! Warning invalid sensor shuttle \n ," "This application will not support this sensor \n");
            }
        }
    }

    coines_delay_msec(100);

    /* Power up the board */
    coines_set_shuttleboard_vdd_vddio_config(3300, 3300);

    coines_delay_msec(200);

    return result;
    #endif
    return 0;
}

/*!
 *  @brief Function to select the interface between SPI and I2C.
 */
int8_t bmi2_interface_init(struct bmi2_dev *bmi, uint8_t intf)
{

    int8_t rslt = BMI2_OK;
#if 0
    if (bmi != NULL)
    {
        #if 0
        int16_t result = coines_board_init(COINES_COMM_INTF_USB, true);

        if (result != COINES_SUCCESS)
        {
            printf("\n Unable to open device ! \n");

            return COINES_E_UNABLE_OPEN_DEVICE;
        }
        #endif
        /* Bus configuration : I2C */
        if (intf == BMI2_I2C_INTF)
        {
            printf("I2C Interface \n");

            /* To initialize the user I2C function */
            dev_addr = BMI2_I2C_PRIM_ADDR;
            bmi->intf = BMI2_I2C_INTF;
            bmi->read = bmi2_i2c_read;
            bmi->write = bmi2_i2c_write;
            #if 0
            /* SDO to Ground */
            coines_set_pin_config(COINES_SHUTTLE_PIN_22, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW);

            /* Make CSB pin HIGH */
            coines_set_pin_config(COINES_SHUTTLE_PIN_21, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_HIGH);
            coines_delay_msec(100);

            /* SDO pin is made low */
            coines_set_pin_config(COINES_SHUTTLE_PIN_SDO, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW);

            result = coines_config_i2c_bus(COINES_I2C_BUS_0, COINES_I2C_STANDARD_MODE);
            #endif
            bus_inst = COINES_I2C_BUS_0;
        }
        /* Bus configuration : SPI */
        else if (0/*intf == BMI2_SPI_INTF*/)
        {
            #if 0
            printf("SPI Interface \n");

            /* To initialize the user SPI function */
            dev_addr = COINES_MINI_SHUTTLE_PIN_2_1;
            bmi->intf = BMI2_SPI_INTF;
            bmi->read = bmi2_spi_read;
            bmi->write = bmi2_spi_write;

            result = coines_config_spi_bus(COINES_SPI_BUS_0, COINES_SPI_SPEED_5_MHZ, COINES_SPI_MODE0);

            coines_set_pin_config(COINES_SHUTTLE_PIN_21, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_HIGH);

            bus_inst = COINES_SPI_BUS_0;
            #endif
        }

        if (COINES_SUCCESS == result)
        {
            /* Assign device address and bus instance to interface pointer */
            intf_conf.bus = bus_inst;
            intf_conf.dev_addr = dev_addr;
            bmi->intf_ptr = ((void *)&intf_conf);

            /* Configure delay in microseconds */
            bmi->delay_us = bmi2_delay_us;

            /* Configure max read/write length (in bytes) ( Supported length depends on target machine) */
            bmi->read_write_len = READ_WRITE_LEN;

            /* Assign to NULL to load the default config file. */
            bmi->config_file_ptr = NULL;
        }
        else
        {
            rslt = COINES_E_COMM_INIT_FAILED;
        }
    }
    else
    {
        rslt = BMI2_E_NULL_PTR;
    }

#endif
    return rslt;
}

/*!
 *  @brief Prints the execution status of the APIs.
 */
void bmi2_error_codes_print_result(int8_t rslt)
{
    switch (rslt)
    {
        case BMI2_OK:

            /* Do nothing */
            break;

        case BMI2_W_FIFO_EMPTY:
            printf("Warning [%d] : FIFO empty\r\n", rslt);
            break;
        case BMI2_W_PARTIAL_READ:
            printf("Warning [%d] : FIFO partial read\r\n", rslt);
            break;
        case BMI2_E_NULL_PTR:
            printf(
                "Error [%d] : Null pointer error. It occurs when the user tries to assign value (not address) to a pointer," " which has been initialized to NULL.\r\n",
                rslt);
            break;

        case BMI2_E_COM_FAIL:
            printf(
                "Error [%d] : Communication failure error. It occurs due to read/write operation failure and also due " "to power failure during communication\r\n",
                rslt);
            break;

        case BMI2_E_DEV_NOT_FOUND:
            printf("Error [%d] : Device not found error. It occurs when the device chip id is incorrectly read\r\n",
                   rslt);
            break;

        case BMI2_E_INVALID_SENSOR:
            printf(
                "Error [%d] : Invalid sensor error. It occurs when there is a mismatch in the requested feature with the " "available one\r\n",
                rslt);
            break;

        case BMI2_E_SELF_TEST_FAIL:
            printf(
                "Error [%d] : Self-test failed error. It occurs when the validation of accel self-test data is " "not satisfied\r\n",
                rslt);
            break;

        case BMI2_E_INVALID_INT_PIN:
            printf(
                "Error [%d] : Invalid interrupt pin error. It occurs when the user tries to configure interrupt pins " "apart from INT1 and INT2\r\n",
                rslt);
            break;

        case BMI2_E_OUT_OF_RANGE:
            printf(
                "Error [%d] : Out of range error. It occurs when the data exceeds from filtered or unfiltered data from " "fifo and also when the range exceeds the maximum range for accel and gyro while performing FOC\r\n",
                rslt);
            break;

        case BMI2_E_ACC_INVALID_CFG:
            printf(
                "Error [%d] : Invalid Accel configuration error. It occurs when there is an error in accel configuration" " register which could be one among range, BW or filter performance in reg address 0x40\r\n",
                rslt);
            break;

        case BMI2_E_GYRO_INVALID_CFG:
            printf(
                "Error [%d] : Invalid Gyro configuration error. It occurs when there is a error in gyro configuration" "register which could be one among range, BW or filter performance in reg address 0x42\r\n",
                rslt);
            break;

        case BMI2_E_ACC_GYR_INVALID_CFG:
            printf(
                "Error [%d] : Invalid Accel-Gyro configuration error. It occurs when there is a error in accel and gyro" " configuration registers which could be one among range, BW or filter performance in reg address 0x40 " "and 0x42\r\n",
                rslt);
            break;

        case BMI2_E_CONFIG_LOAD:
            printf(
                "Error [%d] : Configuration load error. It occurs when failure observed while loading the configuration " "into the sensor\r\n",
                rslt);
            break;

        case BMI2_E_INVALID_PAGE:
            printf(
                "Error [%d] : Invalid page error. It occurs due to failure in writing the correct feature configuration " "from selected page\r\n",
                rslt);
            break;

        case BMI2_E_SET_APS_FAIL:
            printf(
                "Error [%d] : APS failure error. It occurs due to failure in write of advance power mode configuration " "register\r\n",
                rslt);
            break;

        case BMI2_E_AUX_INVALID_CFG:
            printf(
                "Error [%d] : Invalid AUX configuration error. It occurs when the auxiliary interface settings are not " "enabled properly\r\n",
                rslt);
            break;

        case BMI2_E_AUX_BUSY:
            printf(
                "Error [%d] : AUX busy error. It occurs when the auxiliary interface buses are engaged while configuring" " the AUX\r\n",
                rslt);
            break;

        case BMI2_E_REMAP_ERROR:
            printf(
                "Error [%d] : Remap error. It occurs due to failure in assigning the remap axes data for all the axes " "after change in axis position\r\n",
                rslt);
            break;

        case BMI2_E_GYR_USER_GAIN_UPD_FAIL:
            printf(
                "Error [%d] : Gyro user gain update fail error. It occurs when the reading of user gain update status " "fails\r\n",
                rslt);
            break;

        case BMI2_E_SELF_TEST_NOT_DONE:
            printf(
                "Error [%d] : Self-test not done error. It occurs when the self-test process is ongoing or not " "completed\r\n",
                rslt);
            break;

        case BMI2_E_INVALID_INPUT:
            printf("Error [%d] : Invalid input error. It occurs when the sensor input validity fails\r\n", rslt);
            break;

        case BMI2_E_INVALID_STATUS:
            printf("Error [%d] : Invalid status error. It occurs when the feature/sensor validity fails\r\n", rslt);
            break;

        case BMI2_E_CRT_ERROR:
            printf("Error [%d] : CRT error. It occurs when the CRT test has failed\r\n", rslt);
            break;

        case BMI2_E_ST_ALREADY_RUNNING:
            printf(
                "Error [%d] : Self-test already running error. It occurs when the self-test is already running and " "another has been initiated\r\n",
                rslt);
            break;

        case BMI2_E_CRT_READY_FOR_DL_FAIL_ABORT:
            printf(
                "Error [%d] : CRT ready for download fail abort error. It occurs when download in CRT fails due to wrong " "address location\r\n",
                rslt);
            break;

        case BMI2_E_DL_ERROR:
            printf(
                "Error [%d] : Download error. It occurs when write length exceeds that of the maximum burst length\r\n",
                rslt);
            break;

        case BMI2_E_PRECON_ERROR:
            printf(
                "Error [%d] : Pre-conditional error. It occurs when precondition to start the feature was not " "completed\r\n",
                rslt);
            break;

        case BMI2_E_ABORT_ERROR:
            printf("Error [%d] : Abort error. It occurs when the device was shaken during CRT test\r\n", rslt);
            break;

        case BMI2_E_WRITE_CYCLE_ONGOING:
            printf(
                "Error [%d] : Write cycle ongoing error. It occurs when the write cycle is already running and another " "has been initiated\r\n",
                rslt);
            break;

        case BMI2_E_ST_NOT_RUNING:
            printf(
                "Error [%d] : Self-test is not running error. It occurs when self-test running is disabled while it's " "running\r\n",
                rslt);
            break;

        case BMI2_E_DATA_RDY_INT_FAILED:
            printf(
                "Error [%d] : Data ready interrupt error. It occurs when the sample count exceeds the FOC sample limit " "and data ready status is not updated\r\n",
                rslt);
            break;

        case BMI2_E_INVALID_FOC_POSITION:
            printf(
                "Error [%d] : Invalid FOC position error. It occurs when average FOC data is obtained for the wrong" " axes\r\n",
                rslt);
            break;

        default:
            printf("Error [%d] : Unknown error code\r\n", rslt);
            break;
    }
}

/*!
 *  @brief Deinitializes coines platform
 *
 *  @return void.
 */
void bmi2_coines_deinit(void)
{
    #if 0
    fflush(stdout);

    coines_set_shuttleboard_vdd_vddio_config(0, 0);
    coines_delay_msec(100);

    /* Coines interface reset */
    coines_soft_reset();
    coines_delay_msec(100);

    coines_close_comm_intf(COINES_COMM_INTF_USB, NULL);
    #endif
}


/*!
 * @brief This internal API is used to set configurations for accel and gyro.
 */
int8_t set_accel_gyro_config(struct bmi2_dev *bmi)
{
    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Structure to define accelerometer and gyro configuration. */
    struct bmi2_sens_config config[2];

    /* Configure the type of feature. */
    config[ACCEL].type = BMI2_ACCEL;
    config[GYRO].type = BMI2_GYRO;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi2_get_sensor_config(config, 2, bmi);
    bmi2_error_codes_print_result(rslt);

    /* Map data ready interrupt to interrupt pin. */
    rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, bmi);
    bmi2_error_codes_print_result(rslt);

    if (rslt == BMI2_OK)
    {
        /* NOTE: The user can change the following configuration parameters according to their requirement. */
        /* Set Output Data Rate */
        config[ACCEL].cfg.acc.odr = BMI2_ACC_ODR_400HZ;

        /* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G). */
        config[ACCEL].cfg.acc.range = BMI2_ACC_RANGE_8G;

        /* The bandwidth parameter is used to configure the number of sensor samples that are averaged
         * if it is set to 2, then 2^(bandwidth parameter) samples
         * are averaged, resulting in 4 averaged samples.
         * Note1 : For more information, refer the datasheet.
         * Note2 : A higher number of averaged samples will result in a lower noise level of the signal, but
         * this has an adverse effect on the power consumed.
         */
        config[ACCEL].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;

        /* Enable the filter performance mode where averaging of samples
         * will be done based on above set bandwidth and ODR.
         * There are two modes
         *  0 -> Ultra low power mode
         *  1 -> High performance mode(Default)
         * For more info refer datasheet.
         */
        config[ACCEL].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

        /* The user can change the following configuration parameters according to their requirement. */
        /* Set Output Data Rate */
        config[GYRO].cfg.gyr.odr = BMI2_GYR_ODR_400HZ;

        /* Gyroscope Angular Rate Measurement Range.By default the range is 2000dps. */
        config[GYRO].cfg.gyr.range = BMI2_GYR_RANGE_2000;//BMI2_GYR_RANGE_2000

        /* Gyroscope bandwidth parameters. By default the gyro bandwidth is in normal mode. */
        config[GYRO].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;// BMI2_GYR_OSR4_MODE;

        /* Enable/Disable the noise performance mode for precision yaw rate sensing
         * There are two modes
         *  0 -> Ultra low power mode(Default)
         *  1 -> High performance mode
         */
        config[GYRO].cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;

        /* Enable/Disable the filter performance mode where averaging of samples
         * will be done based on above set bandwidth and ODR.
         * There are two modes
         *  0 -> Ultra low power mode
         *  1 -> High performance mode(Default)
         */
        config[GYRO].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

        /* Set the accel and gyro configurations. */
        rslt = bmi2_set_sensor_config(config, 2, bmi);
        bmi2_error_codes_print_result(rslt);
    }

    return rslt;
}

/*!
 * @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 * range 2G, 4G, 8G or 16G.
 */
float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (GRAVITY_EARTH * val * g_range) / half_scale;
}

/*!
 * @brief This function converts lsb to degree per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 */
float lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (dps / (half_scale)) * (val);
}

/*!
 * @brief This function converts lsb to degree per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 * range 2.181661565, 4.36332313, 8.72664626, 17.45329252, 34.90658504
 */
float lsb_to_rps(int16_t val, float rps, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (rps / (half_scale)) * (val);
}