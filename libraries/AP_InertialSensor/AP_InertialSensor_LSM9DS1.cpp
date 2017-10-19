/*
 *  This program is free software
*/
#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include "AP_InertialSensor_LSM9DS1.h"

#include <utility>

#include <AP_HAL_Linux/GPIO.h>

extern const AP_HAL::HAL& hal;


#define WHO_AM_I     0x68

#define LSM9DS1_DRY_XG_PIN -1


/*
 *  Accelerometer and Gyroscope registers
*/
#define LSM9DS1XG_ACT_THS                               0x04 //only for debugger


#define LSM9DS1XG_WHO_AM_I                              0x0F
#define LSM9DS1XG_CTRL_REG1_G                           0x10
#   define LSM9DS1XG_CTRL_REG1_G_ODR_G_952Hz      (0x6 << 5)
#   define LSM9DS1XG_CTRL_REG1_FS_G_2000DPS       (0x3 << 3)
#define LSM9DS1XG_STATUS_REG                            0x17
#   define LSM9DS1XG_STATUS_REG_GDA               (0x1 << 1)
#   define LSM9DS1XG_STATUS_REG_XLDA              (0x1 << 0)
#define LSM9DS1XG_OUT_X_L_G                             0x18
#define LSM9DS1XG_CTRL_REG4                             0x1E
#   define LSM9DS1XG_CTRL_REG4_Zen_G              (0x1 << 5)
#   define LSM9DS1XG_CTRL_REG4_Yen_G              (0x1 << 4)
#   define LSM9DS1XG_CTRL_REG4_Xen_G              (0x1 << 3)
#define LSM9DS1XG_CTRL_REG5_XL                          0x1F
#   define LSM9DS1XG_CTRL_REG5_XL_Zen_XL          (0x1 << 5)
#   define LSM9DS1XG_CTRL_REG5_XL_Yen_XL          (0x1 << 4)
#   define LSM9DS1XG_CTRL_REG5_XL_Xen_XL          (0x1 << 3)
#define LSM9DS1XG_CTRL_REG6_XL                          0x20
#   define LSM9DS1XG_CTRL_REG6_XL_ODR_XL_952Hz    (0x6 << 5)
#   define LSM9DS1XG_CTRL_REG6_XL_FS_XL_16G       (0x1 << 3)
#define LSM9DS1XG_CTRL_REG9                             0x23
#   define LSM9DS1XG_CTRL_REG9_I2C_DISABLE        (0x1 << 2)
#define LSM9DS1XG_OUT_X_L_XL                            0x28


#define LSM9DS1XG_INT_GEN_DUR_G                         0x37 // for debug



AP_InertialSensor_LSM9DS1::AP_InertialSensor_LSM9DS1(AP_InertialSensor &imu,
                                                     AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                                     int drdy_pin_num_xg,
                                                     enum Rotation rotation)
    : AP_InertialSensor_Backend(imu)
    , _dev(std::move(dev))
    , _drdy_pin_num_xg(drdy_pin_num_xg)
    , _rotation(rotation)

{
}

AP_InertialSensor_Backend *AP_InertialSensor_LSM9DS1::probe(AP_InertialSensor &_imu,
                                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                                            enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }
    AP_InertialSensor_LSM9DS1 *sensor =
        new AP_InertialSensor_LSM9DS1(_imu,std::move(dev),
                                      LSM9DS1_DRY_XG_PIN,
                                      rotation);
    if (!sensor || !sensor->_init_sensor()) {

        delete sensor;
        return nullptr;
    }

    return sensor;
}

bool AP_InertialSensor_LSM9DS1::_init_sensor()
{
    /*
     * Same semaphore for both since they necessarily share the same bus (with
     * different CS)
     */
    _spi_sem = _dev->get_semaphore();


    if (_drdy_pin_num_xg >= 0) {
        _drdy_pin_xg = hal.gpio->channel(_drdy_pin_num_xg);
        if (_drdy_pin_xg == nullptr) {
            AP_HAL::panic("LSM9DS1: null accel data-ready GPIO channel\n");
        }

        _drdy_pin_xg->mode(HAL_GPIO_INPUT);
    }


    bool success = _hardware_init();

#if LSM9DS1_DEBUG
    _dump_registers();
#endif
    return success;
}

bool AP_InertialSensor_LSM9DS1::_hardware_init()
{
    if (!_spi_sem->take(1)) {
        return false;
    }

    uint8_t tries, whoami;

    // set flag for reading registers
    _dev->set_read_flag(0x80);


    whoami = _register_read_xg(LSM9DS1XG_WHO_AM_I);
    if (whoami != WHO_AM_I) {
        hal.console->printf("LSM9DS1: unexpected acc/gyro WHOAMI 0x%x\n", whoami);
        goto fail_whoami;
    }


    for (tries = 0; tries < 5; tries++) {
        _dev->set_speed(AP_HAL::Device::SPEED_LOW);

        _dev->write_register(LSM9DS1XG_CTRL_REG9,LSM9DS1XG_CTRL_REG9_I2C_DISABLE);

        _gyro_init();
        _accel_init();

        _dev->set_speed(AP_HAL::Device::SPEED_HIGH);

        hal.scheduler->delay(10);
        if (_accel_data_ready() && _gyro_data_ready()) {
            break;
        }

#if LSM9DS1_DEBUG
        _dump_registers();
#endif
    }
    if (tries == 5) {
        hal.console->printf("Failed to boot LSM9DS1 5 times\n\n");
        goto fail_tries;
    }

    _spi_sem->give();

    return true;

fail_tries:
fail_whoami:
    _spi_sem->give();
    return false;
}


/*
  start the sensor going
 */
void AP_InertialSensor_LSM9DS1::start(void)
{
    _gyro_instance = _imu.register_gyro(952, _dev->get_bus_id_devtype(DEVTYPE_GYR_LSM9DS1));
    _accel_instance = _imu.register_accel(952, _dev->get_bus_id_devtype(DEVTYPE_ACC_LSM9DS1));


    set_accel_orientation(_accel_instance, _rotation);
    set_gyro_orientation(_gyro_instance, _rotation);


    _set_accel_max_abs_offset(_accel_instance, 5.0f);

    /* start the timer process to read samples */
    _dev->register_periodic_callback(1000, FUNCTOR_BIND_MEMBER(&AP_InertialSensor_LSM9DS1::_poll_data, void));
}


uint8_t AP_InertialSensor_LSM9DS1::_register_read_xg(uint8_t reg)
{
    uint8_t val = 0;

    _dev->read_registers(reg, &val, 1);

    return val;
}


void AP_InertialSensor_LSM9DS1::_register_write_xg(uint8_t reg, uint8_t val, bool checked)
{
    _dev->write_register(reg, val, checked);
}


void AP_InertialSensor_LSM9DS1::_gyro_init()
{

    _register_write_xg(LSM9DS1XG_CTRL_REG1_G, LSM9DS1XG_CTRL_REG1_G_ODR_G_952Hz |
                                              LSM9DS1XG_CTRL_REG1_FS_G_2000DPS);
    hal.scheduler->delay(1);

    _register_write_xg(LSM9DS1XG_CTRL_REG4, LSM9DS1XG_CTRL_REG4_Zen_G |
                                            LSM9DS1XG_CTRL_REG4_Yen_G |
                                            LSM9DS1XG_CTRL_REG4_Xen_G);
    _set_gyro_scale(G_SCALE_2000DPS);
    hal.scheduler->delay(1);
}

void AP_InertialSensor_LSM9DS1::_accel_init()
{

    _register_write_xg(LSM9DS1XG_CTRL_REG6_XL, LSM9DS1XG_CTRL_REG6_XL_ODR_XL_952Hz |
                                               LSM9DS1XG_CTRL_REG6_XL_FS_XL_16G);
    hal.scheduler->delay(1);

    _register_write_xg(LSM9DS1XG_CTRL_REG5_XL, LSM9DS1XG_CTRL_REG5_XL_Zen_XL |
                                               LSM9DS1XG_CTRL_REG5_XL_Yen_XL |
                                               LSM9DS1XG_CTRL_REG5_XL_Xen_XL);
    _set_accel_scale(A_SCALE_16G);
    hal.scheduler->delay(1);
}

void AP_InertialSensor_LSM9DS1::_set_gyro_scale(gyro_scale scale)
{
    /* scales values from datasheet in mdps/digit */
    switch (scale) {
    case G_SCALE_245DPS:
        _gyro_scale = 8.75;
        break;
    case G_SCALE_500DPS:
        _gyro_scale = 17.50;
        break;
    case G_SCALE_2000DPS:
        _gyro_scale = 70;
        break;
    }

    /* convert mdps/digit to dps/digit */
    _gyro_scale /= 1000;
    /* convert dps/digit to (rad/s)/digit */
    _gyro_scale *= DEG_TO_RAD;
}

void AP_InertialSensor_LSM9DS1::_set_accel_scale(accel_scale scale)
{
    /*
     * Possible accelerometer scales (and their register bit settings) are:
     * 2 g (000), 4g (001), 6g (010) 8g (011), 16g (100). Here's a bit of an
     * algorithm to calculate g/(ADC tick) based on that 3-bit value:
     */
    _accel_scale = (((float) scale + 1.0f) * 2.0f) / 32768.0f;
    if (scale == A_SCALE_16G) {
        /* the datasheet shows an exception for +-16G */
        _accel_scale = 0.000732;
    }
    /* convert to G/LSB to (m/s/s)/LSB */
    _accel_scale *= GRAVITY_MSS;
}

/**
 * Timer process to poll for new data from the LSM9DS1.
 */
void AP_InertialSensor_LSM9DS1::_poll_data()
{
    if (_gyro_data_ready()) {
        _read_data_transaction_g();
    }
    if (_accel_data_ready()) {
        _read_data_transaction_x();
    }

    // check next register value for correctness
    if (!_dev->check_next_register()) {
        _inc_accel_error_count(_accel_instance);
    }
}

bool AP_InertialSensor_LSM9DS1::_accel_data_ready()
{
    if (_drdy_pin_xg != nullptr) {
        return _drdy_pin_xg->read() != 0;
    }

    uint8_t status = _register_read_xg(LSM9DS1XG_STATUS_REG);
    return status & LSM9DS1XG_STATUS_REG_XLDA;

}

bool AP_InertialSensor_LSM9DS1::_gyro_data_ready()
{
    if (_drdy_pin_xg != nullptr) {
        return _drdy_pin_xg->read() != 0;
    }

    uint8_t status = _register_read_xg(LSM9DS1XG_STATUS_REG);
    return status & LSM9DS1XG_STATUS_REG_GDA;

}

void AP_InertialSensor_LSM9DS1::_read_data_transaction_x()
{
    struct sensor_raw_data raw_data = { };
    const uint8_t reg = LSM9DS1XG_OUT_X_L_XL | 0x80;

    if (!_dev->transfer(&reg, 1, (uint8_t *) &raw_data, sizeof(raw_data))) {
        hal.console->printf("LSM9DS1: error reading accelerometer\n");
        return;
    }

    Vector3f accel_data(raw_data.x, raw_data.y, -raw_data.z);
    accel_data *= _accel_scale;

    _rotate_and_correct_accel(_accel_instance, accel_data);
    _notify_new_accel_raw_sample(_accel_instance, accel_data);
}

/*
 *  read from the data registers and update filtered data
 */
void AP_InertialSensor_LSM9DS1::_read_data_transaction_g()
{
    struct sensor_raw_data raw_data = { };
    const uint8_t reg = LSM9DS1XG_OUT_X_L_G |  0x80;

    if (!_dev->transfer(&reg, 1, (uint8_t *) &raw_data, sizeof(raw_data))) {
        hal.console->printf("LSM9DS1: error reading gyroscope\n");
        return;
    }
    Vector3f gyro_data(raw_data.x, raw_data.y, -raw_data.z);
    gyro_data *= _gyro_scale;

    _rotate_and_correct_gyro(_gyro_instance, gyro_data);
    _notify_new_gyro_raw_sample(_gyro_instance, gyro_data);
}

bool AP_InertialSensor_LSM9DS1::update()
{
    update_gyro(_gyro_instance);
    update_accel(_accel_instance);

    return true;
}

#if LSM9DS1_DEBUG
/*
 *  dump all config registers - used for debug
*/
void AP_InertialSensor_LSM9DS1::_dump_registers(void)
{
    hal.console->println("LSM9DS1 registers:");

    const uint8_t first = LSM9DS1XG_ACT_THS;
    const uint8_t last = LSM9DS1XG_INT_GEN_DUR_G;
    for (uint8_t reg=first; reg<=last; reg++) {
        uint8_t v = _register_read_xg(reg);
        hal.console->printf("%02x:%02x ", reg, v);
        if ((reg - (first-1)) % 16 == 0) {
            hal.console->println();
        }
    }
    hal.console->println();
}
#endif






#endif

