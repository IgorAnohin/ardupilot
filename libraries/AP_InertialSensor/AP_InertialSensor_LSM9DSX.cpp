
#include <AP_HAL/AP_HAL.h>

#include "AP_InertialSensor_LSM9DSX.h"

#include <utility>

#include <AP_HAL_Linux/GPIO.h>

extern const AP_HAL::HAL &hal;


#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RASPILOT
#define LSM9DS0_DRY_X_PIN RPI_GPIO_17
#define LSM9DS0_DRY_G_PIN RPI_GPIO_6
#else
#define LSM9DS0_DRY_X_PIN -1
#define LSM9DS0_DRY_G_PIN -1
#endif

#define LSM9DS0_G_WHOAMI    0xD4 // L3GD20
#define LSM9DS0_G_WHOAMI_H  0xD7 // L3GD20H
#define LSM9DS0_XM_WHOAMI   0x49

////////////////////////////
// LSM9DS0 Gyro Registers //
////////////////////////////
#define WHO_AM_I_G                                    0x0F
#define CTRL_REG1_G                                   0x20
#   define CTRL_REG1_G_DR_760Hz_BW_50Hz         (0xE << 4)
#   define CTRL_REG1_G_PD                       (0x1 << 3)
#   define CTRL_REG1_G_ZEN                      (0x1 << 2)
#   define CTRL_REG1_G_YEN                      (0x1 << 1)
#   define CTRL_REG1_G_XEN                      (0x1 << 0)
#define CTRL_REG2_G                                   0x21
#define CTRL_REG3_G                                   0x22
#   define CTRL_REG3_G_I2_DRDY                  (0x1 << 3)
#define CTRL_REG4_G                                   0x23
#   define CTRL_REG4_G_BDU                      (0x1 << 7)
#   define CTRL_REG4_G_FS_2000DPS               (0x2 << 4)
#define CTRL_REG5_G                                   0x24
#define STATUS_REG_G                                  0x27
#   define STATUS_REG_G_ZYXDA                   (0x1 << 3)
#define OUT_X_L_G                                     0x28

//////////////////////////////////////////
// LSM9DS0 Accel/Magneto (XM) Registers //
//////////////////////////////////////////
#define WHO_AM_I_XM                                   0x0F

#define CTRL_REG0_XM                                  0x1F
#define CTRL_REG1_XM                                  0x20
#   define CTRL_REG1_XM_AODR_1600Hz             (0xA << 4)
#   define CTRL_REG1_XM_BDU                     (0x1 << 3)
#   define CTRL_REG1_XM_AZEN                    (0x1 << 2)
#   define CTRL_REG1_XM_AYEN                    (0x1 << 1)
#   define CTRL_REG1_XM_AXEN                    (0x1 << 0)
#define CTRL_REG2_XM                                  0x21
#   define CTRL_REG2_XM_ABW_194Hz               (0x1 << 6)
#   define CTRL_REG2_XM_AFS_16G                 (0x4 << 3)
#define CTRL_REG3_XM                                  0x22
#   define CTRL_REG3_XM_P1_DRDYA                (0x1 << 2)
#define STATUS_REG_A                                  0x27
#   define STATUS_REG_A_ZYXADA                  (0x1 << 3)
#define OUT_X_L_A                                     0x28

#define ACT_DUR                                       0x3F //for debug



#define LSM9DS1_XG_WHOAMI     0x68

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



AP_InertialSensor_LSM9DSX::AP_InertialSensor_LSM9DSX(AP_InertialSensor &imu,
                                                     AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev_gyro,
                                                     AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev_accel,
                                                     int drdy_pin_num_a,
                                                     int drdy_pin_num_g,
                                                     enum Rotation rotation_a,
                                                     enum Rotation rotation_g,
                                                     enum Rotation rotation_gH,
                                                     enum Sensor_Type sensor,
                                                     int gyro_type,
                                                     int accel_type,
                                                     int sample_gyro_rate,
                                                     int sample_acel_rate,
                                                     int status_reg_accel,
                                                     int check_reg_accel,
                                                     int status_reg_gyro,
                                                     int check_reg_gyro,
                                                     int raw_reg,
                                                     int gyro_out)
    : AP_InertialSensor_Backend(imu)
    , _dev_gyro(std::move(dev_gyro))
    , _dev_accel(std::move(dev_accel))
    , _drdy_pin_num_a(drdy_pin_num_a)
    , _drdy_pin_num_g(drdy_pin_num_g)
    , _rotation_a(rotation_a)
    , _rotation_g(rotation_g)
    , _rotation_gH(rotation_gH)
    , _lsm_type(sensor)
    , _devtype_gyro(gyro_type)
    , _devtype_acc(accel_type)
    , _sample_rate_G(sample_gyro_rate)
    , _sample_rate_X(sample_acel_rate)
    , _status_reg_X (status_reg_accel)
    , _status_reg_X_check(check_reg_accel)
    , _status_reg_G(status_reg_gyro)
    , _status_reg_G_check(check_reg_gyro)
    , _raw_reg(raw_reg)
    , _out_g(gyro_out)

{
}

AP_InertialSensor_Backend *AP_InertialSensor_LSM9DSX::probe(AP_InertialSensor &_imu,
                                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev_gyro,
                                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev_accel,
                                                            enum Rotation rotation_a,
                                                            enum Rotation rotation_g,
                                                            enum Rotation rotation_gH)
{
    if (!dev_gyro || !dev_accel) {
        return nullptr;
    }
    AP_InertialSensor_LSM9DSX *sensor =
        new AP_InertialSensor_LSM9DSX(_imu, std::move(dev_gyro), std::move(dev_accel),
                                      LSM9DS0_DRY_X_PIN, LSM9DS0_DRY_G_PIN,
                                      rotation_a, rotation_g, rotation_gH,
                                      LSM9DS0, DEVTYPE_GYR_L3GD20, DEVTYPE_ACC_LSM303D,
                                      760, 1000,
                                      STATUS_REG_A, STATUS_REG_A_ZYXADA,
                                      STATUS_REG_G, STATUS_REG_G_ZYXDA,
                                      0xC0, OUT_X_L_G);
    if (!sensor || !sensor->_init_sensor()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

AP_InertialSensor_Backend *AP_InertialSensor_LSM9DSX::probe(AP_InertialSensor &_imu,
                                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                                            enum Rotation rotation_a
                                                            )
{
    if (!dev) {
        return nullptr;
    }
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev_accel = dev.get();
    AP_InertialSensor_LSM9DSX *sensor =
        new AP_InertialSensor_LSM9DSX(_imu, std::move(dev), std::move(dev_accel) ,
                                      LSM9DS1_DRY_XG_PIN, LSM9DS1_DRY_XG_PIN,
                                      rotation_a, rotation_a, ROTATION_NONE,
                                      LSM9DS1, DEVTYPE_GYR_LSM9DS1, DEVTYPE_ACC_LSM9DS1,
                                      952, 952,
                                      LSM9DS1XG_STATUS_REG, LSM9DS1XG_STATUS_REG_XLDA,
                                      LSM9DS1XG_STATUS_REG, LSM9DS1XG_STATUS_REG_GDA,
                                      0x80, LSM9DS1XG_OUT_X_L_G);

    if (!sensor || !sensor->_init_sensor()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}


bool AP_InertialSensor_LSM9DSX::_init_sensor()
{
    /*
     * Same semaphore for both since they necessarily share the same bus (with
     * different CS)
     */

    _spi_sem = _dev_gyro->get_semaphore();

    if (_drdy_pin_num_a >= 0) {
        _drdy_pin_a = hal.gpio->channel(_drdy_pin_num_a);
        if (_drdy_pin_a == nullptr) {
            AP_HAL::panic("LSM9DSX: null accel data-ready GPIO channel\n");
        }

        _drdy_pin_a->mode(HAL_GPIO_INPUT);
    }

    if (_drdy_pin_num_g >= 0) {
        _drdy_pin_g = hal.gpio->channel(_drdy_pin_num_g);
        if (_drdy_pin_g == nullptr) {
            AP_HAL::panic("LSM9DSX: null gyro data-ready GPIO channel\n");
        }

        _drdy_pin_g->mode(HAL_GPIO_INPUT);
    }

    bool success = _hardware_init();


    return  success;
}

bool AP_InertialSensor_LSM9DSX::_hardware_init()
{
    if (!_spi_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return false;
    }

    uint8_t tries, whoami;

    // set flag for reading registers
    _dev_gyro->set_read_flag(0x80);
    _dev_accel->set_read_flag(0x80);

    whoami_g = _register_read_g(WHO_AM_I_G);
    if (whoami_g != LSM9DS0_G_WHOAMI && whoami_g != LSM9DS0_G_WHOAMI_H && whoami_g != LSM9DS1_XG_WHOAMI) {
        hal.console->printf("LSM9DS0: unexpected gyro WHOAMI 0x%x\n", (unsigned)whoami_g);
        goto fail_whoami;
    }

    whoami = _register_read_xm(WHO_AM_I_XM);
    if (whoami != LSM9DS0_XM_WHOAMI && whoami !=LSM9DS1_XG_WHOAMI) {
        hal.console->printf("LSM9DS0: unexpected acc  WHOAMI 0x%x\n", (unsigned)whoami);
        goto fail_whoami;
    }

    // setup for register checking
    if (_lsm_type == LSM9DS0)
    {
        _dev_gyro->setup_checked_registers(5, 20);
        _dev_accel->setup_checked_registers(4, 20);
    } else {
        _dev_gyro->write_register(LSM9DS1XG_CTRL_REG9,LSM9DS1XG_CTRL_REG9_I2C_DISABLE);  //simultaneously disable for accel and gyro on LSM9DS1
    }
    for (tries = 0; tries < 5; tries++) {
        _dev_gyro->set_speed(AP_HAL::Device::SPEED_LOW);
        _dev_accel->set_speed(AP_HAL::Device::SPEED_LOW);

        _gyro_init();
        _accel_init();

        _dev_gyro->set_speed(AP_HAL::Device::SPEED_HIGH);
        _dev_accel->set_speed(AP_HAL::Device::SPEED_HIGH);

        hal.scheduler->delay(10);
        if (_accel_data_ready() && _gyro_data_ready()) {
            break;
        }

    }
    if (tries == 5) {
        hal.console->printf("Failed to boot LSM9DS0 5 times\n\n");
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
void AP_InertialSensor_LSM9DSX::start(void)
{
    _gyro_instance = _imu.register_gyro(_sample_rate_G, _dev_gyro->get_bus_id_devtype(_devtype_gyro));
    _accel_instance = _imu.register_accel(_sample_rate_X, _dev_accel->get_bus_id_devtype(_devtype_acc));


    if (whoami_g == LSM9DS0_G_WHOAMI_H) {
        set_gyro_orientation(_gyro_instance, _rotation_gH);
    } else {
        set_gyro_orientation(_gyro_instance, _rotation_g);
    }
    set_accel_orientation(_accel_instance, _rotation_a);

    _set_accel_max_abs_offset(_accel_instance, 5.0f);

    /* start the timer process to read samples */
    _dev_gyro->register_periodic_callback(1000, FUNCTOR_BIND_MEMBER(&AP_InertialSensor_LSM9DSX::_poll_data, void));
}


uint8_t AP_InertialSensor_LSM9DSX::_register_read_xm(uint8_t reg)
{
    uint8_t val = 0;

    _dev_accel->read_registers(reg, &val, 1);

    return val;
}

uint8_t AP_InertialSensor_LSM9DSX::_register_read_g(uint8_t reg)
{
    uint8_t val = 0;

    _dev_gyro->read_registers(reg, &val, 1);

    return val;
}

void AP_InertialSensor_LSM9DSX::_register_write_xm(uint8_t reg, uint8_t val, bool checked)
{
    _dev_accel->write_register(reg, val, checked);
}

void AP_InertialSensor_LSM9DSX::_register_write_g(uint8_t reg, uint8_t val, bool checked)
{
    _dev_gyro->write_register(reg, val, checked);
}

void AP_InertialSensor_LSM9DSX::_gyro_disable_i2c()
{
    uint8_t retries = 10;
    while (retries--) {
        // add retries
        uint8_t a = _register_read_g(0x05);
        _register_write_g(0x05, (0x20 | a));
        if (_register_read_g(0x05) == (a | 0x20)) {
            return;
        }
    }
    AP_HAL::panic("LSM9DS0_G: Unable to disable I2C");
}

void AP_InertialSensor_LSM9DSX::_accel_disable_i2c()
{
    uint8_t a = _register_read_xm(0x02);
    _register_write_xm(0x02, (0x10 | a));
    a = _register_read_xm(0x02);
    _register_write_xm(0x02, (0xF7 & a));
    a = _register_read_xm(0x15);
    _register_write_xm(0x15, (0x80 | a));
    a = _register_read_xm(0x02);
    _register_write_xm(0x02, (0xE7 & a));
}

void AP_InertialSensor_LSM9DSX::_gyro_init()
{
    if (_lsm_type == LSM9DS0)
    {
        _gyro_disable_i2c();
        hal.scheduler->delay(1);

        _register_write_g(CTRL_REG1_G,
                          CTRL_REG1_G_DR_760Hz_BW_50Hz |
                          CTRL_REG1_G_PD |
                          CTRL_REG1_G_ZEN |
                          CTRL_REG1_G_YEN |
                          CTRL_REG1_G_XEN, true);
        hal.scheduler->delay(1);

        _register_write_g(CTRL_REG2_G, 0x00, true);
        hal.scheduler->delay(1);

        /*
         * Gyro data ready on DRDY_G
         */
        _register_write_g(CTRL_REG3_G, CTRL_REG3_G_I2_DRDY, true);
        hal.scheduler->delay(1);

        _register_write_g(CTRL_REG4_G,
                          CTRL_REG4_G_BDU |
                          CTRL_REG4_G_FS_2000DPS, true);

        _register_write_g(CTRL_REG5_G, 0x00, true);
        hal.scheduler->delay(1);
    } else {

        _register_write_g(LSM9DS1XG_CTRL_REG1_G, LSM9DS1XG_CTRL_REG1_G_ODR_G_952Hz |
                                                  LSM9DS1XG_CTRL_REG1_FS_G_2000DPS);
        hal.scheduler->delay(1);

        _register_write_g(LSM9DS1XG_CTRL_REG4, LSM9DS1XG_CTRL_REG4_Zen_G |
                                                LSM9DS1XG_CTRL_REG4_Yen_G |
                                                LSM9DS1XG_CTRL_REG4_Xen_G);
    }

    _set_gyro_scale(G_SCALE_2000DPS);
    hal.scheduler->delay(1);
}

void AP_InertialSensor_LSM9DSX::_accel_init()
{
    if (_lsm_type == LSM9DS0)
    {
    _accel_disable_i2c();
    hal.scheduler->delay(1);

    _register_write_xm(CTRL_REG0_XM, 0x00, true);
    hal.scheduler->delay(1);

    _register_write_xm(CTRL_REG1_XM,
                       CTRL_REG1_XM_AODR_1600Hz |
                       CTRL_REG1_XM_BDU |
                       CTRL_REG1_XM_AZEN |
                       CTRL_REG1_XM_AYEN |
                       CTRL_REG1_XM_AXEN, true);
    hal.scheduler->delay(1);

    _register_write_xm(CTRL_REG2_XM,
                       CTRL_REG2_XM_ABW_194Hz |
                       CTRL_REG2_XM_AFS_16G, true);

    /* Accel data ready on INT1 */
    _register_write_xm(CTRL_REG3_XM, CTRL_REG3_XM_P1_DRDYA, true);
    hal.scheduler->delay(1);

    } else {

        _register_write_xm(LSM9DS1XG_CTRL_REG6_XL, LSM9DS1XG_CTRL_REG6_XL_ODR_XL_952Hz |
                                                   LSM9DS1XG_CTRL_REG6_XL_FS_XL_16G);
        hal.scheduler->delay(1);

        _register_write_xm(LSM9DS1XG_CTRL_REG5_XL, LSM9DS1XG_CTRL_REG5_XL_Zen_XL |
                                                   LSM9DS1XG_CTRL_REG5_XL_Yen_XL |
                                                   LSM9DS1XG_CTRL_REG5_XL_Xen_XL);

    }
    _set_accel_scale(A_SCALE_16G);
    hal.scheduler->delay(1);
}

void AP_InertialSensor_LSM9DSX::_set_gyro_scale(gyro_scale scale)
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

void AP_InertialSensor_LSM9DSX::_set_accel_scale(accel_scale scale)
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
 * Timer process to poll for new data from the LSM9DS0.
 */
void AP_InertialSensor_LSM9DSX::_poll_data()
{
    if (_gyro_data_ready()) {
        _read_data_transaction_g();
    }
    if (_accel_data_ready()) {
        _read_data_transaction_a();
    }

    // check next register value for correctness
    if (!_dev_gyro->check_next_register()) {
        _inc_gyro_error_count(_gyro_instance);
    }
    if (!_dev_accel->check_next_register()) {
        _inc_accel_error_count(_accel_instance);
    }
}

bool AP_InertialSensor_LSM9DSX::_accel_data_ready()
{
    if (_drdy_pin_a != nullptr) {
        return _drdy_pin_a->read() != 0;
    }

    uint8_t status = _register_read_xm(_status_reg_X);
    return status & _status_reg_X_check;
}

bool AP_InertialSensor_LSM9DSX::_gyro_data_ready()
{
    if (_drdy_pin_g != nullptr) {
        return _drdy_pin_g->read() != 0;
    }

    uint8_t status = _register_read_g(_status_reg_G);
    return status & _status_reg_G_check;
}

void AP_InertialSensor_LSM9DSX::_read_data_transaction_a()
{
    struct sensor_raw_data raw_data = { };
    const uint8_t reg = OUT_X_L_A | _raw_reg;

    if (!_dev_accel->transfer(&reg, 1, (uint8_t *) &raw_data, sizeof(raw_data))) {
        hal.console->printf("LSM9DS0: error reading accelerometer\n");
        return;
    }

    if (_lsm_type == LSM9DS1)
            raw_data.y = -raw_data.y;

    Vector3f accel_data(raw_data.x, -raw_data.y, -raw_data.z);
    accel_data *= _accel_scale;

    _rotate_and_correct_accel(_accel_instance, accel_data);
    _notify_new_accel_raw_sample(_accel_instance, accel_data, AP_HAL::micros64());
}

/*
 *  read from the data registers and update filtered data
 */
void AP_InertialSensor_LSM9DSX::_read_data_transaction_g()
{
    struct sensor_raw_data raw_data = { };
    const uint8_t reg = _out_g | _raw_reg;

    if (!_dev_gyro->transfer(&reg, 1, (uint8_t *) &raw_data, sizeof(raw_data))) {
        hal.console->printf("LSM9DS0: error reading gyroscope\n");
        return;
    }

    if (_lsm_type == LSM9DS1)
            raw_data.y = -raw_data.y;

    Vector3f gyro_data(raw_data.x, -raw_data.y, -raw_data.z);

    gyro_data *= _gyro_scale;

    _rotate_and_correct_gyro(_gyro_instance, gyro_data);
    _notify_new_gyro_raw_sample(_gyro_instance, gyro_data, AP_HAL::micros64());
}

bool AP_InertialSensor_LSM9DSX::update()
{
    update_gyro(_gyro_instance);
    update_accel(_accel_instance);

    return true;
}


