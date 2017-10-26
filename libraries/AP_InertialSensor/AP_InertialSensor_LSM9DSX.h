#ifndef AP_INERTIALSENSOR_LSM9DSX_H
#define AP_INERTIALSENSOR_LSM9DSX_H

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/SPIDevice.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

class AP_InertialSensor_LSM9DSX : public AP_InertialSensor_Backend
{
public:
    virtual ~AP_InertialSensor_LSM9DSX() { }
    void start(void) override;
    bool update() override;

    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev_gyro,
                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev_accel,
                                            enum Rotation rotation_a = ROTATION_NONE,
                                            enum Rotation rotation_g = ROTATION_NONE,
                                            enum Rotation rotation_gH = ROTATION_NONE);

    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev_gyro,
                                            enum Rotation rotation_a = ROTATION_NONE
                                            );

    enum Sensor_Type
    {
        LSM9DS0 = 0,
        LSM9DS1
    };


private:
    AP_InertialSensor_LSM9DSX(AP_InertialSensor &imu,
                              AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev_gyro,
                              AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev_accel,
                              int drdy_pin_num_a, int drdy_pin_num_b,
                              enum Rotation rotation_a,
                              enum Rotation rotation_g,
                              enum Rotation rotation_gH,
                              enum Sensor_Type sensor,
                              int gyro_type, int accel_type,
                              int sample_gyro_rate, int sample_acel_rate,
                              int status_reg_accel, int check_reg_accel,
                              int status_reg_gyro, int check_reg_gyro,
                              int raw_reg,
                              int gyro_out);



    struct PACKED sensor_raw_data {
        int16_t x;
        int16_t y;
        int16_t z;
    };

    enum gyro_scale {
        G_SCALE_245DPS = 0,
        G_SCALE_500DPS,
        G_SCALE_2000DPS,
    };

    enum accel_scale {
        A_SCALE_2G = 0,
        A_SCALE_4G,
        A_SCALE_6G,
        A_SCALE_8G,
        A_SCALE_16G,
    };


    bool _accel_data_ready();
    bool _gyro_data_ready();

    void _poll_data();

    bool _init_sensor();
    bool _hardware_init();

    void _gyro_init();
    void _accel_init();

    void _gyro_disable_i2c();
    void _accel_disable_i2c();

    void _set_gyro_scale(gyro_scale scale);
    void _set_accel_scale(accel_scale scale);

    uint8_t _register_read_xm(uint8_t reg);
    uint8_t _register_read_g(uint8_t reg);
    void _register_write_xm(uint8_t reg, uint8_t val, bool checked=false);
    void _register_write_g(uint8_t reg, uint8_t val, bool checked=false);

    void _read_data_transaction_a();
    void _read_data_transaction_g();


    AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev_gyro;
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev_accel;
    AP_HAL::Semaphore *_spi_sem;

    /*
     * If data-ready GPIO pins numbers are not defined (i.e. any negative
     * value), the fallback approach used is to check if there's new data ready
     * by reading the status register. It is *strongly* recommended to use
     * data-ready GPIO pins for performance reasons.
     */
    AP_HAL::DigitalSource * _drdy_pin_a;
    AP_HAL::DigitalSource * _drdy_pin_g;
    float _gyro_scale;
    float _accel_scale;
    int _drdy_pin_num_a;
    int _drdy_pin_num_g;
    uint8_t _gyro_instance;
    uint8_t _accel_instance;

    // gyro whoami
    uint8_t whoami_g;

    /*
      for boards that have a separate LSM303D and L3GD20 there can be
      different rotations for each
     */
    enum Rotation _rotation_a;
    enum Rotation _rotation_g;  // for L3GD20
    enum Rotation _rotation_gH; // for L3GD20H

    enum Sensor_Type _lsm_type;

    int _devtype_gyro;
    int _devtype_acc;

    int _G_WHOAMI;
    int _X_WHOAMI;
    uint16_t _sample_rate_G;
    uint16_t _sample_rate_X;
    uint8_t _status_reg_X;
    uint8_t _status_reg_X_check;
    uint8_t _status_reg_G;
    uint8_t _status_reg_G_check;
    uint8_t _raw_reg;
    uint8_t _out_g;
};

#endif // AP_INERTIALSENSOR_LSM9DSX_H
