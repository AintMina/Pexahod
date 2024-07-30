#include "sensors.h"


// Set up the shared analog inputs
Analog sensor_adc = Analog(servo2040::SHARED_ADC);
Analog voltage_adc = Analog(servo2040::SHARED_ADC, servo2040::VOLTAGE_GAIN);
Analog current_adc = Analog(servo2040::SHARED_ADC, servo2040::CURRENT_GAIN, servo2040::SHUNT_RESISTOR, servo2040::CURRENT_OFFSET);

// Set up the analog multiplexer, including the pin for controlling pull-up/pull-down
AnalogMux mux = AnalogMux(servo2040::ADC_ADDR_0, servo2040::ADC_ADDR_1, servo2040::ADC_ADDR_2, PIN_UNUSED, servo2040::SHARED_ADC);

// Create the user button
Button user_sw(servo2040::USER_SW);


uint8_t sensors_init() {
  // Set up the sensor addresses with pull downs
    for(auto i = 0u; i < servo2040::NUM_SENSORS; i++) {
        mux.configure_pulls(servo2040::SENSOR_1_ADDR + i, false, true);
    }
    return 0;
}


uint8_t read_end_stop_sensors() {
    uint8_t value = 0;

    for(auto i = 0u; i < servo2040::NUM_SENSORS; i++) {
        mux.select(servo2040::SENSOR_1_ADDR + i);
        if (sensor_adc.read_voltage() > 1.7) {
            value |= 0x01;
        }
        value << 1;
    }

    return value;
}


float read_voltage_sensor() {
    mux.select(servo2040::VOLTAGE_SENSE_ADDR);
    return voltage_adc.read_voltage();
}


float read_current_sensor() {
    mux.select(servo2040::CURRENT_SENSE_ADDR);
    return current_adc.read_current();
}