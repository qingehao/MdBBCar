#include "BLDCDriver3PWM.h"
#include "drv_gpio.h"

#define DD_USE_BSP_FOC
BLDCDriver3PWM::BLDCDriver3PWM(uint8_t motor_index, int en_pin){
  // Pin initialization

  this->motor_index = motor_index;
  this->enable_pin = en_pin;

  // default power-supply value
  voltage_power_supply = DEF_POWER_SUPPLY;
  voltage_limit = NOT_SET;
  pwm_frequency = NOT_SET;

#ifndef DD_USE_BSP_FOC
  pwm_dev = bsp_pwm_request(2);
#else
  bsp_foc = bsp_foc_request();
#endif

}

// enable motor driver
void  BLDCDriver3PWM::enable(){
#ifndef DD_USE_BSP_FOC
  bsp_pwm_set_duty(pwm_dev, 1, 0);
  bsp_pwm_set_duty(pwm_dev, 2, 0);
  bsp_pwm_set_duty(pwm_dev, 3, 0);
#else
  if ( _isset(enable_pin) ) rt_pin_write(enable_pin, 1);
  bsp_foc_set_pwm(bsp_foc, motor_index, 0, 0, 0);
#endif
}

// disable motor driver
void BLDCDriver3PWM::disable()
{
#ifndef DD_USE_BSP_FOC
  bsp_pwm_set_duty(pwm_dev, 1, 0);
  bsp_pwm_set_duty(pwm_dev, 2, 0);
  bsp_pwm_set_duty(pwm_dev, 3, 0);
#else
  if ( _isset(enable_pin) ) rt_pin_write(enable_pin, 0);
  bsp_foc_set_pwm(bsp_foc, motor_index, 0, 0, 0);
#endif
}

// init hardware pins
int BLDCDriver3PWM::init() {
	// PWM pins
	// sanity check for the voltage limit configuration
	if(!_isset(voltage_limit) || voltage_limit > voltage_power_supply) voltage_limit =  voltage_power_supply;

#ifndef DD_USE_BSP_FOC
	bsp_pwm_enable(pwm_dev, 1, 1);
	bsp_pwm_enable(pwm_dev, 2, 1);
	bsp_pwm_enable(pwm_dev, 3, 1);
#else
	if ( _isset(enable_pin) )
	{
		rt_pin_mode(enable_pin, PIN_MODE_OUTPUT);
		rt_pin_write(enable_pin, 0);
	}
	bsp_foc_pwm_enable(bsp_foc, motor_index, 1);
#endif
	initialized = 1;
	return 1;
}



// Set voltage to the pwm pin
void BLDCDriver3PWM::setPhaseState(PhaseState sa, PhaseState sb, PhaseState sc) {

}

// Set voltage to the pwm pin
void BLDCDriver3PWM::setPwm(float Ua, float Ub, float Uc) {

  // limit the voltage in driver
  Ua = _constrain(Ua, 0.0f, voltage_limit);
  Ub = _constrain(Ub, 0.0f, voltage_limit);
  Uc = _constrain(Uc, 0.0f, voltage_limit);
  // calculate duty cycle
  // limited in [0,1]
  dc_a = _constrain(Ua / voltage_power_supply, 0.0f , 1.0f );
  dc_b = _constrain(Ub / voltage_power_supply, 0.0f , 1.0f );
  dc_c = _constrain(Uc / voltage_power_supply, 0.0f , 1.0f );

  // hardware specific writing
  // hardware specific function - depending on driver and mcu
#ifndef DD_USE_BSP_FOC
  bsp_pwm_set_duty(pwm_dev, 1, dc_a);
  bsp_pwm_set_duty(pwm_dev, 2, dc_b);
  bsp_pwm_set_duty(pwm_dev, 3, dc_c);
#else
  bsp_foc_set_pwm(bsp_foc, motor_index, dc_a, dc_b, dc_c);
#endif
}
