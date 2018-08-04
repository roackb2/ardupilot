#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

//some code are copied from paparazziuav

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_MAMBO
#include "RCOutput_Mambo.h"
#include "Util.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdio.h>

/* build ioctl unique identifiers for R/W operations */
#define PWM_MAGIC 'p'
typedef struct { unsigned int val[4]; } __attribute__ ((packed)) pwm_delos_quadruplet;
#define PWM_DELOS_SET_RATIOS _IOR(PWM_MAGIC, 9,  pwm_delos_quadruplet*)
#define PWM_DELOS_SET_SPEEDS _IOR(PWM_MAGIC, 10, pwm_delos_quadruplet*)
#define PWM_DELOS_SET_CTRL   _IOR(PWM_MAGIC, 11, unsigned int)
#define PWM_DELOS_REQUEST    _IO(PWM_MAGIC, 12)

#define PWM_NB_BITS   (9)

/* PWM can take value between 0 and 511 */
#ifndef PWM_TOTAL_RANGE
#define PWM_TOTAL_RANGE (1<<PWM_NB_BITS)
#endif

#define PWM_REG_RATIO_PRECISION_MASK (PWM_NB_BITS<<16)
#define PWM_REG_SATURATION (PWM_REG_RATIO_PRECISION_MASK|PWM_TOTAL_RANGE)

using namespace Linux;

static const AP_HAL::HAL& hal = AP_HAL::get_HAL();

enum {
  SiP6_PWM0_START = (1<<0),
  SiP6_PWM1_START = (1<<1),
  SiP6_PWM2_START = (1<<2),
  SiP6_PWM3_START = (1<<3),
};

RCOutput_Mambo::RCOutput_Mambo()
{
}

void RCOutput_Mambo::init()
{
    _min_pwm = 1100;
    _max_pwm = 1900;
    _frequency = 50;
    _cork = false;
    _changed = false;

    actuators_fd = open("/dev/pwm", O_RDWR);
    pwm_delos_quadruplet m = {{ 1, 1, 1, 1 }};
    int ret __attribute__((unused)) = ioctl(actuators_fd, PWM_DELOS_SET_SPEEDS, &m);
    printf("Return Speeds: %d\n", ret);

    unsigned int control_reg = (SiP6_PWM0_START|SiP6_PWM1_START|SiP6_PWM2_START|SiP6_PWM3_START);
    ret = ioctl(actuators_fd, PWM_DELOS_SET_CTRL, &control_reg);
    printf("Return control: %d\n", ret);
}

void RCOutput_Mambo::set_freq(uint32_t chmask, uint16_t freq_hz)
{
    _frequency = freq_hz;
}

uint16_t RCOutput_Mambo::get_freq(uint8_t ch)
{
    return _frequency;
}

void RCOutput_Mambo::enable_ch(uint8_t ch)
{
}

void RCOutput_Mambo::disable_ch(uint8_t ch)
{
}

void RCOutput_Mambo::write(uint8_t ch, uint16_t period_us)
{
    if (ch < 4) {
        if (period_us != _period_us[ch]) {
            _changed = true;
            _period_us[ch] = period_us;
            _rpm_ref[ch] = _period_us_to_rpm(period_us);
        }
    }
    if (!_cork) push();
}

void RCOutput_Mambo::cork()
{
    _cork = true;
}

void RCOutput_Mambo::push()
{
    pwm_delos_quadruplet m;

    if (!_cork) {
        return;
    }
    _cork = false;

    if (!_changed) return;
    _changed = false;
 
  m.val[0] = _rpm_ref[0] & 0xffff;
  m.val[1] = _rpm_ref[1] & 0xffff;
  m.val[2] = _rpm_ref[2] & 0xffff;
  m.val[3] = _rpm_ref[3] & 0xffff;


  if( _rpm_ref[0] > (PWM_TOTAL_RANGE) ) { m.val[0] = PWM_REG_SATURATION; }
  if( _rpm_ref[1] > (PWM_TOTAL_RANGE) ) { m.val[1] = PWM_REG_SATURATION; }
  if( _rpm_ref[2] > (PWM_TOTAL_RANGE) ) { m.val[2] = PWM_REG_SATURATION; }
  if( _rpm_ref[3] > (PWM_TOTAL_RANGE) ) { m.val[3] = PWM_REG_SATURATION; }

  /* The upper 16-bit word of the ratio register contains the number
   * of bits used to code the ratio command  */
  m.val[0] |= PWM_REG_RATIO_PRECISION_MASK;
  m.val[1] |= PWM_REG_RATIO_PRECISION_MASK;
  m.val[2] |= PWM_REG_RATIO_PRECISION_MASK;
  m.val[3] |= PWM_REG_RATIO_PRECISION_MASK;

  int ret __attribute__((unused)) = ioctl(actuators_fd, PWM_DELOS_SET_RATIOS, &m);
}

uint16_t RCOutput_Mambo::read(uint8_t ch)
{
    if (ch < 4) {
        return _period_us[ch];
    } else {
        return 0;
    }
}

void RCOutput_Mambo::read(uint16_t* period_us, uint8_t len)
{
    for (int i = 0; i < len; i++) {
        period_us[i] = read(0 + i);
    }
}

void RCOutput_Mambo::set_esc_scaling(uint16_t min_pwm, uint16_t max_pwm)
{
    _min_pwm = min_pwm;
    _max_pwm = max_pwm;
}

uint16_t RCOutput_Mambo::_period_us_to_rpm(uint16_t period_us)
{
    period_us = constrain_int16(period_us, _min_pwm, _max_pwm);
    float period_us_fl = period_us;
    float rpm_fl = (period_us_fl - _min_pwm)/(_max_pwm - _min_pwm) * 511;

    return (uint16_t)rpm_fl;
}

#endif
