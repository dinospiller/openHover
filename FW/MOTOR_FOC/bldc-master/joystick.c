/*
 * joystick.c
 *
 *  Created on: 14/giu/2017
 *      Author: mac_daino
 */

#include "ch.h"
#include "hal.h"
#include "applications/app.h"
#include "joystick.h"


#define ADC_DRV_USED			ADC1
#define ADC_BUF_DEPTH      		8
#define ADC_FORWARD_PORT		GPIOC
#define ADC_FORWARD_BIT			PAL_PORT_BIT(1)
#define ADC_STEERING_PORT		GPIOC
#define ADC_STEERING_BIT		PAL_PORT_BIT(2)
#define ADC_FORWARD_CH			ADC_CHANNEL_IN7
#define ADC_STEERING_CH			ADC_CHANNEL_IN8

#define JOY_DEADZONE_LIMIT		(float)0.2

typedef enum{
	ADC_NUM_FWD = 0,
	ADC_NUM_STEER,
	ADC_NUM_CHANNELS
}ADC_GRP1_CH_TYPE;

typedef enum{
	JOYSTICK_CALIBRATING,
	JOYSTICK_RUNNING
}JOYSTICK_STATE;

float joystick_last_fwd_val;
float joystick_last_steer_val;
float fwd_bias;
float steer_bias;

static JOYSTICK_STATE joystick_state;
//static adcsample_t samples1[ADC_NUM_CHANNELS * ADC_BUF_DEPTH];
//
///*
// * ADC conversion group.
// * Mode:        Linear buffer, 8 samples of 2 channels, SW triggered.
// */
//static const ADCConversionGroup adcgrpcfg1 = {
//  FALSE,
//  ADC_NUM_CHANNELS,
//  NULL,
//  NULL,
//  ADC_CFGR_CONT,            /* CFGR    */
//  ADC_TR(0, 4095),          /* TR1     */
//  0,                        /* CCR     */
//  {                         /* SMPR[2] */
//    0,
//    0
//  },
//  {                         /* SQR[4]  */
//    ADC_SQR1_SQ1_N(ADC_FORWARD_CH) | ADC_SQR1_SQ2_N(ADC_STEERING_CH),
//    0,
//    0,
//    0
//  }
//};
//
//uint16_t AvgSamples(uint16_t *buf, uint8_t num, ADC_GRP1_CH_TYPE ch){
//	uint8_t ind, ind_initial;
//	uint16_t result=0;
//	if(ch== ADC_NUM_FWD){
//		ind_initial =0;
//	}else{
//		ind_initial =1;
//	}
//	for(ind=ind_initial;ind<(num*ADC_NUM_CHANNELS);ind+=ADC_NUM_CHANNELS){
//		result+=buf[ind];
//	}
//	result/=num;
//	return result;
//}
//
//
//// map the output to +/- 32
//float AdcToJoyVal(uint16_t val, uint16_t bias, uint16_t max_adc_val){
//	float value = (float)(((float)val-(float)bias)/(float)max_adc_val);
//	return value;
//}
//
///*
// * ADC conversion thread, times are in milliseconds.
// */
//static THD_WORKING_AREA(waThread1, 512);
//static THD_FUNCTION(Thread1, arg) {
//
//	(void)arg;
//	chRegSetThreadName("joystick_read_thread");
//	uint16_t fwdAvg;
//	uint16_t steerAvg;
//
//	for(;;){
//		/*
//		* Linear conversion.
//		*/
//		adcConvert(&ADCD1, &adcgrpcfg1, samples1, ADC_BUF_DEPTH);
//
//		fwdAvg=AvgSamples(samples1,ADC_BUF_DEPTH,ADC_NUM_FWD);
//		steerAvg=AvgSamples(samples1,ADC_BUF_DEPTH,ADC_NUM_STEER);
//
//
//		switch(joystick_state){
//		case JOYSTICK_CALIBRATING:
//#ifdef PRINT_DEBUG_JOYSTICK
//			// printout last conversions
//			comm_usb_printf("Joystick calibrating...");
//#endif //PRINT_DEBUG
//			fwd_bias=fwdAvg;
//			steer_bias=steerAvg;
//			joystick_state=JOYSTICK_RUNNING;
//			break;
//		case JOYSTICK_RUNNING:
//			// printout last conversions
//			joystick_last_fwd_val = AdcToJoyVal(fwdAvg,fwd_bias,4096);
//			joystick_last_steer_val = AdcToJoyVal(steerAvg,steer_bias,4096);
//#ifdef PRINT_DEBUG_JOYSTICK
//			comm_usb_printf("Joy_fwd:%.4f;Joy_steer:%.4f\n",joystick_last_fwd_val,joystick_last_steer_val);
//#endif //PRINT_DEBUG
//			break;
//		}
//
//		chThdSleepMilliseconds(50);
//	}
//}


// this module makes use of "app_adc" and overwrites its settings
void JoystickInit(void){

	joystick_state=JOYSTICK_CALIBRATING;

	// overwrite configuration settings:
	adc_config cfg;
	cfg.update_rate_hz = 50;
	cfg.voltage_inverted = false;
	cfg.use_filter = true;

	app_adc_configure(&cfg);

	app_adc_start(false);// don't use tx and rx as input pins

	chThdSleepMilliseconds(100);

	fwd_bias = app_adc_get_voltage();
	steer_bias = app_adc_get_voltage2();

	chThdSleepMilliseconds(100);
}

float JoystickGetForward(void){
	// map the output to max +/- 1
	float ret_val = (app_adc_get_voltage()-fwd_bias)/fwd_bias;

	// return zero value in the "dead-zone"
	if(((ret_val > 0)&&(ret_val<JOY_DEADZONE_LIMIT))||
		((ret_val < 0)&&(ret_val>-JOY_DEADZONE_LIMIT))){
		ret_val=0.0;
	}

	return ret_val;
}
float JoystickGetSteering(void){
	// map the output to max +/- 1
	float ret_val = (app_adc_get_voltage2()-steer_bias)/steer_bias;

	// return zero value in the "dead-zone"
	if(((ret_val > 0)&&(ret_val<JOY_DEADZONE_LIMIT))||
		((ret_val < 0)&&(ret_val>-JOY_DEADZONE_LIMIT))){
		ret_val=0.0;
	}

	return ret_val;
}
