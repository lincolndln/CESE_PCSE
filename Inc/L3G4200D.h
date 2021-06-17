/*
 * L3G4200D.h
 *
 *  Created in: May, 2021
 *      Author: Lincoln de León
 */

#ifndef INC_L3G4200D_H_
#define INC_L3G4200D_H_


/*
 *		Libreria para comunicacion I2C entre el Giroscopio L3G4200D y el
 *		microcontrolador STM32F401RE
 *
 *
 * ---Instructions---
 * If pin SDO (4) is connected to VCC LSb = 1, the slave address is : 1101 001	(7 bits)
 * If pin SDO (4) is connected to LSb = 0, the slave address is : 1101 000		(7 bits)
 * So the L3G4200D_ADDRESS could be: 0xD2 or 0xD0 respectively.
 *
 * Select a number for SAMPLES_FOR_THRESHOLD
 *
 * The multiplier for the standard deviation, MUL_STDDEV, is recommended as three (3)
 * */





/*--------------------------------------------*/
/*----------------- includes -----------------*/
/*--------------------------------------------*/
#include "main.h"
#include <stdbool.h>
#include <math.h>


/*-------------------------------------------*/
/*----------------- defines -----------------*/
/*-------------------------------------------*/

/* Modify for another board, in this case the ""PmGYro Digilent"" is used */
#define L3G4200D_ADDRESS 		0xD2	/* SDO/SA0 = 1 in PmGYro Digilent */

/*	Reserved registers - 00-0E 	*/
#define L3G4200D_WHO_AM_I_R 	0x0F
/*	Reserved registers - 10-1F	*/

#define L3G4200D_CTRL_REG1_RW	0x20
#define L3G4200D_CTRL_REG2_RW 	0x21
#define L3G4200D_CTRL_REG3_RW 	0x22
#define L3G4200D_CTRL_REG4_RW 	0x23
#define L3G4200D_CTRL_REG5_RW 	0x24

#define L3G4200D_REFERENCE_RW 	0x25
#define L3G4200D_OUT_TEMP_R 	0x26
#define L3G4200D_STATUS_REG_R 	0x27


#define L3G4200D_OUT_X_L_R 		0x28
#define L3G4200D_OUT_X_H_R 		0x29

#define L3G4200D_OUT_Y_L_R 		0x2A
#define L3G4200D_OUT_Y_H_R 		0x2B

#define L3G4200D_OUT_Z_L_R 		0x2C
#define L3G4200D_OUT_Z_H_R 		0x2D

#define L3G4200D_FIFO_CTRL_REG_RW 	0x2E
#define L3G4200D_FIFO_SRC_REG_R 	0x2F

#define L3G4200D_INT1_CFG_RW 		0x30
#define L3G4200D_INT1_SRC_R 		0x31

#define L3G4200D_INT1_TSH_XH_RW		0x32
#define L3G4200D_INT1_TSH_XL_RW 	0x33

#define L3G4200D_INT1_TSH_YH_RW 	0x34
#define L3G4200D_INT1_TSH_YL_RW 	0x35

#define L3G4200D_INT1_TSH_ZH_RW 	0x36
#define L3G4200D_INT1_TSH_ZL_RW 	0x37

#define L3G4200D_INT1_DURATION_RW 	0x38


/*----------------------------------------------*/
/*----------------- my defines -----------------*/
/*----------------------------------------------*/
#define MY_I2C_DELAY HAL_MAX_DELAY
#define MUL_STDDEV 3
#define SAMPLES_FOR_THRESHOLD 100   		/* [50, 100] recommended range */

/*-----------------------------------------*/
/*----------------- enums -----------------*/
/*-----------------------------------------*/


/* ----- CTRL_REG1 ----- */
/*
 * Used for Output Data Rate selection (ODR), Bandwidth selection (BW)
 * */
typedef enum{
	L3G4200D_ODR_100_BW_12_5	= 0b0000,	/* Default */
	L3G4200D_ODR_100_BW_25		= 0b0001,  	/* = 0b0001, 0b0010, 0b0011 and 0b0011 */

	L3G4200D_ODR_200_BW_12_5	= 0b0100,
	L3G4200D_ODR_200_BW_25		= 0b0101,
	L3G4200D_ODR_200_BW_50		= 0b0110,
	L3G4200D_ODR_200_BW_70		= 0b0111,

	L3G4200D_ODR_400_BW_20		= 0b1000,
	L3G4200D_ODR_400_BW_25		= 0b1001,
	L3G4200D_ODR_400_BW_50		= 0b1010,
	L3G4200D_ODR_400_BW_110		= 0b1011,

	L3G4200D_ODR_800_BW_30		= 0b1100,
	L3G4200D_ODR_800_BW_35		= 0b1101,
	L3G4200D_ODR_800_BW_50		= 0b1110,
	L3G4200D_ODR_800_BW_110		= 0b1111,

}l3g4200d_odr_bw_t;


/* ----- CTRL_REG2 ----- */
/*
 * Used to configure High pass filter mode
 * */
typedef enum{
	L3G4200D_HPM_NORMAL_MODE_00_DEFAULT	= 0b00,	/* Default */
	L3G4200D_HPM_REF_SIG_4_FILTERING	= 0b01,
	L3G4200D_HPM_NORMAL_MODE			= 0b10,
	L3G4200D_HPM_AUTORESET_ON_INTERRPUT_EVENT = 0b11
}l3g4200d_highPassFilterMode_t;


/*
 * Used for High pass filter cut off frequency configuration [Hz]
 * */
typedef enum{
	L3G4200D_HPCF_0000	= 0b0000, 	/* Default */
	L3G4200D_HPCF_0001	= 0b0001,
	L3G4200D_HPCF_0010	= 0b0010,
	L3G4200D_HPCF_0011	= 0b0011,
	L3G4200D_HPCF_0100	= 0b0100,
	L3G4200D_HPCF_0101	= 0b0101,
	L3G4200D_HPCF_0110	= 0b0110,
	L3G4200D_HPCF_0111	= 0b0111,
	L3G4200D_HPCF_1000	= 0b1000,
	L3G4200D_HPCF_1001	= 0b1001
}l3g4200d_highPassFilterCutOffFreq_t;



/* ----- CTRL_REG4 ----- */
/*
 * Used for Block Data Update
 * */
typedef enum{
	L3G4200D_BDU_CONTINUOUS 			= 0,	/* Default */
	L3G4200D_BDU_UPDATED_UNTIL_READING	= 1
}l3g4200d_block_data_update_t;

/*
 * Big/Little Endian Data Selection.
 * */
typedef enum{
	L3G4200D_BLE_LSB_LOWER_ADDRESS = 0,	/* Default */
	L3G4200D_BLE_MSB_LOWER_ADDRESS = 1
}l3g4200d_ble_data_t;

/*
 * Full Scale selection
 * */
typedef enum{
	L3G4200D_SCALE_SELECTION_250_DPS	= 0b00,	/* Default */
	L3G4200D_SCALE_SELECTION_500_DPS	= 0b01,
	L3G4200D_SCALE_SELECTION_2000_DPS	= 0b10, /* = 0b11 */

}l3g4200d_scale_t;

/*
 * Self Test
 * */
typedef enum{
	L3G4200D_SELF_TEST_NORMAL 	= 0b00,	/* Default */
	L3G4200D_SELF_TEST_0_PLUS	= 0b01,
	L3G4200D_SELF_TEST_1_MINUS	= 0b11
}l3g4200d_self_test_mode_t;

/*
 * SPI Serial Interface Mode
 * */
typedef enum{
	L3G4200D_SPI_MODE_4W = 0,	/* Default */
	L3G4200D_SPI_MODE_3W = 1,
}l3g4200d_spi_mode_t;



/*-------------------------------------------*/
/*----------------- structs -----------------*/
/*--------------------------------------------*/
/*
 * Vector for z, y and x axis.
 * */
typedef struct{
	float xAxis;
	float yAxis;
	float zAxis;
}l3g4200d_vector_t;




/*---------------------------------------------*/
/*----------------- variables -----------------*/
/*---------------------------------------------*/
HAL_StatusTypeDef l3g4200d_retSTDValue;
float l3g4200d_Sensitivity;
l3g4200d_vector_t zeroRateLevelVectorXYZL3g4200d;
l3g4200d_vector_t thresholdLevelVectorXYZL3g4200d;
l3g4200d_odr_bw_t l3g4200d_ODRBW;



/*----------------------------------------------*/
/*----------------- prototypes -----------------*/
/*----------------------------------------------*/


/* ----- CTRL_REG1 ----- */
bool setCtrlReg1(I2C_HandleTypeDef * 	ptHi2c,
		l3g4200d_odr_bw_t				odr_bw,
		bool							PM,		/* PowerDown/NormalSleep mode 	*/
		bool 							Zen,	/* Dis/En-able Z axis 			*/
		bool 							Yen,	/* Dis/En-able Y axis 			*/
		bool 							Xen		/* Dis/En-able X axis  			*/
		);

bool setQuickCtrlReg140050(I2C_HandleTypeDef * ptHi2);

/* ----- CTRL_REG2 ----- */
bool setCtrlReg2(I2C_HandleTypeDef * 		ptHi2c,
		l3g4200d_highPassFilterMode_t 		hpm,
		l3g4200d_highPassFilterCutOffFreq_t	hpcf);
bool setQuickCtrlReg2DisableHighPassFilter(I2C_HandleTypeDef * ptHi2);

/* ----- CTRL_REG3 ----- */
bool setQuickCtrlReg3DataReadyInt2(I2C_HandleTypeDef * ptHi2 , bool dataReadyInt2);

/* ----- CTRL_REG4 ----- */
bool setCtrlReg4(I2C_HandleTypeDef * 	ptHi2c,
		l3g4200d_block_data_update_t 	bdu,
		l3g4200d_ble_data_t 			ble,
		l3g4200d_scale_t 				scale,
		l3g4200d_self_test_mode_t 		selfTest,
		l3g4200d_spi_mode_t 			spiMode);
bool setQuickCtrlReg4Scale(I2C_HandleTypeDef * ptHi2c, l3g4200d_scale_t scale);

/* ----- CTRL_REG5 ----- */
bool setQuickCtrlReg5(I2C_HandleTypeDef * 	ptHi2c);



l3g4200d_scale_t getScaleL3g4200d(I2C_HandleTypeDef * ptHi2c);
l3g4200d_odr_bw_t getOdrBwL3g4200d(I2C_HandleTypeDef * ptHi2c);
uint8_t getTemperature(I2C_HandleTypeDef * ptHi2c);

l3g4200d_vector_t getRawVectorXYZL3g4200d(I2C_HandleTypeDef * ptHi2c);

void setZeroRateAndThresholdLevelVectorXYZL3g4200d(I2C_HandleTypeDef * ptHi2c, uint8_t samples);
bool setQuickZeroAndThresholdVectorXYZL3g4200d(I2C_HandleTypeDef * ptHi2c, bool getZeroRateLevelVector);

l3g4200d_vector_t getCompensatedVectorXYZL3g4200d(I2C_HandleTypeDef * ptHi2c);

bool setBeginNormalL3g4200d(I2C_HandleTypeDef * ptHi2c,
		bool				getZeroRateLevelVector,
		l3g4200d_odr_bw_t 	odr_bw,
		l3g4200d_scale_t 	scale,
		bool				Zen,
		bool				Yen,
		bool				Xen);
bool setBeginQuickL3g4200d(I2C_HandleTypeDef * ptHi2c);



/*---------------------------------- End of L3G4200D.h file ----------------------------------*/

#endif /* INC_L3G4200D_H_ */
