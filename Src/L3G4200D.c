/*
 * L3G4200D.c
 *
 *  Created in: May, 2021
 *      Author: Lincoln de León
 */


#include "L3G4200D.h"


/**
  * @brief  Writes 8 bits in an specified register address
  * @param  ptHi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C
  * @param  regAddress Register address
  * @param  value Value to be written
  * @retval bool type
  */
static bool writeRegister8bit(I2C_HandleTypeDef * ptHi2c, uint8_t regAddress, uint8_t value){
	l3g4200d_retSTDValue = HAL_I2C_Mem_Write(ptHi2c, L3G4200D_ADDRESS, regAddress, 1, &value, 1, MY_I2C_DELAY);
	if(l3g4200d_retSTDValue == HAL_OK){
		return true;
	}
	return false;
}


/**
  * @brief  reads 8 bits from an specified register address
  * @param 	ptHi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C
  * @param  regAddress Address to read from
  * @retval read value as uint8_t
  */
static uint8_t readRegister8bit(I2C_HandleTypeDef * ptHi2c, uint8_t regAddress){
	uint8_t value = 0x00;
	l3g4200d_retSTDValue = HAL_I2C_Mem_Read(ptHi2c, L3G4200D_ADDRESS, regAddress, 1, &value, 1, MY_I2C_DELAY);
	return value;
}





/* ----- CTRL_REG1 ----- */

/**
  * @brief  sets the CTRL_REG1
  * @param  ptHi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C
  * @param  odr_bw Output Data Rate selection (4 bits) l3g4200d_odr_bw_t type
  * @param  PM Power down mode
  * @param  Zen Z axis enable
  * @param  Yen Y axis enable
  * @param  Xen X axis enable
  * @retval bool type
  */
bool setCtrlReg1(I2C_HandleTypeDef * 	ptHi2c,
		l3g4200d_odr_bw_t				odr_bw,
		bool							PM,		/* PowerDown/NormalSleep mode 	*/
		bool 							Zen,	/* Dis/En-able Z axis 			*/
		bool 							Yen,	/* Dis/En-able Y axis 			*/
		bool 							Xen		/* Dis/En-able X axis  			*/
		){

	uint8_t myReg1 = 0x00;
	myReg1 |= ((uint8_t) Xen);			/* bit 0 Dis/En-able X axis 				*/
	myReg1 |= ((uint8_t) Yen) << 1;		/* bit 1 Dis/En-able Y axis 				*/
	myReg1 |= ((uint8_t) Zen) << 2;		/* bit 2 Dis/En-able Z axis 		  		*/
	myReg1 |= ((uint8_t) PM)  << 3;		/* bit 3 PowerDown/NormalSleep mode 		*/
	myReg1 |= ((uint8_t) odr_bw)  << 4; /* bit 7.4 OutputDataRate(2b) and BWcutOff 	*/

	return writeRegister8bit(ptHi2c, L3G4200D_CTRL_REG1_RW, myReg1);
}

/**
  * @brief  sets the CTRL_REG1 in a quick way
  * @param  ptHi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C
  * @retval bool type
  */
bool setQuickCtrlReg140050(I2C_HandleTypeDef * ptHi2){
	return setCtrlReg1(ptHi2, L3G4200D_ODR_400_BW_50, true, true, true, true);
}



/* ----- CTRL_REG2 ----- */

/**
  * @brief  sets the CTRL_REG2
  * @param  ptHi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C
  * @param  hpm High pass filter mode (2 bits)
  * @param  hpcf High pass filter cut off frecuency
  * @retval type
  */
bool setCtrlReg2(I2C_HandleTypeDef * 		ptHi2c,
		l3g4200d_highPassFilterMode_t 		hpm,
		l3g4200d_highPassFilterCutOffFreq_t	hpcf){

	return writeRegister8bit(ptHi2c, L3G4200D_CTRL_REG2_RW, (((uint8_t) hpm) << 4) | ((uint8_t) hpcf) );
}

/**
  * @brief  sets the CTRL_REG2 in a quick way
  * @param  ptHi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval bool type
  */
bool setQuickCtrlReg2DisableHighPassFilter(I2C_HandleTypeDef * ptHi2){
	return setCtrlReg2(ptHi2, L3G4200D_HPM_NORMAL_MODE_00_DEFAULT, L3G4200D_HPCF_0000);
}



/* ----- CTRL_REG3 ----- */

/**
  * @brief  sets the CTRL_REG3 in a quick way
  * @param  ptHi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C
  * @retval bool type
  */
bool setQuickCtrlReg3DataReadyInt2(I2C_HandleTypeDef * ptHi2 , bool dataReadyInt2){
	return writeRegister8bit(ptHi2, L3G4200D_CTRL_REG3_RW, ((uint8_t) dataReadyInt2) << 3);
}


/* ----- CTRL_REG4 ----- */

/**
  * @brief  sets the CTRL_REG4
  * @param  ptHi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C
  * @param  bdu Block Data Update.
  * @param  ble Big/Little Endian Data Selection
  * @param  scale Full Scale selection
  * @param  selfTest Self Test Enable
  * @param  SPI Serial Interface Mode selection
  * @retval type
  */
bool setCtrlReg4(I2C_HandleTypeDef * 	ptHi2c,
		l3g4200d_block_data_update_t 	bdu,
		l3g4200d_ble_data_t 			ble,
		l3g4200d_scale_t 				scale,
		l3g4200d_self_test_mode_t 		selfTest,
		l3g4200d_spi_mode_t 			spiMode){

	uint8_t myReg1 = 0x00;
	myReg1 |= ((uint8_t) spiMode);			/*bit  0			*/
	myReg1 |= ((uint8_t) selfTest << 1);	/*bits 2.1 			*/
											/*bit  3    (empty)	*/
	myReg1 |= ((uint8_t) scale << 4);		/*bits 5.4. 		*/
	myReg1 |= ((uint8_t) ble << 6);			/*bit  6			*/
	myReg1 |= ((uint8_t) bdu << 7);			/*bit  7			*/

	return writeRegister8bit(ptHi2c, L3G4200D_CTRL_REG4_RW, myReg1);

}

/**
  * @brief  sets the CTRL_REG4 in a quick way but the scale must be specified
  * @param  ptHi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C
  * @param  scale  Full Scale selection
  * @retval bool type
  */
bool setQuickCtrlReg4Scale(I2C_HandleTypeDef * ptHi2c, l3g4200d_scale_t scale){
	return setCtrlReg4(ptHi2c, L3G4200D_BDU_UPDATED_UNTIL_READING, L3G4200D_BLE_LSB_LOWER_ADDRESS, scale,
			L3G4200D_SELF_TEST_NORMAL, L3G4200D_SPI_MODE_4W);
}



/* ----- CTRL_REG5 ----- */

/**
  * @brief  sets the CTRL_REG5 in a quick way
  * @param  ptHi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C
  * @retval bool type
  */
bool setQuickCtrlReg5(I2C_HandleTypeDef * 	ptHi2c){
	/*	Reboot memory content 	= Normal mode
	 * 	FIFO					= disable
	 * 	High Pass filter		= disabled
	 * 	INT1 selection			= 0
	 * 	Out selection configuration	= 0 */
	return writeRegister8bit(ptHi2c, L3G4200D_CTRL_REG5_RW, 0x00);
}




/**
  * @brief  gives the current value of scale y the gyroscope
  * @param  ptHi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C
  * @retval l3g4200d_scale_t
  */
l3g4200d_scale_t getScaleL3g4200d(I2C_HandleTypeDef * ptHi2c){
	return (l3g4200d_scale_t) ((readRegister8bit(ptHi2c, L3G4200D_CTRL_REG4_RW) >> 4) & 0x03);
}

/**
  * @brief  gives the Output Data Rate selection
  * @param  ptHi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C
  * @param
  * @retval l3g4200d_odr_bw_t
  */
l3g4200d_odr_bw_t getOdrBwL3g4200d(I2C_HandleTypeDef * ptHi2c){
	return (l3g4200d_odr_bw_t) ((readRegister8bit(ptHi2c, L3G4200D_CTRL_REG1_RW) >> 4) & 0x0F);
}

/**
  * @brief  give the value in the temperature register
  * @param  ptHi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C
  * @param
  * @retval uint8_t
  */
uint8_t getTemperature(I2C_HandleTypeDef * ptHi2c){
	return readRegister8bit(ptHi2c, L3G4200D_OUT_TEMP_R);
}



/**
  * @brief  gives a vector with the current angular rate data for the three axes
  * 		the values in the structure are float type values
  * @param  ptHi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C
  * @retval l3g4200d_vector_t
  */
l3g4200d_vector_t getRawVectorXYZL3g4200d(I2C_HandleTypeDef * ptHi2c){
	uint8_t xL;
	uint8_t xH;

	uint8_t yL;
	uint8_t yH;

	uint8_t zL;
	uint8_t zH;

	//bool result;
	//result = setCtrlReg1(ptHi2c, l3g4200d_ODRBW, false, true, true, true);

	/* Angular rate data rate for each axis is read */
	xH = readRegister8bit(ptHi2c, L3G4200D_OUT_X_H_R);
	xL = readRegister8bit(ptHi2c, L3G4200D_OUT_X_L_R);


	yH = readRegister8bit(ptHi2c, L3G4200D_OUT_Y_H_R);
	yL = readRegister8bit(ptHi2c, L3G4200D_OUT_Y_L_R);


	zH = readRegister8bit(ptHi2c, L3G4200D_OUT_Z_H_R);
	zL = readRegister8bit(ptHi2c, L3G4200D_OUT_Z_L_R);

	//result = setCtrlReg1(ptHi2c, l3g4200d_ODRBW, true, true, true, true);

	uint16_t xComp = 0;
	uint16_t yComp = 0;
	uint16_t zComp = 0;


	/* Angular rate data rate for each axis represented as uint16_t */
	xComp = (((uint16_t) xH) << 8) | ((uint16_t) xL);
	yComp = (((uint16_t) yH) << 8) | ((uint16_t) yL);
	zComp = (((uint16_t) zH) << 8) | ((uint16_t) zL);


	l3g4200d_vector_t myVector;		/* Vector to be returned by the function */


	/*---------------------- Two's complement is obtained ----------------------*/
	if (xComp > 0x7FFF){							/* X component */
		xComp = (~xComp) + 1;
		myVector.xAxis = 0.0 - ((float) xComp );
	}else{
		myVector.xAxis = (float) xComp;
	}

	if (yComp > 0x7FFF){							/* Y component */
		yComp = (~yComp) + 1;
		myVector.yAxis = 0.0 - ((float) yComp );
	}else{
		myVector.yAxis = (float) yComp;
	}

	if (zComp > 0x7FFF){							/* Z component */
		zComp = (~zComp) + 1;
		myVector.zAxis = 0.0 - ((float) zComp);
	}else{
		myVector.zAxis = (float) zComp;
	}

	return myVector;
}


/**
  * @brief  gives a zero rate level vector and sets a threshold level vector
  * @param  ptHi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C
  * @param	samples Specify the amount of samples to take
  * @retval N/A
  */
void setZeroRateAndThresholdLevelVectorXYZL3g4200d(I2C_HandleTypeDef * ptHi2c, uint8_t samples){
	float xZero = 0.0;
	float yZero = 0.0;
	float zZero = 0.0;

	float sumSquaredX = 0.0;
	float sumSquaredY = 0.0;
	float sumSquaredZ = 0.0;


	l3g4200d_vector_t zeroRateLevelVector;

	for(uint8_t i = 0; i < samples; i++){
		zeroRateLevelVector = getRawVectorXYZL3g4200d(ptHi2c);
		xZero += zeroRateLevelVector.xAxis;
		yZero += zeroRateLevelVector.yAxis;
		zZero += zeroRateLevelVector.zAxis;

		sumSquaredX += (zeroRateLevelVector.xAxis * zeroRateLevelVector.xAxis);
		sumSquaredY += (zeroRateLevelVector.yAxis * zeroRateLevelVector.yAxis);
		sumSquaredZ += (zeroRateLevelVector.zAxis * zeroRateLevelVector.zAxis);
	}

	zeroRateLevelVector.xAxis = xZero / samples;
	zeroRateLevelVector.yAxis = yZero / samples;
	zeroRateLevelVector.zAxis = zZero / samples;

	/* zero rate level vector is stored */
	zeroRateLevelVectorXYZL3g4200d.xAxis = zeroRateLevelVector.xAxis;
	zeroRateLevelVectorXYZL3g4200d.yAxis = zeroRateLevelVector.yAxis;
	zeroRateLevelVectorXYZL3g4200d.zAxis = zeroRateLevelVector.zAxis;

	/* threshold level vector is stored */
	thresholdLevelVectorXYZL3g4200d.xAxis = MUL_STDDEV * sqrt(fabs(
			(sumSquaredX / samples) - (zeroRateLevelVector.xAxis * zeroRateLevelVector.xAxis) 	));

	thresholdLevelVectorXYZL3g4200d.yAxis = MUL_STDDEV * sqrt(fabs(
			(sumSquaredY / samples) - (zeroRateLevelVector.yAxis * zeroRateLevelVector.yAxis)	));

	thresholdLevelVectorXYZL3g4200d.zAxis = MUL_STDDEV * sqrt(fabs(
			(sumSquaredZ / samples) - (zeroRateLevelVector.zAxis * zeroRateLevelVector.zAxis)	));
}


/**
  * @brief  sets the zero rate level vector and the threshold level vector
  * @param  ptHi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C
  * @param	getZeroRateLevelVector
  * @retval booltype
  */
bool setQuickZeroAndThresholdVectorXYZL3g4200d(I2C_HandleTypeDef * ptHi2c, bool getZeroRateLevelVector){
	bool output = false;

	if (getZeroRateLevelVector){
		setZeroRateAndThresholdLevelVectorXYZL3g4200d(ptHi2c, SAMPLES_FOR_THRESHOLD);
		output = true;
	}else{
		zeroRateLevelVectorXYZL3g4200d.xAxis = 0.0;
		zeroRateLevelVectorXYZL3g4200d.yAxis = 0.0;
		zeroRateLevelVectorXYZL3g4200d.zAxis = 0.0;

		thresholdLevelVectorXYZL3g4200d.xAxis = 0.0;
		thresholdLevelVectorXYZL3g4200d.yAxis = 0.0;
		thresholdLevelVectorXYZL3g4200d.zAxis = 0.0;
		output = true;
	}

	return output;
}


/**
  * @brief	gives the compensated vector. If the absolute value of the difference
  * 		of the raw vector and the zero rate level vector components are less than
  * 		the threshold level vector 0.0 is returned in every component.
  *
  * @param  ptHi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C
  * @param
  * @retval l3g4200d_vector_t
  */
l3g4200d_vector_t getCompensatedVectorXYZL3g4200d(I2C_HandleTypeDef * ptHi2c){
	l3g4200d_vector_t rawVect;
	l3g4200d_vector_t compVect;

	rawVect = getRawVectorXYZL3g4200d(ptHi2c);

	compVect.xAxis = rawVect.xAxis - zeroRateLevelVectorXYZL3g4200d.xAxis;
	compVect.yAxis = rawVect.yAxis - zeroRateLevelVectorXYZL3g4200d.yAxis;
	compVect.zAxis = rawVect.zAxis - zeroRateLevelVectorXYZL3g4200d.zAxis;

	if(fabs(compVect.xAxis) < thresholdLevelVectorXYZL3g4200d.xAxis){
		compVect.xAxis = 0;
	}
	if(fabs(compVect.yAxis) < thresholdLevelVectorXYZL3g4200d.yAxis){
		compVect.yAxis = 0;
	}
	if(fabs(compVect.zAxis) < thresholdLevelVectorXYZL3g4200d.zAxis){
		compVect.zAxis = 0;
	}

	return compVect;
}


/**
  * @brief  configures the L3G4200D to begin
  * @param  ptHi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C
  * @param  getZeroRateLevelVector establishes if sample are taken to get zero and threshold
  * @param  odr_bw Output Data Rate selection
  * @param  scale Full Scale selection
  * @param  Zen Z axis
  * @param  Yen Y axis
  * @param  Xen Z axis
  * @retval bool type
  */
bool setBeginNormalL3g4200d(I2C_HandleTypeDef * ptHi2c,
		bool				getZeroRateLevelVector,
		l3g4200d_odr_bw_t 	odr_bw,
		l3g4200d_scale_t 	scale,
		bool				Zen,
		bool				Yen,
		bool				Xen){

	bool result = true;
	l3g4200d_ODRBW = odr_bw;

	if (readRegister8bit(ptHi2c, L3G4200D_WHO_AM_I_R) != 0xD3){  	/*  0b11010011  */
		return false;
	}

	result &= setCtrlReg1(ptHi2c, l3g4200d_ODRBW, true, true, true, true); 	/*Reg1*/
	result &= setQuickCtrlReg2DisableHighPassFilter(ptHi2c);		/*Reg2*/
	result &= setQuickCtrlReg3DataReadyInt2(ptHi2c, false);   		/*Reg3*/ /* disabled Date Ready on DRDY/INT2 */
	result &= setQuickCtrlReg4Scale(ptHi2c, scale);					/*Reg4*/
	result &= setQuickCtrlReg5(ptHi2c);								/*Reg5*/
	result &= writeRegister8bit(ptHi2c, L3G4200D_FIFO_CTRL_REG_RW, 0x40);	/* Stream mode Enabled */


	/* From Datasheet 'L3G4200D' Table 4. 'Mechanical characteristics' */
	switch(scale){
	case L3G4200D_SCALE_SELECTION_250_DPS:		/* Scale =  250 dps  -->>  8.75 mdps/digit	*/
		l3g4200d_Sensitivity = 0.00875;
		break;
	case L3G4200D_SCALE_SELECTION_500_DPS:		/* Scale =  500 dps  -->>  17.50 mdps/digit	*/
		l3g4200d_Sensitivity = 0.01750;
		break;
	case L3G4200D_SCALE_SELECTION_2000_DPS:		/* Scale = 2000 dps  -->>  70 mdps/digit	*/
		l3g4200d_Sensitivity = 0.07000;
		break;
	default:
		l3g4200d_Sensitivity = 0.0;
		break;
	}

	result &= setQuickZeroAndThresholdVectorXYZL3g4200d(ptHi2c, getZeroRateLevelVector);

	return result;
}


/**
  * @brief  sets the L3G4200D Gyroscope in a quick way
  * 		a zero rate level vector is obtained in the beginning and a threshold of MUL_STDDEV times
  * 		the standard deviation is set, an output data rate of 800 Hz and 110 cut-off frequency
  * 		the scale is set to 2000 degrees per second (dps) and the sensitivity is 0.07. The three axis are enabled.
  * @param  ptHi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C
  * @retval bool type
  */
bool setBeginQuickL3g4200d(I2C_HandleTypeDef * ptHi2c){
	return setBeginNormalL3g4200d(ptHi2c, true, L3G4200D_ODR_800_BW_110, L3G4200D_SCALE_SELECTION_2000_DPS,
			true, true, true);

}





/*---------------------------------- End of L3G4200D.c file ----------------------------------*/



