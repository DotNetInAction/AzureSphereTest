#include <errno.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <curl/curl.h>

// applibs_versions.h defines the API struct versions to use for applibs APIs.
#include "applibs_versions.h"

#include <applibs/log.h>
#include <applibs/i2c.h>

#include "mt3620_avnet_dev.h"
#include "deviceTwin.h"
#include "azure_iot_utilities.h"
#include "build_options.h"
#include "i2c.h"
#include "lsm6dso_reg.h"
#include "lps22hh_reg.h"

/* Private variables ---------------------------------------------------------*/
static axis3bit16_t data_raw_acceleration;
static axis3bit16_t data_raw_angular_rate;
static axis3bit16_t raw_angular_rate_calibration;
static axis1bit32_t data_raw_pressure;
static axis1bit16_t data_raw_temperature;
static float acceleration_mg[3];
static float angular_rate_dps[3];
static float lsm6dsoTemperature_degC;
static float pressure_hPa;
static float lps22hhTemperature_degC;

static uint8_t whoamI, rst;
static int accelTimerFd = -1;
const uint8_t lsm6dsOAddress = LSM6DSO_ADDRESS;     // Addr = 0x6A
lsm6dso_ctx_t dev_ctx;
lps22hh_ctx_t pressure_ctx;
CURL *curl;

//Extern variables
int i2cFd = -1;
extern int epollFd;
extern volatile sig_atomic_t terminationRequired;

//Private functions

// Routines to read/write to the LSM6DSO device
static int32_t platform_write(int *fD, uint8_t reg, uint8_t *bufp, uint16_t len);
static int32_t platform_read(int *fD, uint8_t reg, uint8_t *bufp, uint16_t len);

// Routines to read/write to the LPS22HH device connected to the LSM6DSO sensor hub
static int32_t lsm6dso_write_lps22hh_cx(void* ctx, uint8_t reg, uint8_t* data, uint16_t len);
static int32_t lsm6dso_read_lps22hh_cx(void* ctx, uint8_t reg, uint8_t* data, uint16_t len);

/// <summary>
///     Sleep for delayTime ms
/// </summary>
void HAL_Delay(int delayTime) {
	struct timespec ts;
	ts.tv_sec = 0;
	ts.tv_nsec = delayTime * 50000;
	nanosleep(&ts, NULL);
}

void reverse(char *str, int len)
{
	int i = 0, j = len - 1, temp;
	while (i < j)
	{
		temp = str[i];
		str[i] = str[j];
		str[j] = temp;
		i++; j--;
	}
}

int intToStr(int x, char str[], int d)
{
	int i = 0;
	while (x)
	{
		str[i++] = (x % 10) + '0';
		x = x / 10;
	}

	// If number of digits required is more, then 
	// add 0s at the beginning 
	while (i < d)
		str[i++] = '0';

	reverse(str, i);
	str[i] = '\0';
	return i;
}

void ftoa(float n, char *res, int afterpoint)
{
	// Extract integer part 
	int ipart = (int)n;

	// Extract floating part 
	float fpart = n - (float)ipart;

	// convert integer part to string 
	int i = intToStr(ipart, res, 0);

	// check for display option after point 
	if (afterpoint != 0)
	{
		res[i] = '.';  // add dot 

		// Get the value of fraction part upto given no. 
		// of points after dot. The third parameter is needed 
		// to handle cases like 233.007 
		fpart = fpart * pow(10, afterpoint);

		intToStr((int)fpart, res + i + 1, afterpoint);
	}
}


char* concat(const char *s1, const char *s2)
{
	char *result = malloc(strlen(s1) + strlen(s2) + 1); // +1 for the null-terminator
	// in real code you would check for errors in malloc here
	strcpy(result, s1);
	strcat(result, s2);
	return result;
}


/// <summary>
///     Print latest data from on-board sensors.
/// </summary>
void AccelTimerEventHandler(EventData *eventData)
{
	uint8_t reg;
	lps22hh_reg_t lps22hhReg;

#if (defined(IOT_CENTRAL_APPLICATION) || defined(IOT_HUB_APPLICATION))
	static bool firstPass = true;
#endif
	// Consume the event.  If we don't do this we'll come right back 
	// to process the same event again
	if (ConsumeTimerFdEvent(accelTimerFd) != 0) {
		terminationRequired = true;
		return;
	}

	// Read the sensors on the lsm6dso device

	lps22hh_read_reg(&pressure_ctx, LPS22HH_STATUS, (uint8_t *)&lps22hhReg, 1);

	//Read output only if new value is available

	if ((lps22hhReg.status.p_da == 1) && (lps22hhReg.status.t_da == 1))
	{
		memset(data_raw_pressure.u8bit, 0x00, sizeof(int32_t));
		lps22hh_pressure_raw_get(&pressure_ctx, data_raw_pressure.u8bit);
	
		pressure_hPa = lps22hh_from_lsb_to_hpa(data_raw_pressure.i32bit);

		memset(data_raw_temperature.u8bit, 0x00, sizeof(int16_t));
		lps22hh_temperature_raw_get(&pressure_ctx, data_raw_temperature.u8bit);
		lps22hhTemperature_degC = lps22hh_from_lsb_to_celsius(data_raw_temperature.i16bit);

		curl = curl_easy_init();
		if (curl) {

			char Output[7];
			
			ftoa(pressure_hPa, Output, 1);

			Log_Debug(Output);
			Log_Debug("\r\n");

			char* s = concat("http://192.168.1.11:3005?room=main&sensor=preassure&temp=", Output);

			//Log_Debug(s);

			curl_easy_setopt(curl, CURLOPT_URL, s);

			curl_easy_setopt(curl, CURLOPT_HTTPGET, 1L);

			/* Perform the request */
			curl_easy_perform(curl);

			curl_easy_cleanup(curl);

			free(s);
		}

		//Log_Debug("LPS22HH: Pressure     [hPa] : %.2f\r\n", pressure_hPa);		
	}

#if (defined(IOT_CENTRAL_APPLICATION) || defined(IOT_HUB_APPLICATION))

	// We've seen that the first read of the Accelerometer data is garbage.  If this is the first pass
	// reading data, don't report it to Azure.  Since we're graphing data in Azure, this data point
	// will skew the data.
	if (!firstPass) {

		// Allocate memory for a telemetry message to Azure
		char *pjsonBuffer = (char *)malloc(JSON_BUFFER_SIZE);
		if (pjsonBuffer == NULL) {
			Log_Debug("ERROR: not enough memory to send telemetry");
		}

		// construct the telemetry message
		snprintf(pjsonBuffer, JSON_BUFFER_SIZE, "{\"gX\":\"%.4lf\", \"gY\":\"%.4lf\", \"gZ\":\"%.4lf\", \"pressure\": \"%.2f\", \"aX\": \"%4.2f\", \"aY\": \"%4.2f\", \"aZ\": \"%4.2f\"}",
			acceleration_mg[0], acceleration_mg[1], acceleration_mg[2], pressure_hPa, angular_rate_dps[0], angular_rate_dps[1], angular_rate_dps[2]);

		Log_Debug("\n[Info] Sending telemetry: %s\n", pjsonBuffer);
		AzureIoT_SendMessage(pjsonBuffer);
		free(pjsonBuffer);

}

	firstPass = false;

#endif 

}

/// <summary>
///     Initializes the I2C interface.
/// </summary>
/// <returns>0 on success, or -1 on failure</returns>
int initI2c(void) {

	// Begin MT3620 I2C init 

	i2cFd = I2CMaster_Open(MT3620_RDB_HEADER4_ISU2_I2C);
	if (i2cFd < 0) {
		Log_Debug("ERROR: I2CMaster_Open: errno=%d (%s)\n", errno, strerror(errno));
		return -1;
	}

	int result = I2CMaster_SetBusSpeed(i2cFd, I2C_BUS_SPEED_STANDARD);
	if (result != 0) {
		Log_Debug("ERROR: I2CMaster_SetBusSpeed: errno=%d (%s)\n", errno, strerror(errno));
		return -1;
	}

	result = I2CMaster_SetTimeout(i2cFd, 100);
	if (result != 0) {
		Log_Debug("ERROR: I2CMaster_SetTimeout: errno=%d (%s)\n", errno, strerror(errno));
		return -1;
	}

	// Start lsm6dso specific init

	// Initialize lsm6dso mems driver interface
	dev_ctx.write_reg = platform_write;
	dev_ctx.read_reg = platform_read;
	dev_ctx.handle = &i2cFd;

	// Check device ID
	lsm6dso_device_id_get(&dev_ctx, &whoamI);
	if (whoamI != LSM6DSO_ID) {
		Log_Debug("LSM6DSO not found!\n");
		return -1;
	}
	else {
		Log_Debug("LSM6DSO Found!\n");
	}
		
	 // Restore default configuration
	lsm6dso_reset_set(&dev_ctx, PROPERTY_ENABLE);
	do {
		lsm6dso_reset_get(&dev_ctx, &rst);
	} while (rst);

	 // Disable I3C interface
	lsm6dso_i3c_disable_set(&dev_ctx, LSM6DSO_I3C_DISABLE);

	// Enable Block Data Update
	lsm6dso_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

	 // Set Output Data Rate
	lsm6dso_xl_data_rate_set(&dev_ctx, LSM6DSO_XL_ODR_12Hz5);
	lsm6dso_gy_data_rate_set(&dev_ctx, LSM6DSO_GY_ODR_12Hz5);

	 // Set full scale
	lsm6dso_xl_full_scale_set(&dev_ctx, LSM6DSO_4g);
	lsm6dso_gy_full_scale_set(&dev_ctx, LSM6DSO_2000dps);

	 // Configure filtering chain(No aux interface)
	// Accelerometer - LPF1 + LPF2 path	
	lsm6dso_xl_hp_path_on_out_set(&dev_ctx, LSM6DSO_LP_ODR_DIV_100);
	lsm6dso_xl_filter_lp2_set(&dev_ctx, PROPERTY_ENABLE);

	// lps22hh specific init

	// Initialize lps22hh mems driver interface
	pressure_ctx.read_reg = lsm6dso_read_lps22hh_cx;
	pressure_ctx.write_reg = lsm6dso_write_lps22hh_cx;
	pressure_ctx.handle = &i2cFd;

	bool lps22hhDetected = false;
	int failCount = 10;

	while (!lps22hhDetected) {
		
		// Enable pull up on master I2C interface.
		lsm6dso_sh_pin_mode_set(&dev_ctx, LSM6DSO_INTERNAL_PULL_UP);

		// Check if LPS22HH is connected to Sensor Hub
		lps22hh_device_id_get(&pressure_ctx, &whoamI);
		if (whoamI != LPS22HH_ID) {
			Log_Debug("LPS22HH not found!\n");
		}
		else {
			lps22hhDetected = true;
			Log_Debug("LPS22HH Found!\n");
		}

		// Restore the default configuration
		lps22hh_reset_set(&pressure_ctx, PROPERTY_ENABLE);
		do {
			lps22hh_reset_get(&pressure_ctx, &rst);
		} while (rst);

		// Enable Block Data Update
		lps22hh_block_data_update_set(&pressure_ctx, PROPERTY_ENABLE);

		//Set Output Data Rate
		lps22hh_data_rate_set(&pressure_ctx, LPS22HH_10_Hz_LOW_NOISE);

		// If we failed to detect the lps22hh device, then pause before trying again.
		if (!lps22hhDetected) {
			HAL_Delay(100);
		}

		if (failCount-- == 0) {
			Log_Debug("Failed to read LSM22HH device ID, exiting\n");
			return -1;
		}
	}

	// Read the raw angular rate data from the device to use as offsets.  We're making the assumption that the device
	// is stationary.

	uint8_t reg;

	
	Log_Debug("LSM6DSO: Calibrating angular rate . . .\n"); 
	Log_Debug("LSM6DSO: Please make sure the device is stationary.\n");

	do {
		// Read the calibration values
		lsm6dso_gy_flag_data_ready_get(&dev_ctx, &reg);
		if (reg)
		{
			// Read angular rate field data to use for calibration offsets
			memset(data_raw_angular_rate.u8bit, 0x00, 3 * sizeof(int16_t));
			lsm6dso_angular_rate_raw_get(&dev_ctx, raw_angular_rate_calibration.u8bit);
		}

		// Read the angular data rate again and verify that after applying the calibration, we have 0 angular rate in all directions

		lsm6dso_gy_flag_data_ready_get(&dev_ctx, &reg);
		if (reg)
		{
			// Read angular rate field data
			memset(data_raw_angular_rate.u8bit, 0x00, 3 * sizeof(int16_t));
			lsm6dso_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate.u8bit);

			// Before we store the mdps values subtract the calibration data we captured at startup.
			angular_rate_dps[0] = lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[0] - raw_angular_rate_calibration.i16bit[0]);
			angular_rate_dps[1] = lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[1] - raw_angular_rate_calibration.i16bit[1]);
			angular_rate_dps[2] = lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[2] - raw_angular_rate_calibration.i16bit[2]);
		}

	// If the angular values after applying the offset are not zero, then do it again!
	} while (angular_rate_dps[0] == angular_rate_dps[1] == angular_rate_dps[2] == 0.0);

	Log_Debug("LSM6DSO: Calibrating angular rate complete!\n");


	// Init the epoll interface to periodically run the AccelTimerEventHandler routine where we read the sensors

	// Define the period in the build_options.h file
	struct timespec accelReadPeriod = { .tv_sec = ACCEL_READ_PERIOD_SECONDS,.tv_nsec = ACCEL_READ_PERIOD_NANO_SECONDS };
	// event handler data structures. Only the event handler field needs to be populated.
	static EventData accelEventData = { .eventHandler = &AccelTimerEventHandler };
	accelTimerFd = CreateTimerFdAndAddToEpoll(epollFd, &accelReadPeriod, &accelEventData, EPOLLIN);
	if (accelTimerFd < 0) {
		return -1;
	}

	return 0;
}

/// <summary>
///     Closes the I2C interface File Descriptors.
/// </summary>
void closeI2c(void) {

	CloseFdAndPrintError(i2cFd, "i2c");
	CloseFdAndPrintError(accelTimerFd, "accelTimer");
}

/// <summary>
///     Writes data to the lsm6dso i2c device
/// </summary>
/// <returns>0</returns>

static int32_t platform_write(int *fD, uint8_t reg, uint8_t *bufp,
	uint16_t len)
{

#ifdef ENABLE_READ_WRITE_DEBUG
	Log_Debug("platform_write()\n");
	Log_Debug("reg: %0x\n", reg);
	Log_Debug("len: %0x\n", len);
	Log_Debug("bufp contents: ");
	for (int i = 0; i < len; i++) {

		Log_Debug("%0x: ", bufp[i]);
	}
	Log_Debug("\n");
#endif 

	// Construct a new command buffer that contains the register to write to, then the data to write
	uint8_t cmdBuffer[len + 1];
	cmdBuffer[0] = reg;
	for (int i = 0; i < len; i++) {
		cmdBuffer[i + 1] = bufp[i];
	}

#ifdef ENABLE_READ_WRITE_DEBUG
	Log_Debug("cmdBuffer contents: ");
	for (int i = 0; i < len + 1; i++) {

		Log_Debug("%0x: ", cmdBuffer[i]);
	}
	Log_Debug("\n");
#endif

	// Write the data to the device
	int32_t retVal = I2CMaster_Write(*fD, lsm6dsOAddress, cmdBuffer, (size_t)len + 1);
	if (retVal < 0) {
		Log_Debug("ERROR: platform_write: errno=%d (%s)\n", errno, strerror(errno));
		return -1;
	}
#ifdef ENABLE_READ_WRITE_DEBUG
	Log_Debug("Wrote %d bytes to device.\n\n", retVal);
#endif
	return 0;
}

/// <summary>
///     Reads generic device register from the i2c interface
/// </summary>
/// <returns>0</returns>

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(int *fD, uint8_t reg, uint8_t *bufp,
	uint16_t len)
{

#ifdef ENABLE_READ_WRITE_DEBUG
	Log_Debug("platform_read()\n");
	Log_Debug("reg: %0x\n", reg);
	Log_Debug("len: %d\n", len);
;
#endif

	// Set the register address to read
	int32_t retVal = I2CMaster_Write(i2cFd, lsm6dsOAddress, &reg, 1);
	if (retVal < 0) {
		Log_Debug("ERROR: platform_read(write step): errno=%d (%s)\n", errno, strerror(errno));
		return -1;
	}

	// Read the data into the provided buffer
	retVal = I2CMaster_Read(i2cFd, lsm6dsOAddress, bufp, len);
	if (retVal < 0) {
		Log_Debug("ERROR: platform_read(read step): errno=%d (%s)\n", errno, strerror(errno));
		return -1;
	}

#ifdef ENABLE_READ_WRITE_DEBUG
	Log_Debug("Read returned: ");
	for (int i = 0; i < len; i++) {
		Log_Debug("%0x: ", bufp[i]);
	}
	Log_Debug("\n\n");
#endif 	   

	return 0;
}
/*
 * @brief  Write lsm2mdl device register (used by configuration functions)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t lsm6dso_write_lps22hh_cx(void* ctx, uint8_t reg, uint8_t* data,
	uint16_t len)
{
	axis3bit16_t data_raw_acceleration;
	int32_t ret;
	uint8_t drdy;
	lsm6dso_status_master_t master_status;
	lsm6dso_sh_cfg_write_t sh_cfg_write;

	// Configure Sensor Hub to write to the LPS22HH, and send the write data
	sh_cfg_write.slv0_add = (LPS22HH_I2C_ADD_L & 0xFEU) >> 1; // 7bit I2C address
	sh_cfg_write.slv0_subadd = reg,
	sh_cfg_write.slv0_data = *data,
	ret = lsm6dso_sh_cfg_write(&dev_ctx, &sh_cfg_write);

	/* Disable accelerometer. */
	lsm6dso_xl_data_rate_set(&dev_ctx, LSM6DSO_XL_ODR_OFF);

	/* Enable I2C Master. */
	lsm6dso_sh_master_set(&dev_ctx, PROPERTY_ENABLE);

	/* Enable accelerometer to trigger Sensor Hub operation. */
	lsm6dso_xl_data_rate_set(&dev_ctx, LSM6DSO_XL_ODR_104Hz);

	/* Wait Sensor Hub operation flag set. */
	lsm6dso_acceleration_raw_get(&dev_ctx, data_raw_acceleration.u8bit);
	do
	{
		HAL_Delay(20);
		lsm6dso_xl_flag_data_ready_get(&dev_ctx, &drdy);
	} while (!drdy);

	do
	{
		HAL_Delay(20);
		lsm6dso_sh_status_get(&dev_ctx, &master_status);
	} while (!master_status.sens_hub_endop);

	/* Disable I2C master and XL (trigger). */
	lsm6dso_sh_master_set(&dev_ctx, PROPERTY_DISABLE);
	lsm6dso_xl_data_rate_set(&dev_ctx, LSM6DSO_XL_ODR_OFF);

	return ret;
}

/*
 * @brief  Read lsm2mdl device register (used by configuration functions)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t lsm6dso_read_lps22hh_cx(void* ctx, uint8_t reg, uint8_t* data, uint16_t len)
{
	lsm6dso_sh_cfg_read_t sh_cfg_read;
	axis3bit16_t data_raw_acceleration;
	int32_t ret;
	uint8_t drdy;
	lsm6dso_status_master_t master_status;

	/* Disable accelerometer. */
	lsm6dso_xl_data_rate_set(&dev_ctx, LSM6DSO_XL_ODR_OFF);
	
	// For each byte we need to read from the lps22hh
	for (int i = 0; i < len; i++) {

		/* Configure Sensor Hub to read LPS22HH. */
		sh_cfg_read.slv_add = (LPS22HH_I2C_ADD_L &0xFEU) >> 1; /* 7bit I2C address */
		sh_cfg_read.slv_subadd = reg+i;
		sh_cfg_read.slv_len = 1;

		// Call the command to read the data from the sensor hub.
		// This data will be read from the device connected to the 
		// sensor hub, and saved into a register for us to read.
		ret = lsm6dso_sh_slv0_cfg_read(&dev_ctx, &sh_cfg_read);
		lsm6dso_sh_slave_connected_set(&dev_ctx, LSM6DSO_SLV_0);

		/* Enable I2C Master and I2C master. */
		lsm6dso_sh_master_set(&dev_ctx, PROPERTY_ENABLE);

		/* Enable accelerometer to trigger Sensor Hub operation. */
		lsm6dso_xl_data_rate_set(&dev_ctx, LSM6DSO_XL_ODR_104Hz);

		/* Wait Sensor Hub operation flag set. */
		lsm6dso_acceleration_raw_get(&dev_ctx, data_raw_acceleration.u8bit);
		do {
			HAL_Delay(20);
			lsm6dso_xl_flag_data_ready_get(&dev_ctx, &drdy);
		} while (!drdy);

		do {
			HAL_Delay(20);
			lsm6dso_sh_status_get(&dev_ctx, &master_status);
		} while (!master_status.sens_hub_endop);

		/* Disable I2C master and XL(trigger). */
		lsm6dso_sh_master_set(&dev_ctx, PROPERTY_DISABLE);
		lsm6dso_xl_data_rate_set(&dev_ctx, LSM6DSO_XL_ODR_OFF);

		// Read the data from the device.  The call below reads
		// all 18 sensor hub data.  We just need the data from 
		// sensor hub 1, so copy that into our data array.
		uint8_t buffer[18];
		lsm6dso_sh_read_data_raw_get(&dev_ctx, buffer);
		data[i] = buffer[1];

#ifdef ENABLE_READ_WRITE_DEBUG
		Log_Debug("Read %d bytes: ", len);
		for (int i = 0; i < len; i++) {
			Log_Debug("[%0x] ", data[i]);
		}
		Log_Debug("\n", len);
#endif 
	}

	/* Re-enable accelerometer */
	lsm6dso_xl_data_rate_set(&dev_ctx, LSM6DSO_XL_ODR_104Hz);

	return ret;
}

