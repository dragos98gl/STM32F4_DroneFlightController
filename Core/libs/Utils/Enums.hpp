
#ifndef LIBS_ENUMS_HPP_
#define LIBS_ENUMS_HPP_

enum EnumSensorsInterrupt : uint32_t
{
	ICM42688P_t = (1<<0),
	BMP390_t = (1<<1),
	LIS3MDLTR_t = (1<<2),
	PMW_t = (1<<3),
	REMOTERX_t = (1<<4),
	SONAR_t = (1<<5),
	VL53L0X_t = (1<<6)
};

enum class FaultsStatus
{
	OKAY,
	NOT_READY,
	FAILURE,
	CRITICAL
};

#endif /* LIBS_ENUMS_HPP_ */
