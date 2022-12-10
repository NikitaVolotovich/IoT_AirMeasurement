
struct Registry_t {

    char		ID_product[32];			//! Product ID
    char		ID_vendor[32];			//! Vendor ID
    char		ID_version[32];			//! Version ID
    char		ID_date[32];			//! Version date

    uint8_t		mode;
    uint8_t		battery_lvl;
    uint8_t		mcu_temp;
    float		vref;


    float		bme280_temperature;
    float		bme280_humidity;
    float		bme280_pressure;
    float		bme280_altitude;


	float	 	bme680_temperature; /*! Temperature in degree celsius x100 */
	uint32_t 	bme680_pressure; /*! Pressure in Pascal */
	float	 	bme680_humidity; /*! Humidity in % relative humidity x1000 */
	float		bme680_altitude;
	uint32_t 	bme680_gas; /*! Gas resistance in Ohms */
	uint16_t	bme680_iaq;

	uint16_t	MiCS_CO;
	uint16_t	MiCS_NH3;
	uint16_t	MiCS_NO2;
};

#define ID_PRODUCT			"AirAnalyzer"
#define ID_VENDOR			"Mikita Valatovich"
#define ID_VERSION			"0.0.1"
#define ID_DATE				"20221205"
