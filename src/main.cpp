
//#define DEBUG
#define DUAL
#define GPS

#include <MS5611.h>             //  https://github.com/nichtgedacht/Arduino-MS5611-Interrupt
#include "JetiExBusProtocol.h"  //	https://github.com/nichtgedacht/JetiExBus

JetiExBusProtocol exBus;

//int j = 0;

// minumum time constants of filter
// will be controlled by ExBus channel 12 later
double T1 = 150000;
double T2 = 200000;

char input;

uint16_t channelValue;

// sets refresh rate
char refresh_10hz[] = { 0xb5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00,
                        0x01, 0x00, 0x01, 0x00, 0x7a, 0x12 };

// sets messages 
char ubx_nav_pvt[] = { 0xb5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x07,
                       0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x18, 0xe1 };

// sets port
char prt_ubx_only_19200[] = { 0xb5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00,
                              0x00, 0x00, 0xd0, 0x08, 0x00, 0x00, 0x00, 0x4b,
                              0x00, 0x00, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00,
                              0x00, 0x00, 0x46, 0x4b };


char enable_galileo[] = { 0xb5, 0x62, 0x06, 0x3e, 0x0c, 0x00, 0x00, 0x00,
                          0x20, 0x01, 0x02, 0x04, 0x08, 0x00, 0x01, 0x00,
                          0x01, 0x01, 0x82, 0x56 };

// it is strongly recommended to keep IDs in order without a gap 
enum {
    ID_CLIMB = 1,
    ID_ALTITUDE,
    ID_GPSLON,
    ID_GPSLAT,
    ID_GPSSPD,
    ID_GPSALT,
    ID_GPSVEH
};

// Attention! parameter priority added. Value will be send after every N times of completed sets
// of all sensors are sent. Where N is priority. 
JETISENSOR_CONST sensors[] PROGMEM = {
    //id,           name,            unit,               dataType, precision, priority 
    { ID_CLIMB,     "Vario",         "m/s",  JetiSensor::TYPE_14b, 2,          0 },
    { ID_ALTITUDE,  "AltRelat.",     "m",    JetiSensor::TYPE_14b, 1,          4 },
#ifdef GPS
    { ID_GPSLON,    "GPS Longitude", "",     JetiSensor::TYPE_GPS, 0,          3 },
	{ ID_GPSLAT,    "GPS Latitude",  "",     JetiSensor::TYPE_GPS, 0,          3 },
    { ID_GPSSPD,    "GPS Speed",     "m/s",  JetiSensor::TYPE_14b, 2,          3 },
	{ ID_GPSALT,    "GPS Altitude",  "m",    JetiSensor::TYPE_14b, 1,          4 },
	{ ID_GPSVEH,    "GPS Vehicles",   "",    JetiSensor::TYPE_6b,  0,         10 },
#endif      
    0                           // end of array
};

double referencePressure_1 = 0, referencePressure_2 = 0;
double r_altitude_1 = 0, r_altitude0_1 = 0, r_altitude_2 = 0, r_altitude0_2 = 0; 
double climb_1 = 0, climb0_1 = 0, climb_2 = 0, climb0_2 = 0; 
double dyn_alfa_1, dyn_alfa_2, alfa_1, alfa_2, factor;

uint32_t diff_t_A, max_diff_t_A, diff_t_B, max_diff_t_B;
double relativeAltitude_1 = 0, relativeAltitude_2 = 0; 

//-------------------- UBX ----------------------------

typedef union {

    struct {
                                /* Comments stohlen from PX4 */
	    uint32_t iTOW;          /**< GPS Time of Week [ms] */
	    uint16_t year;          /**< Year (UTC)*/
	    uint8_t  month;         /**< Month, range 1..12 (UTC) */
	    uint8_t  day;           /**< Day of month, range 1..31 (UTC) */
	    uint8_t  hour;          /**< Hour of day, range 0..23 (UTC) */
	    uint8_t  min;           /**< Minute of hour, range 0..59 (UTC) */
	    uint8_t  sec;           /**< Seconds of minute, range 0..60 (UTC) */
	    uint8_t  valid;         /**< Validity flags (see UBX_RX_NAV_PVT_VALID_...) */
	    uint32_t tAcc;          /**< Time accuracy estimate (UTC) [ns] */
	    int32_t  nano;          /**< Fraction of second (UTC) [-1e9...1e9 ns] */
	    uint8_t  fixType;       /**< GNSSfix type: 0 = No fix, 1 = Dead Reckoning only, 2 = 2D fix, 3 = 3d-fix, 4 = GNSS + dead reckoning, 5 = time only fix */
	    uint8_t  flags;         /**< Fix Status Flags (see UBX_RX_NAV_PVT_FLAGS_...) */
	    uint8_t  flags2;        /**< Additional flags */
	    uint8_t  numSV;         /**< Number of SVs used in Nav Solution */
	    int32_t  lon;           /**< Longitude [1e-7 deg] */
	    int32_t  lat;           /**< Latitude [1e-7 deg] */
	    int32_t  height;        /**< Height above ellipsoid [mm] */
	    int32_t  hMSL;          /**< Height above mean sea level [mm] */
	    uint32_t hAcc;          /**< Horizontal accuracy estimate [mm] */
	    uint32_t vAcc;          /**< Vertical accuracy estimate [mm] */
	    int32_t  velN;          /**< NED north velocity [mm/s]*/
	    int32_t  velE;          /**< NED east velocity [mm/s]*/
	    int32_t  velD;          /**< NED down velocity [mm/s]*/
	    int32_t  gSpeed;        /**< Ground Speed (2-D) [mm/s] */
	    int32_t  headMot;       /**< Heading of motion (2-D) [1e-5 deg] */
	    uint32_t sAcc;          /**< Speed accuracy estimate [mm/s] */
	    uint32_t headAcc;       /**< Heading accuracy estimate (motion and vehicle) [1e-5 deg] */
	    uint16_t pDOP;          /**< Position DOP [0.01] */
	    uint16_t flags3;        /**< Additional flags */
	    uint32_t reserved1;     /**< reserved */
	    int32_t  headVeh;       /**< (ubx8+ only) Heading of vehicle (2-D) [1e-5 deg] */
	    int16_t  magDec;        /**< magnetc declination */
        uint16_t magAcc;        /**< magnetic accelaration */

    } Val;

    uint8_t UBXBuffer[92];

} NavPvt_t;
   
typedef union {

    struct {
                         /* Comments stohlen from PX4 */
	    uint32_t iTOW;   /**< GPS Time of Week [ms] */
	    int32_t  lon;    /**< Longitude [1e-7 deg] */
	    int32_t  lat;    /**< Latitude [1e-7 deg] */
	    int32_t  height; /**< Height above ellipsoid [mm] */
	    int32_t  hMSL;   /**< Height above mean sea level [mm] */
	    uint32_t hAcc;   /**< Horizontal accuracy estimate [mm] */
	    uint32_t vAcc;   /**< Vertical accuracy estimate [mm] */

    } Val;

    uint8_t UBXBuffer[28];

} NavPosllh_t;

enum UBXFrameState
{
	WaitStart1,
    WaitStart2,
	WaitClass,
    WaitID,
	WaitLen1,
    WaitLen2,
    WaitPayload,
    WaitChk2
};

UBXFrameState state = WaitStart1;
NavPvt_t NavPvt;
NavPosllh_t NavPosllh;

void DecodeUBX(uint8_t Class, uint8_t ID) {

    bool valid = false;

    if ( Class == 1 && ID == 7 ) {

        if ( NavPvt.Val.fixType == 3 )
            valid = true;

#ifdef DEBUG
        SerialUSB.print((double)NavPvt.Val.lon/10000000,7);
        SerialUSB.println("°");
        SerialUSB.print((double)NavPvt.Val.lat/10000000,7);
        SerialUSB.println("°");
        SerialUSB.print( (double)NavPvt.Val.hMSL  / 1000 ,3);
        SerialUSB.println("m (MSL)");
        SerialUSB.print( (double)NavPvt.Val.height / 1000 ,3);
        SerialUSB.println("m (above Ellipsoid)");
        SerialUSB.print((double)NavPvt.Val.gSpeed / 1000, 3);
        SerialUSB.println("m/s");
        SerialUSB.print(NavPvt.Val.year);
        SerialUSB.print("/");
        SerialUSB.print(NavPvt.Val.month);
        SerialUSB.print("/");
        SerialUSB.println(NavPvt.Val.day);
        SerialUSB.print(NavPvt.Val.hour);
        SerialUSB.print(":");
        SerialUSB.print(NavPvt.Val.min);
        SerialUSB.print(":");
        SerialUSB.println(NavPvt.Val.sec);
#endif
        exBus.SetSensorValueGPS (ID_GPSLAT, false, (double)NavPvt.Val.lat/10000000, valid);
        exBus.SetSensorValueGPS (ID_GPSLON, true,  (double)NavPvt.Val.lon/10000000, valid);
        exBus.SetSensorValue (ID_GPSALT, round(NavPvt.Val.height/100), valid);
        exBus.SetSensorValue (ID_GPSSPD, round(NavPvt.Val.gSpeed/10), valid);
        exBus.SetSensorValue (ID_GPSVEH, NavPvt.Val.numSV, valid);
    }
    
#ifdef DEBUG
    SerialUSB.print(NavPvt.Val.numSV);
    SerialUSB.println(" vehicles");    

    switch ( NavPvt.Val.fixType ) {

        case 0: 
            SerialUSB.println("no fix");
            break;
        case 2:
            SerialUSB.println("2d fix");
            break;
        case 3:
            SerialUSB.println("3d fix");
            break;           
    } 
#endif      
}

void parse_UBX_NAV(char c) {

    static int i;
    static uint16_t UBXLengt;
    static uint8_t length1;
    static uint8_t UBXClass;
    static uint8_t UBXID;
    static uint8_t CheckSumA;
    static uint8_t CheckSumB;
    static uint8_t UBXChkA;
    static uint8_t UBXChkB;
    static uint16_t UBXPayloadLength;
    static uint8_t *buffer;

    switch ( state ) {

        case WaitStart1: 
            if ( c == 0xb5 ) { 
                CheckSumA = 0;
                CheckSumB = 0;
                state = WaitStart2;
            }
            break;

        case WaitStart2:
            if ( c == 0x62 ) {
                state = WaitClass;
            }
            break;

        case WaitClass:
            UBXClass = c;
            CheckSumA = CheckSumA + c;
            CheckSumB = CheckSumB + CheckSumA;
            state = WaitID;
            break;

        case WaitID:
            UBXID = c;
            CheckSumA = CheckSumA + c;
            CheckSumB = CheckSumB + CheckSumA;
             
            switch (UBXID) {
                case 7:
                    UBXPayloadLength = 92;
                    buffer = &NavPvt.UBXBuffer[0];
                    state = WaitLen1;
                    break;
                case 2:
                    UBXPayloadLength = 28;
                    buffer = &NavPosllh.UBXBuffer[0];
                    state = WaitLen1;
                    break;
                default:
                    SerialUSB.print("no decoder for NAV-ID: ");
                    SerialUSB.println(UBXID);
                    state = WaitStart1;
                    break;
            }
            break;

        case WaitLen1:
            length1 = c;
            CheckSumA = CheckSumA + c;
            CheckSumB = CheckSumB + CheckSumA;
            state = WaitLen2;
            break;

        case WaitLen2:
            UBXLengt = (c << 8) | length1;
            CheckSumA = CheckSumA + c;
            CheckSumB = CheckSumB + CheckSumA;

            if ( UBXLengt == UBXPayloadLength) {
                state = WaitPayload;
                i = 0;
            } else {
                state = WaitStart1; // error, try again
                SerialUSB.println("error UBXLengt");
                state = WaitStart1;
            }
            break;

        case  WaitPayload:          
            if ( i < UBXLengt ) {
                buffer[i++] = c;
                CheckSumA = CheckSumA + c;
                CheckSumB = CheckSumB + CheckSumA;
            }  else {
                UBXChkA = c;
                state = WaitChk2;
            }
            break;

        case  WaitChk2:
            UBXChkB = c;
            if ( CheckSumA == UBXChkA && CheckSumB == UBXChkB ) {
                DecodeUBX(UBXClass, UBXID);
            }
            state = WaitStart1;
            break;

        default:
            state = WaitStart1;
            break;  
    }     
}         

// -------------------- /UBX ---------------------------

void setup () {

    uint8_t i;

#ifdef GPS

    // Start GPS connection default baud rate
    Serial1.begin (9600);
	while (!Serial1) {};

    for (i = 0; i < sizeof(prt_ubx_only_19200); i++) {
       Serial1.write(prt_ubx_only_19200[i]); 
    }

    delay(100);

    Serial1.begin (19200);
	while (!Serial1) {};
    delay(100);

    // Set output UBX NAV-PVT on UART1
    for (i = 0; i < sizeof(ubx_nav_pvt); i++) {
        Serial1.write(ubx_nav_pvt[i]);
    }

    for (i = 0; i < sizeof(refresh_10hz); i++) {
        Serial1.write(refresh_10hz[i]);
    }

    for (i = 0; i < sizeof(enable_galileo); i++) {
        Serial1.write(enable_galileo[i]);
    }

#endif    

#ifdef DEBUG
    Serial1.println("Initialize MS5611 Sensor");
#endif

    // Initialize MS5611 sensor(s)
    // first argument oversampling rate pressure, second for temperature
    // if third argument is "true" second sensor with alternative (non default) address becomes active
    // remaining arguments are for the second sensor
    // except the third argument the following call is showing the default values of the lib

#ifdef DUAL      
    while (!ms5611.begin (MS5611_ULTRA_HIGH_RES, MS5611_STANDARD, true, MS5611_ULTRA_HIGH_RES, MS5611_STANDARD)) {
#else
    while (!ms5611.begin (MS5611_ULTRA_HIGH_RES, MS5611_STANDARD)) {
#endif

    // if a single sensor is used, only 2 args needed as in the old version of the lib: 
    // while (!ms5611.begin (MS5611_ULTRA_HIGH_RES, MS5611_STANDARD )) {
    // if these 2 args are as shown they can be ommited also:
    // while (!ms5611.begin ()) {
         
#ifdef DEBUG
        Serial1.println ("Could not find a valid MS5611 sensor, check wiring!");
#endif
        delay (500);
    }

    // calc alfas from ms5611.delta_t for time constants chosen
    // ms5611.delta_t depends on number of sensors and oversampling rates chosen
    alfa_1 = ms5611.delta_t / ( T1 + ms5611.delta_t );
    alfa_2 = ms5611.delta_t / ( T2 + ms5611.delta_t );

    // calc gain from time constants chosen 
    factor = 1000000 / (T2 - T1);

    // warm up
    i = 0;
    while (i < 100) {
        if (ms5611.data_ready) {
            ms5611.getPressure (true, 1);
#ifdef DUAL            
            ms5611.getPressure (true, 2);
#endif            
            i++;
        }
    }

    // get reference
    i = 0;
    while (i < 100) {
        if (ms5611.data_ready) {
            referencePressure_1 += ms5611.getPressure (true, 1);
#ifdef DUAL            
            referencePressure_2 += ms5611.getPressure (true, 2);
#endif            
            i++;
        }
    }
    referencePressure_1 = referencePressure_1 / 100;
#ifdef DUAL    
    referencePressure_2 = referencePressure_2 / 100;
#endif    

    exBus.SetDeviceId(0x76, 0x32); // 0x3276
    exBus.Start ("mini_vario", sensors, 0);

    // all build in LEDs
    pinMode(PIN_LED_TXL, OUTPUT);
    pinMode(PIN_LED_13, OUTPUT);
    pinMode(PIN_LED_RXL, OUTPUT);
    digitalWrite( 13, HIGH );
    digitalWrite( 12, HIGH );
    digitalWrite( 11, HIGH );

    // for debugging with logic analyzer
    pinMode(PIN_A8, OUTPUT);
    digitalWrite(PIN_A8, LOW);
    pinMode(PIN_A0, OUTPUT);
    digitalWrite(PIN_A0, LOW);
}

void loop () {

    //digitalWrite(PIN_A8, HIGH);

 	if (exBus.IsBusReleased())
	{ 
        if ( exBus.HasNewChannelData() )
        {
            // channel 12 hardcoded for now
            channelValue = exBus.GetChannel(11);
            //char buf[30];
            //sprintf(buf, "chan-%d: %.4d", 12, channelValue);
            //SerialUSB.println(buf);

            // make time constants variable
            // channel at 100% and no trimm -> 1000 - 2000
            // check in servo monitor !
            T1 = 150000 + 150000 * ((double)channelValue / 1000 - 1 );
            T2 = 200000 + 200000 * ((double)channelValue / 1000 - 1 );

            // calc alfas from ms5611.delta_t for time constants chosen
            // ms5611.delta_t depends on number of sensors and oversampling rates chosen
            alfa_1 = ms5611.delta_t / ( T1 + ms5611.delta_t );
            alfa_2 = ms5611.delta_t / ( T2 + ms5611.delta_t );

            // calc gain from time constants chosen 
            factor = 1000000 / (T2 - T1);

            //dtostrf(T1, 6, 0, buf);
            //SerialUSB.println(buf);
       	}
	}
    
    if (ms5611.data_ready) {    // flag is interrupt trigggered and reset by ms5611.getPressure

        // For debugging with logic analyzer
        // digitalWrite(PIN_A8, HIGH);

        long realPressure_1 = ms5611.getPressure (true, 1);
#ifdef DUAL        
        long realPressure_2 = ms5611.getPressure (true, 2);
#endif
        relativeAltitude_1 = ms5611.getAltitude (realPressure_1, referencePressure_1);
#ifdef DUAL        
        relativeAltitude_2 = ms5611.getAltitude (realPressure_2, referencePressure_2);
#endif

/******************************* TEST sample period 13.5ms (Dual Sensor) ***************************************
        relativeAltitude_2 = 0;
        relativeAltitude_1 = 0;
        j++;
        if ( j <= 186) {
            relativeAltitude_2 += j * 0.0135;  // 2.511m / 2.511s = 1m/s
            relativeAltitude_1 += j * 0.0135;  // 2.511m / 2.511s = 1m/s
        }
        else {
            if ( j <= 1000 ) {
                relativeAltitude_2 += 2.511;
                relativeAltitude_1 += 2.511;   
            } else {
                if ( j <= 1019 ) {
                    relativeAltitude_2 += 2.511 - ( j - 1000 ) * 0.13215789474; // -2.511m / 0.2511s = -10m/s
                    relativeAltitude_1 += 2.511 - ( j - 1000 ) * 0.13215789474; // -2.511m / 0.2511s = -10m/s
                }
            }
        }
        if ( j == 2000) {
            j = 0;
        }
*****************************************************************************************************************/      

// *   dt means sample time interval [s] (delta_t)
// *   T  means time constant of exponential filter [s] (T1, T2)
// *   dT means difference of these Time constants [s] (T2 - T1)
// *   dx means difference of the results of both exponetial filters  
// *   alpha means smoothsness factor ( determines T )
// *   
// *   exponential filter:
// *   output[n] = output[n-1] - alpha * ( output[n-1] - input )
// *   
// *   if alpha == 1 then the output will be equal the input without
// *   any filtering
// *   
// *   relations:
// *   
// *   climb = dx / dT
// *   
// *   alpha = dt / ( T + dt )
// *   T = (dt / alpha) - dt
// * 
// *   dT = T2 - T1
// *   dT = (dt / alpha1) - ( dt / alpha2 ) 
// * 

        r_altitude0_1 = r_altitude0_1 - alfa_1 * (r_altitude0_1 - relativeAltitude_1);
#ifdef DUAL        
        r_altitude0_2 = r_altitude0_2 - alfa_1 * (r_altitude0_2 - relativeAltitude_2);
#endif        
        
        r_altitude_1 = r_altitude_1 -  alfa_2 * (r_altitude_1 - relativeAltitude_1);
#ifdef DUAL        
        r_altitude_2 = r_altitude_2 -  alfa_2 * (r_altitude_2 - relativeAltitude_2);
#endif
    
        climb0_1 = (r_altitude0_1 - r_altitude_1) * factor;   // Factor is 1000000/dT ( 1/dT as seconds )
#ifdef DUAL        
        climb0_2 = (r_altitude0_2 - r_altitude_2) * factor;   // Factor is 1000000/dT ( 1/dT as seconds )
#endif
        
        // smoothing the climb value by another exponential filter
        // time constant of filter changes dynamically
        // greater speed of change means less filtering.
        // see "Nonlinear Exponential Filter"   
        dyn_alfa_1 = abs( (climb_1 - climb0_1) / 0.8 );
#ifdef DUAL        
        dyn_alfa_2 = abs( (climb_2 - climb0_2) / 0.8 );
#endif        
        if ( dyn_alfa_1 >= 1 ) {
            dyn_alfa_1 = 1;
        }
        
#ifdef DUAL        
        if ( dyn_alfa_2 >= 1 ) {
            dyn_alfa_2 = 1;
        }
#endif                
        climb_1 = climb_1 - dyn_alfa_1 * ( climb_1 - climb0_1 );
#ifdef DUAL        
        climb_2 = climb_2 - dyn_alfa_2 * ( climb_2 - climb0_2 );
#endif        
        
#ifdef DEBUG
        /*/ output for plotter
        SerialUSB.print (climb_1);
        SerialUSB.print ("\t");
        SerialUSB.print (climb_2);     
        SerialUSB.print ("\t");
        SerialUSB.print (r_altitude0_1);
        SerialUSB.print ("\t");
        SerialUSB.println (r_altitude0_2);
        */
        SerialUSB.print (climb_1);
        SerialUSB.println ("m/s");
        SerialUSB.print (climb_2);     
        SerialUSB.println ("m/s");
        SerialUSB.print (r_altitude0_1);
        SerialUSB.println ("m");
        SerialUSB.print (r_altitude0_2);
        SerialUSB.println ("m");

#endif

#ifdef DUAL
        // use climb from vario with TEK feature            
        exBus.SetSensorValue (ID_CLIMB, round ((climb_2) * 100), true);
#else 
        exBus.SetSensorValue (ID_CLIMB, round ((climb_1) * 100));
#endif         
        exBus.SetSensorValue (ID_ALTITUDE, round ((r_altitude0_1) * 10), true);

        // For debugging with logic analyzer
        //digitalWrite(PIN_A8, LOW);
#ifdef GPS
    } else { // if ms5611.data_ready false

        // For debugging with logic analyzer
        //digitalWrite(PIN_A8, HIGH);

        while ( Serial1.available())
        {
            // calls decodeUBX() 
            parse_UBX_NAV(Serial1.read());

            //input = Serial1.read();
            //SerialUSB.print(input, HEX);
            //SerialUSB.print(" ");   

        }        
#endif        
        // For debugging with logic analyzer
        //digitalWrite(PIN_A8, LOW);
    }

    //digitalWrite(PIN_A8, LOW);

    //digitalWrite(PIN_A8, HIGH);
    exBus.DoJetiExBus();
    //digitalWrite(PIN_A8, LOW);
      
} // loop
