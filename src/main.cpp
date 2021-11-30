/* TTN-Tracker-Node-M0 - Version 0.1.0 - 2020-08-24
 * *******************************************************************************
 * 
 * This program is built to provide GPS tracking via LoRaWAN.
 * 
 * *******************************************************************************
 * 
 * Copyright (c) 2020 Matt Rude <matt@mattrude.com>
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * *******************************************************************************
 */

#include <Arduino.h>
#include <Adafruit_GPS.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

// Schedule TX every this many seconds (might become longer due to duty cycle limitations).
const unsigned TX_INTERVAL = 30;

// LoRaWAN NwkSKey, network session key
static const PROGMEM u1_t NWKSKEY[16] = { 0x5F, 0xDA, 0xF3, 0xA3,
                                          0xBD, 0xF9, 0x38, 0x29,
                                          0x78, 0x8A, 0x54, 0x9D,
                                          0x7F, 0xC0, 0x59, 0x27 };

// LoRaWAN AppSKey, application session key
static const u1_t PROGMEM APPSKEY[16] = { 0x09, 0x6F, 0x72, 0x70,
                                          0x51, 0x7C, 0xDF, 0x8E,
                                          0x9E, 0x74, 0x75, 0xD3,
                                          0x75, 0x8B, 0xFC, 0x02 };

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 26021314;

/*********************************************************************************
 * Setup the LoRa module
 */

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in arduino-lmic/project_config/lmic_project_config.h,
// otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;

// Pin mapping
// Adapted for Feather M0 per p.10 of [feather]
const lmic_pinmap lmic_pins = {
    .nss = 8,                       // chip select on feather (rf95module) CS
    .rxtx = 7,
    .rst = 4,                       // reset pin
    .dio = {6, 5, 3}, // assumes external jumpers [feather_lora_jumper]
                                    // DIO1 is on JP1-1: is io1 - we connect to GPO6
                                    // DIO1 is on JP5-3: is D2 - we connect to GPO5
};

/*********************************************************************************
 * Setup the GPS module
 */

// what's the name of the hardware serial port?
#define GPSSerial Serial1

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

// Create the mills timer
uint32_t timer = millis();


/*********************************************************************************
 * Create the required variables
 */

unsigned char loraData[10];
unsigned char txData[8];


/*********************************************************************************/

void do_send(osjob_t* j) {
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        #ifdef DEBUG
        Serial.println(F("OP_TXRXPEND, not sending"));
        #endif
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, txData, sizeof(txData)-1, 0);
        #ifdef DEBUG
        Serial.println(F("Packet queued"));
        #endif
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void onEvent (ev_t ev) {
    #ifdef DEBUG
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            Serial.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;
        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
    #endif
}

/*********************************************************************************
 * Initialize the main function
 */
int main(void) {
    
    /*****************************************************************************
     * Setup the serial interface (only used for debuging)
     */

    
    // Enanble the serial monitor at 115200 baud
    //#ifdef DEBUG
    Serial.begin(115200);
    //#endif

    // Uncomment to have the sketch wait until Serial is ready
    // (disable for production)
    //#ifdef DEBUG
    while (!Serial);

    // Display the program header at boot
    Serial.println("Booting TTN-Tracker-Node-M0");
    //#endif


    /*****************************************************************************
     * Setup the LoRa module
     */
    
    // LMIC init
    os_init();

    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x13, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x13, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    #if defined(CFG_us915) || defined(CFG_au915)
    // NA-US and AU channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);

    #elif defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set. The LMIC doesn't let you change
    // the three basic settings, but we show them here.
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band

    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
        
    #elif defined(CFG_as923)
    // Set up the channels used in your country. Only two are defined by default,
    // and they cannot be changed.  Use BAND_CENTI to indicate 1% duty cycle.
    LMIC_setupChannel(0, 923200000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
    LMIC_setupChannel(1, 923400000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);

    #elif defined(CFG_kr920)
    // Set up the channels used in your country. Three are defined by default,
    // and they cannot be changed. Duty cycle doesn't matter, but is conventionally
    // BAND_MILLI.
    LMIC_setupChannel(0, 922100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
    LMIC_setupChannel(1, 922300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
    LMIC_setupChannel(2, 922500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
    
    #elif defined(CFG_in866)
    // Set up the channels used in your country. Three are defined by default,
    // and they cannot be changed. Duty cycle doesn't matter, but is conventionally
    // BAND_MILLI.
    LMIC_setupChannel(0, 865062500, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
    LMIC_setupChannel(1, 865402500, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
    LMIC_setupChannel(2, 865985000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);

    #else
    # warning No Region selected, please add "build_flags = -D CFG_us915=1" to your platformio.ini
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink
    LMIC_setDrTxpow(DR_SF7,14);


    /*****************************************************************************
     * Setup the GPS module
     */

    // 9600 is the default baud rate for Adafruit GPS, but others may use 4800
    GPS.begin(9600);

    // Enable RMC (recommended minimum) and GGA (fix data) including altitude
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

    // Set the update rate
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate

    // Request updates on antenna status, comment out to keep quiet
    GPS.sendCommand(PGCMD_ANTENNA);

    // Wait for one second to allow the GPS to correctly boot
    delay(1000);

    // Ask for firmware version
    GPSSerial.println(PMTK_Q_RELEASE);


    /*****************************************************************************
     * Start the main loop
     */

    while (1) {
        #ifdef DEBUG
        // read data from the GPS in the 'main loop'
        char c = GPS.read();

        // if you want to debug, this is a good time to do it!
        if (GPSECHO)
            if (c) Serial.print(c);
        #endif

        // approximately every 2 seconds or so, print out the current stats
        if (millis() - timer > 2000) {

            // Reset the timer
            timer = millis();

            // Start printing the GPS data to the serial prompt
            #ifdef DEBUG
            Serial.print("\nTime: ");
            if (GPS.hour < 10) { Serial.print('0'); }
            Serial.print(GPS.hour, DEC); Serial.print(':');
            if (GPS.minute < 10) { Serial.print('0'); }
            Serial.print(GPS.minute, DEC); Serial.print(':');
            if (GPS.seconds < 10) { Serial.print('0'); }
            Serial.print(GPS.seconds, DEC); Serial.print('.');
        
            if (GPS.milliseconds < 10) {
                Serial.print("00");
            } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
                Serial.print("0");
            }
        
            Serial.println(GPS.milliseconds);
            Serial.print("Date: ");
            Serial.print(GPS.day, DEC); Serial.print('/');
            Serial.print(GPS.month, DEC); Serial.print("/20");
            Serial.println(GPS.year, DEC);
            Serial.print("Fix: "); Serial.print((int)GPS.fix);
            Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
            #endif

            // Check if the GPS is connected, and only send data if it is.
            if (GPS.fix) {
                #ifdef DEBUG
                Serial.print("Location: ");
                Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
                Serial.print(", ");
                Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
                Serial.print("Speed (knots): "); Serial.println(GPS.speed);
                Serial.print("Angle: "); Serial.println(GPS.angle);
                Serial.print("Altitude: "); Serial.println(GPS.altitude);
                Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
                #endif
                
                // Build the item to send via LoRaWAN
                txData[0] = GPS.lat;
                txData[1] = GPS.lon;
                txData[2] = GPS.altitude;
                txData[3] = GPS.satellites;
                txData[4] = GPS.fixquality;
                txData[6] = GPS.fix;
                txData[7] = GPS.speed;

                // And send the data via LoRaWAN
                do_send(&sendjob);
            }
        }
    }
}