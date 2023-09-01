#include <FlashStorage.h>       //  build in lib for storing configuration
#include "config.h"

FlashStorage(flashStore, config_t);

config_t getConf() {

    config_t config = flashStore.read();

    if (config.valid == false) {
        // set default values
        config.prio_VARIOM = 0;
        config.prio_ALTITU = 4;

        config.prio_VOLTAG = 5;

        config.prio_GPSLON = 3;
        config.prio_GPSLAT = 3;
        config.prio_GPSSPD = 3;
        config.prio_GPSALT = 4;
        config.prio_GPSTIM = 0;
        config.prio_GPSSAT = 20;
        config.prio_GPSDIS = 3;
        config.prio_GPSHAC = 10;
        config.prio_GPSVAC = 10;
        config.prio_GPSHEA = 3;

        config.enab_VARIOM = 1;
        config.enab_ALTITU = 1;

        config.enab_VOLTAG = 1;

        config.enab_GPSLON = 1;
        config.enab_GPSLAT = 1;
        config.enab_GPSSPD = 1;
        config.enab_GPSALT = 1;
        config.enab_GPSTIM = 0;
        config.enab_GPSSAT = 1;
        config.enab_GPSDIS = 1;
        config.enab_GPSHAC = 1;
        config.enab_GPSVAC = 1;
        config.enab_GPSHEA = 1;

        config.ctrl_CHANNL = 11;
        config.rset_CHANNL = 10;

        config.enab_CALIBR = 1;
        config.high_MEASUR = 31.04;
        config.lowr_MEASUR = 3.18;
        config.high_VOLTAG = 31.00;
        config.lowr_VOLTAG = 3.00;

        config.valid = true;

        flashStore.write(config);
    }

    return config;
}

void printConfValue (const char* name, int value, int flashValue, int line, int column) {

    char string[40];
    if ( value != flashValue ) {
        sprintf(string, CURSPOS BRIGHTYELLOW_FG "%s %d(%d)" WHITE_FG, line, column, name, value, flashValue);
    } else {
        sprintf(string, CURSPOS "%s %d    ", line, column, name, value); 
    }
    SerialUSB.print(string);
}

void printConfFloatValue (const char* name, double value, double flashValue, int line, int column) {

    char doubleBufferValue[10];
    char doubleBufferFlashValue[10];
    char string[40];

    dtostrf(value, 2, 2, doubleBufferValue);
    dtostrf(flashValue, 2, 2, doubleBufferFlashValue);

    if ( value != flashValue ) {
        sprintf(string, CURSPOS BRIGHTYELLOW_FG "%s %s(%s)" WHITE_FG, line, column, name, doubleBufferValue, doubleBufferFlashValue);
    } else {
        sprintf(string, CURSPOS "%s %s       ", line, column, name, doubleBufferValue); 
    }
    SerialUSB.print(string);
}

void showConf(config_t config, config_t flashConfig) {

    SerialUSB.print(BLUE_BG);
    SerialUSB.print(WHITE_FG);

    printConfValue("prio_VARIOM:", config.prio_VARIOM, flashConfig.prio_VARIOM, 14, 1);
    printConfValue("prio_ALTITU:", config.prio_ALTITU, flashConfig.prio_ALTITU, 14, 21);

#ifndef VOLT
    SerialUSB.print(GRAY_FG);
#endif
    printConfValue("prio_VOLTAG:", config.prio_VOLTAG, flashConfig.prio_VOLTAG, 14, 41);
    SerialUSB.print(WHITE_FG);
#ifndef GPS
    SerialUSB.print(GRAY_FG);
#endif
    printConfValue("prio_GPSLON:", config.prio_GPSLON, flashConfig.prio_GPSLON, 15, 1);
    printConfValue("prio_GPSLAT:", config.prio_GPSLAT, flashConfig.prio_GPSLAT, 15, 21);
    printConfValue("prio_GPSSPD:", config.prio_GPSSPD, flashConfig.prio_GPSSPD, 15, 41);
    printConfValue("prio_GPSALT:", config.prio_GPSALT, flashConfig.prio_GPSALT, 16, 1 );
    printConfValue("prio_GPSTIM:", config.prio_GPSTIM, flashConfig.prio_GPSTIM, 16, 21);  
    printConfValue("prio_GPSSAT:", config.prio_GPSSAT, flashConfig.prio_GPSSAT, 16, 41);
    printConfValue("prio_GPSDIS:", config.prio_GPSDIS, flashConfig.prio_GPSDIS, 17, 1);    
    printConfValue("prio_GPSHAC:", config.prio_GPSHAC, flashConfig.prio_GPSHAC, 17, 21);
    printConfValue("prio_GPSVAC:", config.prio_GPSVAC, flashConfig.prio_GPSVAC, 17, 41);
    printConfValue("prio_GPSHEA:", config.prio_GPSHEA, flashConfig.prio_GPSHEA, 18, 1);
    SerialUSB.print(WHITE_FG);
    printConfValue("enab_VARIOM:", config.enab_VARIOM, flashConfig.enab_VARIOM, 20, 1);  
    printConfValue("enab_ALTITU:", config.enab_ALTITU, flashConfig.enab_ALTITU, 20, 21);
#ifndef VOLT
    SerialUSB.print(GRAY_FG);
#endif
    printConfValue("enab_VOLTAG:", config.enab_VOLTAG, flashConfig.enab_VOLTAG, 20, 41);
    SerialUSB.print(WHITE_FG);
#ifndef GPS
    SerialUSB.print(GRAY_FG);
#endif
    printConfValue("enab_GPSLON:", config.enab_GPSLON, flashConfig.enab_GPSLON, 21, 1);
    printConfValue("enab_GPSLAT:", config.enab_GPSLAT, flashConfig.enab_GPSLAT, 21, 21);
    printConfValue("enab_GPSSPD:", config.enab_GPSSPD, flashConfig.enab_GPSSPD, 21, 41);
    printConfValue("enab_GPSALT:", config.enab_GPSALT, flashConfig.enab_GPSALT, 22, 1);
    printConfValue("enab_GPSTIM:", config.enab_GPSTIM, flashConfig.enab_GPSTIM, 22, 21);
    printConfValue("enab_GPSSAT:", config.enab_GPSSAT, flashConfig.enab_GPSSAT, 22, 41);
    printConfValue("enab_GPSDIS:", config.enab_GPSDIS, flashConfig.enab_GPSDIS, 23, 1);
    printConfValue("enab_GPSHAC:", config.enab_GPSHAC, flashConfig.enab_GPSHAC, 23, 21);
    printConfValue("enab_GPSVAC:", config.enab_GPSVAC, flashConfig.enab_GPSVAC, 23, 41);
    printConfValue("enab_GPSHEA:", config.enab_GPSHEA, flashConfig.enab_GPSHEA, 24, 1);
    SerialUSB.print(WHITE_FG);
    printConfValue("ctrl_CHANNL:", config.ctrl_CHANNL+1, flashConfig.ctrl_CHANNL+1, 26, 1);
    printConfValue("rset_CHANNL:", config.rset_CHANNL+1, flashConfig.rset_CHANNL+1, 26, 21);
#ifndef VOLT
    SerialUSB.print(GRAY_FG);
#endif    
    printConfValue("enab_CALIBR:", config.enab_CALIBR, flashConfig.enab_CALIBR, 28, 1);
    printConfFloatValue("high_MEASUR:", config.high_MEASUR, flashConfig.high_MEASUR, 29, 1);
    printConfFloatValue("lowr_MEASUR:", config.lowr_MEASUR, flashConfig.lowr_MEASUR, 29, 31);
    printConfFloatValue("high_VOLTAG:", config.high_VOLTAG, flashConfig.high_VOLTAG, 30, 1);
    printConfFloatValue("lowr_VOLTAG:", config.lowr_VOLTAG, flashConfig.lowr_VOLTAG, 30, 31);
    SerialUSB.print(WHITE_FG);

    SerialUSB.print("\r\n\n" ERASELINE "Input: " CURS_C8 BLACK_BG "                  " CURS_C8);
}

void cliConf (void) {

    uint8_t i;
    char c;
    char command[80];
    uint8_t pos = 0;
    char *cptr;
    char key[20];
    double value = 0;
    bool delete_unwanted = false;

    for(i=0; i<sizeof(command); i++) {
        command[i] = 0;
    }

    delay(100); //wait for other konsole output for example PlatformIO Serial Monitor messages

    SerialUSB.print(BLUE_BG);
    SerialUSB.print(WHITE_FG);
    SerialUSB.print(ERASEALL);

    config_t flashConfig = flashStore.read();
    config_t config = flashStore.read();

    SerialUSB.println( "Welcome to JetiMiniVario CLI\n" );
    SerialUSB.println( "Usage: ");
    SerialUSB.println( "\"<Keyword> <Value>\" stage change" );
    SerialUSB.println( "\"exit\" exit without change" );
    SerialUSB.println( "\"write\" write changes to config\n" );
    SerialUSB.println( "prio_XXX values from 0 (highest) to 20, enab_XXX is 1 for");
    SerialUSB.println( "enabled and 0 for disabled, ctrl_CHANNL and rset_CHANNL" );
    SerialUSB.println( "are each RC-channel-numbers.\n");
    SerialUSB.println( "current config:" );
    SerialUSB.println( "\n" );

    showConf(config, flashConfig);

    while (1) {

        while (SerialUSB.available()) {

            c = SerialUSB.read();

            if ( ( (pos < 17) && (c >= 31 && c <= 126) ) ) {
                SerialUSB.write(c);
            }

            switch (c) {
                // some terminals send 0x0D 0x0A on <enter> others send 0x0D only as Putty on default config
                case '\n':
                case '\r':

                    pos = 0;

                    if (delete_unwanted) { //ESC was detected
                        command[pos] = 0;
                        for(i=0; i<sizeof(command); i++) {
                            command[i] = 0;
                        }
                        SerialUSB.print(CURS_C8 "                  " CURS_C8); //clear input field
                        delete_unwanted = false;
                        break;
                    }

                    // get one or two token from command
                    i = 0;
                    cptr = strtok(command, " ");
                    while ( cptr != NULL) {
                        i++;
                        if ( i == 1) {
                            strcpy(key, cptr);
                        } else if ( i == 2 ) {
                            value = strtod(cptr, NULL);
                        }
                        cptr = strtok(NULL, " =");
                    }

                    switch (i) {

                        case 1:

                            SerialUSB.print(CURSBACK); // cursor back

                            if (strncmp(command, "exit", 4) == 0 ) {
                                SerialUSB.print("exiting rebooting ... ");
                                delay(500);
                                NVIC_SystemReset();
                                break; //never reached
                            } else if ( strncmp(command, "write", 5) == 0 ) {
                                // write current values to config
                                flashStore.write(config);
                                SerialUSB.print("config written rebooting ...");
                                delay(500);
                                NVIC_SystemReset();
                                break; //never reached
                            } else { //wrong command
                                command[pos] = 0;
                                SerialUSB.print(CURS_C8 "                  " CURS_C8); //clear input field
                            }
                            break;

                        case 2:
                            // change values after checking them
                            if ( strncmp(key, "prio_VARIOM", 11 ) == 0 ) {
                                if ( value <= 20 ) {
                                    config.prio_VARIOM = (uint8_t) value;
                                }
#ifdef GPS
                            } else if ( strncmp(key, "prio_ALTITU", 11 ) == 0 ) {
                                if ( value <= 20 ) {
                                    config.prio_ALTITU = (uint8_t) value;
                                }
                            } else if ( strncmp(key, "prio_GPSLON", 11 ) == 0 ) {
                                if ( value <= 20 ) {
                                    config.prio_GPSLON = (uint8_t) value;
                                }
                            } else if ( strncmp(key, "prio_GPSLAT", 11 ) == 0 ) {
                                if ( value <= 20 ) {
                                    config.prio_GPSLAT = (uint8_t) value;
                                }
                            } else if ( strncmp(key, "prio_GPSSPD", 11 ) == 0 ) {
                                if ( value <= 20 ) {
                                    config.prio_GPSSPD = (uint8_t) value;
                                }
                            } else if ( strncmp(key, "prio_GPSALT", 11 ) == 0 ) {
                                if ( value <= 20 ) {
                                    config.prio_GPSALT = (uint8_t) value;
                                }
                            } else if ( strncmp(key, "prio_GPSTIM", 11 ) == 0 ) {
                                if ( value <= 20 ) {
                                    config.prio_GPSTIM = (uint8_t) value;
                                }
                            } else if ( strncmp(key, "prio_GPSSAT", 11 ) == 0 ) {
                                if ( value <= 20 ) {
                                    config.prio_GPSSAT = (uint8_t) value;
                                }
                            } else if ( strncmp(key, "prio_GPSDIS", 11 ) == 0 ) {
                                if ( value <= 20 ) {
                                    config.prio_GPSDIS = (uint8_t) value;
                                }
                            } else if ( strncmp(key, "prio_GPSHAC", 11 ) == 0 ) {
                                if ( value <= 20 ) {
                                    config.prio_GPSHAC = (uint8_t) value;
                                }
                            } else if ( strncmp(key, "prio_GPSVAC", 11 ) == 0 ) {
                                if ( value <= 20 ) {
                                    config.prio_GPSVAC = (uint8_t) value;
                                }
                            } else if ( strncmp(key, "prio_GPSHEA", 11 ) == 0 ) {
                                if ( value <= 20 ) {
                                    config.prio_GPSHEA = (uint8_t) value;
                                }
#endif
                            } else if ( strncmp(key, "enab_VARIOM", 11 ) == 0 ) {
                                if ( value <= 1 ) {
                                    config.enab_VARIOM = (uint8_t) value;
                                }
                            } else if ( strncmp(key, "enab_ALTITU", 11 ) == 0 ) {
                                if ( value <= 1 ) {
                                    config.enab_ALTITU = (uint8_t) value;
                                }
#ifdef GPS
                            } else if ( strncmp(key, "enab_GPSLON", 11 ) == 0 ) {
                                if ( value <= 1 ) {
                                    config.enab_GPSLON = (uint8_t) value;
                                }
                            } else if ( strncmp(key, "enab_GPSLAT", 11 ) == 0 ) {
                                if ( value <= 1 ) {
                                    config.enab_GPSLAT = (uint8_t) value;
                                }
                            } else if ( strncmp(key, "enab_GPSSPD", 11 ) == 0 ) {
                                if ( value <= 1 ) {
                                    config.enab_GPSSPD = (uint8_t) value;
                                }
                            } else if ( strncmp(key, "enab_GPSALT", 11 ) == 0 ) {
                                if ( value <= 1 ) {
                                    config.enab_GPSALT = (uint8_t) value;
                                }
                            } else if ( strncmp(key, "enab_GPSTIM", 11 ) == 0 ) {
                                if ( value <= 1 ) {
                                    config.enab_GPSTIM = (uint8_t) value;
                                }
                            } else if ( strncmp(key, "enab_GPSSAT", 11 ) == 0 ) {
                                if ( value <= 1 ) {
                                    config.enab_GPSSAT = (uint8_t) value;
                                }
                            } else if ( strncmp(key, "enab_GPSDIS", 11 ) == 0 ) {
                                if ( value <= 1 ) {
                                    config.enab_GPSDIS = (uint8_t) value;
                                }
                            } else if ( strncmp(key, "enab_GPSHAC", 11 ) == 0 ) {
                                if ( value <= 1 ) {
                                    config.enab_GPSHAC = (uint8_t) value;
                                }
                            } else if ( strncmp(key, "enab_GPSVAC", 11 ) == 0 ) {
                                if ( value <= 1 ) {
                                    config.enab_GPSVAC = (uint8_t) value;
                                }
                            } else if ( strncmp(key, "enab_GPSHEA", 11 ) == 0 ) {
                                if ( value <= 1 ) {
                                    config.enab_GPSHEA = (uint8_t) value;
                                }
#endif
#ifdef VOLT
                            } else if ( strncmp(key, "enab_VOLTAG", 11 ) == 0 ) {
                                if ( value <= 1 ) {
                                    config.enab_VOLTAG = (uint8_t) value;
                                }
#endif
                            } else if ( strncmp(key, "ctrl_CHANNL", 11 ) == 0 ) {
                                if ( value < 16 ) {
                                    config.ctrl_CHANNL = (uint8_t) value - 1;
                                }
                            } else if ( strncmp(key, "rset_CHANNL", 11 ) == 0 ) {
                                if ( value < 16 ) {
                                    config.rset_CHANNL = (uint8_t) value - 1;
                                }
#ifdef VOLT
                            } else if ( strncmp(key, "enab_CALIBR", 11 ) == 0 ) {
                                if ( value <= 1 ) {
                                    config.enab_CALIBR = (uint8_t) value;
                                }
                            } else if ( strncmp(key, "high_MEASUR", 11 ) == 0 ) {
                                if ( value <= 33 ) {
                                    config.high_MEASUR = value;
                                }
                            } else if ( strncmp(key, "lowr_MEASUR", 11 ) == 0 ) {
                                if ( value <= 33 ) {
                                    config.lowr_MEASUR = value;
                                }
                            } else if ( strncmp(key, "high_VOLTAG", 11 ) == 0 ) {
                                if ( value <= 33 ) {
                                    config.high_VOLTAG = value;
                                }
                            } else if ( strncmp(key, "lowr_VOLTAG", 11 ) == 0 ) {
                                if ( value <= 33 ) {
                                    config.lowr_VOLTAG = value;
                                }
#endif
                            }
                            // resets input field also
                            showConf(config, flashConfig);
                    }

                    //clear command buffer after command was processed
                    for(i=0; i<sizeof(command); i++) {
                        command[i] = 0;
                    }
                    break;

                // accept both as backspace
                case '\b':
                case 0x7F:

                    if (delete_unwanted) { //ESC was detected
                        pos = 0;
                        command[pos] = 0;
                        for(i=0; i<sizeof(command); i++) {
                            command[i] = 0;
                        }
                        SerialUSB.print(CURS_C8 "                  " CURS_C8); //clear input field
                        delete_unwanted = false;
                        break;
                    }

                    if (pos > 0) {
                        command[--pos] = 0;
                        SerialUSB.print("\b \b");
                    }

                    break;

                case  0x1B:
                    delete_unwanted = true;

                default:
                    if (pos < 17) {
                        command[pos++] = c;
                    }
            }
        }
    }
}
