#include <FlashStorage.h>       //  build in lib for storing configuration
#include "config.h"

FlashStorage(flashStore, config_t);

config_t getConf() {

    config_t config = flashStore.read();

    if (config.valid == false) {
        // set default values
        config.prio_VARIOM = 0;
        config.prio_ALTITU = 4;

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
        config.valid = true;

        flashStore.write(config);
    }

    return config;
}

void printConfValue (const char* name, int value, int flashValue, int line, int column) {

    char string[40];
    if ( value != flashValue ) {
        sprintf(string, CURSPOS BRIGHTYELLOW_FG "%s %d (%d)" WHITE_FG, line, column, name, value, flashValue);
    } else {
        sprintf(string, CURSPOS "%s %d        ", line, column, name, value); 
    }
    SerialUSB.print(string);
}

void showConf(config_t config, config_t flashConfig) {

    SerialUSB.print("\033[48;5;4m"); //set global background (blue)

    printConfValue("prio_VARIOM:", config.prio_VARIOM, flashConfig.prio_VARIOM, 14, 0);
    printConfValue("prio_ALTITU:", config.prio_ALTITU, flashConfig.prio_ALTITU, 14, 25);
#ifndef GPS
    SerialUSB.print(GRAY_FG);
#endif
    printConfValue("prio_GPSLON:", config.prio_GPSLON, flashConfig.prio_GPSLON, 15, 0);
    printConfValue("prio_GPSLAT:", config.prio_GPSLAT, flashConfig.prio_GPSLAT, 15, 25);
    printConfValue("prio_GPSSPD:", config.prio_GPSSPD, flashConfig.prio_GPSSPD, 15, 50);
    printConfValue("prio_GPSALT:", config.prio_GPSALT, flashConfig.prio_GPSALT, 16, 0 );
    printConfValue("prio_GPSTIM:", config.prio_GPSTIM, flashConfig.prio_GPSTIM, 16, 25);  
    printConfValue("prio_GPSSAT:", config.prio_GPSSAT, flashConfig.prio_GPSSAT, 16, 50);
    printConfValue("prio_GPSDIS:", config.prio_GPSDIS, flashConfig.prio_GPSDIS, 17, 0);    
    printConfValue("prio_GPSHAC:", config.prio_GPSHAC, flashConfig.prio_GPSHAC, 17, 25);
    printConfValue("prio_GPSVAC:", config.prio_GPSVAC, flashConfig.prio_GPSVAC, 17, 50);
    printConfValue("prio_GPSHEA:", config.prio_GPSHEA, flashConfig.prio_GPSHEA, 18, 0);
    SerialUSB.print(WHITE_FG);
    printConfValue("enab_VARIOM:", config.enab_VARIOM, flashConfig.enab_VARIOM, 20, 0);  
    printConfValue("enab_ALTITU:", config.enab_ALTITU, flashConfig.enab_ALTITU, 20, 25);
#ifndef GPS
    SerialUSB.print(GRAY_FG);
#endif    
    printConfValue("enab_GPSLON:", config.enab_GPSLON, flashConfig.enab_GPSLON, 21, 0);
    printConfValue("enab_GPSLAT:", config.enab_GPSLAT, flashConfig.enab_GPSLAT, 21, 25);
    printConfValue("enab_GPSSPD:", config.enab_GPSSPD, flashConfig.enab_GPSSPD, 21, 50);
    printConfValue("enab_GPSALT:", config.enab_GPSALT, flashConfig.enab_GPSALT, 22, 0);
    printConfValue("enab_GPSTIM:", config.enab_GPSTIM, flashConfig.enab_GPSTIM, 22, 25);
    printConfValue("enab_GPSSAT:", config.enab_GPSSAT, flashConfig.enab_GPSSAT, 22, 50);
    printConfValue("enab_GPSDIS:", config.enab_GPSDIS, flashConfig.enab_GPSDIS, 23, 0);
    printConfValue("enab_GPSHAC:", config.enab_GPSHAC, flashConfig.enab_GPSHAC, 23, 25);
    printConfValue("enab_GPSVAC:", config.enab_GPSVAC, flashConfig.enab_GPSVAC, 23, 50);
    printConfValue("enab_GPSHEA:", config.enab_GPSHEA, flashConfig.enab_GPSHEA, 24, 0);
    SerialUSB.print(WHITE_FG);
    printConfValue("ctrl_CHANNL:", config.ctrl_CHANNL+1, flashConfig.ctrl_CHANNL+1, 26, 0);
    printConfValue("rset_CHANNL:", config.rset_CHANNL+1, flashConfig.rset_CHANNL+1, 26, 25);

    SerialUSB.print(CURSDOWN2 ERASELINE);
    SerialUSB.print("Input: ");
    SerialUSB.print(CYAN_BG "                  " CURS_C8); //make input field cyan
}

void cliConf (void) {

    uint8_t i;
    char c;
    char command[80];
    uint8_t pos = 0;
    char *cptr;
    char key[20];
    uint8_t value = 0;
    bool delete_unwanted = false;

    for(i=0; i<sizeof(command); i++) {
        command[i] = 0;
    }

    delay(100); //wait for other konsole output for example PlatformIO Serial Monitor messages

    SerialUSB.print(GLOBALBLUE_BG);
    SerialUSB.print(GLOBALWHITE_FG);
    SerialUSB.print(HOMEERASE);

    config_t flashConfig = flashStore.read();
    config_t config = flashStore.read();

    SerialUSB.println( "Welcome to JetiMiniVario CLI\n" );
    SerialUSB.println( "Usage: ");
    SerialUSB.println( "\"<Keyword> <Value>\" stage change" );
    SerialUSB.println( "\"exit\" exit without change" );
    SerialUSB.println( "\"write\" write changes to config\n" );
    SerialUSB.println( "prio_XXX values from 0 (highest) to 20, enab_XXX is 1 for enabled and");
    SerialUSB.println( "0 for disabled, ctrl_CHANNL and rset_CHANNL are each RC-channel-numbers." );
    SerialUSB.println( "Invalid input will be ignored.\n" );
    SerialUSB.println( "current config:");
    showConf(config, flashConfig);

    while (1) {

        while (SerialUSB.available()) {

            c = SerialUSB.read();

            if ( ( (pos < 19) && (c >= 31 && c <= 126) ) ) {
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
                            value = strtoumax(cptr, NULL, 10);
                        }
                        cptr = strtok(NULL, " =");
                    }

                    switch (i) {

                        case 1:

                            SerialUSB.print("\033[0G"); // cursor back

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
                                    config.prio_VARIOM = value;
                                }
#ifdef GPS
                            } else if ( strncmp(key, "prio_ALTITU", 11 ) == 0 ) {
                                if ( value <= 20 ) {
                                    config.prio_ALTITU = value;
                                }
                            } else if ( strncmp(key, "prio_GPSLON", 11 ) == 0 ) {
                                if ( value <= 20 ) {
                                    config.prio_GPSLON = value;
                                }
                            } else if ( strncmp(key, "prio_GPSLAT", 11 ) == 0 ) {
                                if ( value <= 20 ) {
                                    config.prio_GPSLAT = value;
                                }
                            } else if ( strncmp(key, "prio_GPSSPD", 11 ) == 0 ) {
                                if ( value <= 20 ) {
                                    config.prio_GPSSPD = value;
                                }
                            } else if ( strncmp(key, "prio_GPSALT", 11 ) == 0 ) {
                                if ( value <= 20 ) {
                                    config.prio_GPSALT = value;
                                }
                            } else if ( strncmp(key, "prio_GPSTIM", 11 ) == 0 ) {
                                if ( value <= 20 ) {
                                    config.prio_GPSTIM = value;
                                }
                            } else if ( strncmp(key, "prio_GPSSAT", 11 ) == 0 ) {
                                if ( value <= 20 ) {
                                    config.prio_GPSSAT = value;
                                }
                            } else if ( strncmp(key, "prio_GPSDIS", 11 ) == 0 ) {
                                if ( value <= 20 ) {
                                    config.prio_GPSDIS = value;
                                }
                            } else if ( strncmp(key, "prio_GPSHAC", 11 ) == 0 ) {
                                if ( value <= 20 ) {
                                    config.prio_GPSHAC = value;
                                }
                            } else if ( strncmp(key, "prio_GPSVAC", 11 ) == 0 ) {
                                if ( value <= 20 ) {
                                    config.prio_GPSVAC = value;
                                }
                            } else if ( strncmp(key, "prio_GPSHEA", 11 ) == 0 ) {
                                if ( value <= 20 ) {
                                    config.prio_GPSHEA = value;
                                }
#endif
                            } else if ( strncmp(key, "enab_VARIOM", 11 ) == 0 ) {
                                if ( value <= 1 ) {
                                    config.enab_VARIOM = value;
                                }
                            } else if ( strncmp(key, "enab_ALTITU", 11 ) == 0 ) {
                                if ( value <= 1 ) {
                                    config.enab_ALTITU = value;
                                }
#ifdef GPS
                            } else if ( strncmp(key, "enab_GPSLON", 11 ) == 0 ) {
                                if ( value <= 1 ) {
                                    config.enab_GPSLON = value;
                                }
                            } else if ( strncmp(key, "enab_GPSLAT", 11 ) == 0 ) {
                                if ( value <= 1 ) {
                                    config.enab_GPSLAT = value;
                                }
                            } else if ( strncmp(key, "enab_GPSSPD", 11 ) == 0 ) {
                                if ( value <= 1 ) {
                                    config.enab_GPSSPD = value;
                                }
                            } else if ( strncmp(key, "enab_GPSALT", 11 ) == 0 ) {
                                if ( value <= 1 ) {
                                    config.enab_GPSALT = value;
                                }
                            } else if ( strncmp(key, "enab_GPSTIM", 11 ) == 0 ) {
                                if ( value <= 1 ) {
                                    config.enab_GPSTIM = value;
                                }
                            } else if ( strncmp(key, "enab_GPSSAT", 11 ) == 0 ) {
                                if ( value <= 1 ) {
                                    config.enab_GPSSAT = value;
                                }
                            } else if ( strncmp(key, "enab_GPSDIS", 11 ) == 0 ) {
                                if ( value <= 1 ) {
                                    config.enab_GPSDIS = value;
                                }
                            } else if ( strncmp(key, "enab_GPSHAC", 11 ) == 0 ) {
                                if ( value <= 1 ) {
                                    config.enab_GPSHAC = value;
                                }
                            } else if ( strncmp(key, "enab_GPSVAC", 11 ) == 0 ) {
                                if ( value <= 1 ) {
                                    config.enab_GPSVAC = value;
                                }
                            } else if ( strncmp(key, "enab_GPSHEA", 11 ) == 0 ) {
                                if ( value <= 1 ) {
                                    config.enab_GPSHEA = value;
                                }
#endif
                            } else if ( strncmp(key, "ctrl_CHANNL", 11 ) == 0 ) {
                                if ( value < 16 ) {
                                    config.ctrl_CHANNL = value - 1;
                                }
                            } else if ( strncmp(key, "rset_CHANNL", 11 ) == 0 ) {
                                if ( value < 16 ) {
                                    config.rset_CHANNL = value - 1;
                                }
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
                    if (pos < 19) {
                        command[pos++] = c;
                    }
            }
        }
    }
}
