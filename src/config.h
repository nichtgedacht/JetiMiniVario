#ifndef CONFIG_H
#define CONFIG_H

#define GPS
#define DUAL

#define HOMEERASE "\033[H\033[2J"       //cursor home and erase all
#define GLOBALBLUE_BG "\033[48;5;4m"    //set background
#define GLOBALWHITE_FG "\033[38;5;15m"  //set foreground
#define CURSBACK "\033[0G"              //cursor column 0
#define CURS_C8 "\33[8G"                //cursor culumn 8
#define CURSDOWN2 "\033[2E"             //cursor down 2 lines
#define ERASELINE "\033[0K"             //erase line
#define BRIGHTYELLOW_FG "\033[93m"      //set foreground
#define WHITE_FG "\033[97m"             //set foreground
#define GRAY_FG "\033[37m"              //set foreground
#define CYAN_BG "\033[46m"              //set background
#define CURSPOS "\033[%d;%dH"           //cursor pos y x

typedef struct {
    boolean valid;
    uint8_t prio_VARIOM;
    uint8_t prio_ALTITU;

    uint8_t prio_GPSLON;
    uint8_t prio_GPSLAT;
    uint8_t prio_GPSSPD;
    uint8_t prio_GPSALT;
    uint8_t prio_GPSTIM;
    uint8_t prio_GPSSAT;
    uint8_t prio_GPSDIS;
    uint8_t prio_GPSHAC;
    uint8_t prio_GPSVAC;
    uint8_t prio_GPSHEA;

    bool enab_VARIOM;
    bool enab_ALTITU;

    bool enab_GPSLON;
    bool enab_GPSLAT;
    bool enab_GPSSPD;
    bool enab_GPSALT;
    bool enab_GPSTIM;
    bool enab_GPSSAT;
    bool enab_GPSDIS;
    bool enab_GPSHAC;
    bool enab_GPSVAC;
    bool enab_GPSHEA;

    uint8_t ctrl_CHANNL;
    uint8_t rset_CHANNL;
} config_t;

config_t getConf(void);
void printConfValue (const char*, int, int, int, int );
void showConf(config_t, config_t);
void cliConf (void);

#endif