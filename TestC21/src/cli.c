/*
 * cli.c
 *
 */
#include <FreeRTOS.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "cli.h"

const char *ws1=" \t";
const char *ws2=" \t\r\n";

/* Hush compiler warnings */
static short check_digits(char *p);
static int16_t get_num(char *token, uint32_t *param);
static uint8_t match_command(char *token);

typedef struct _cli {
  const char *text;
  const char *help;
  uint16_t cmd;
  const char *verbose;
} CLICMDS;

static const
CLICMDS cli[NUM_COMMANDS] = {
    {"can", "\tcan\r\n",                           CMD_CAN,
            "\t! initialize the CAN controller\r\n"},
    {"xcan", "\txcan\r\n",                         CMD_XCAN,
            "\t! stop the CAN controller\r\n"},
    {"baud", "\tbaud 250|500|1000\r\n",            CMD_BAUD,
            "\t! set the CAN baud rate\r\n"},
    {"ping", "\tping <count>\r\n",                 CMD_PING,
            "\t! send 0x422 on CAN until Rx\r\n"},
    {"send", "\tsend\r\n",                         CMD_SEND,
            "\t! send one 0x422 packet\r\n"},
    {"loop", "\tloop <count>\r\n",                 CMD_LOOP,
            "\t! send 0x422 on CAN <count> times\r\n"},
    {"help", "\thelp [cmd]\r\n",                   CMD_HELP,
            "\t! show verbose help for <cmd> or brief help for all commands\r\n"},
    {"reader", "\treader\r\n",                     CMD_READER,
            "\t! Turn on CAN read task\r\n"},
    {"can_detect", "\tcan_detect\r\n",             CMD_DETECT,
            "\t! Listen for packets at various baud rates\r\n"},
    {"nvm", "\tnvm\r\n",                           CMD_NVM,
            "\t! Initialize the NVM controller\r\n"},
    {"eep", "\teep\r\n",                           CMD_EEP,
            "\t! Test read/write/erase of RWWEE area\r\n"},
    {"led", "\tled  <led number>\r\n",             CMD_LED,
            "\t! Turn on LED (1-green, 2-red)\r\n"},
    {"xled", "\txled  <led number>\r\n",           CMD_XLED,
            "\t! Turn off LED (1-green, 2-red)\r\n"},
    {"shift", "\tshift  <value>\r\n",              CMD_SHIFT,
            "\t! Put an 8-bit <value> in the shift register\r\n"},
    {"gen", "\tgen  <analog_input_num> <K-ohms> \r\n", CMD_GEN,
            "\t! Put 2 or 10 Kohms reistance on analogg input 0-3\r\n"},
    {"xgen", "\txgen  <analog_input_num>\r\n",      CMD_XGEN,
            "\t! Disable resistance on analog input 0-3\r\n"},
    {"adctest", "\tadctest\r\n",                    CMD_ADCTEST,
            "\t! Run ADC test - Voltage divider into AIN8\r\n"},
    {"sdtest", "\tsdtest\r\n",                      CMD_SDTEST,
            "\t! Run SDADC test\r\n"},
};

const
char *get_help(uint8_t cmd)
{
    uint8_t i = 0;
    const char *rval = NULL;

    for (i = 0; i < NUM_COMMANDS; i++) {
       if (cmd == cli[i].cmd) {
           rval = cli[i].help;
           break;
       }
    }
    return (rval);
}

const char *get_help_verbose(uint8_t cmd)
{
    uint8_t i = 0;
    const char *rval = NULL;

    for (i = 0; i < NUM_COMMANDS; i++) {
       if (cmd == cli[i].cmd) {
           rval = cli[i].verbose;
           break;
       }
    }
    return (rval);
}

/*
 * Make sure all the chars in a string are numeric digits.
 * If so, return 0
 *   otherwise return 1
 */
short check_digits(char *p)
{
    short rval = 0;
    while (*p) {
        if (!isdigit(*p)) {
            rval = 1;
            break;
        }
        else
            p++;
    }
    return (rval);
}

/*
 * Convert a decimal string to a number.
 * Value is returned in param.
 * Failed conversion returns 1
 */
int16_t get_num(char *token, uint32_t *param)
{
    if (check_digits(token) == 0) {
        *param = atoi(token);
        return 0;
    }
    return 1;
}

/*
 * Find token in table.
 * Return command code, or 0 if not found.
 */
uint8_t match_command(char *token)
{
    uint8_t i = NUM_COMMANDS;
    for (i = 0; i < NUM_COMMANDS; i++)
       if (strcmp(token, cli[i].text) == 0)
           return cli[i].cmd;
    return (0);
}

/*
 * Scan buffer for command.
 * Return CMD value, or 0 if no command.
 * If command has a arg, return in *param.
 */
int16_t get_command(char *buf, uint32_t *param, uint32_t *param2)
{
    char *token = buf;
    int16_t rval = 0;
    uint8_t cmd;

    /* Find beginning of command... (skip leading spaces) */
    token = strtok(buf, ws2);
    cmd = match_command(token);

    /* Early exit if argless command  */
    if (cmd == 0 ||                   
        cmd == CMD_CAN     || 
        cmd == CMD_XCAN    || 
        cmd == CMD_READER  || 
        cmd == CMD_DETECT  || 
        cmd == CMD_NVM     || 
        cmd == CMD_EEP     || 
        cmd == CMD_ADCTEST || 
        cmd == CMD_SDTEST  || 
        cmd == CMD_SEND)  
        return (cmd);

    switch (cmd) {              /* Switch on command */

        case CMD_GEN:

            token = strtok(NULL, ws1);
            rval = get_num(token, param);

            if (rval) {
                cmd = 0;
                break;
            }

            token = strtok(NULL, ws2);
            rval = get_num(token, param2);

            if (rval) 
                cmd = 0;
            
            break;

        case CMD_BAUD:
        case CMD_LOOP:
        case CMD_LED:
        case CMD_XLED:
        case CMD_PING:
        case CMD_SHIFT:
        case CMD_XGEN:

            token = strtok(NULL, ws2);
            rval = get_num(token, param);

            if (rval) 
                cmd = 0;

            break;

        case CMD_HELP:

            token = strtok(NULL, ws2);
            *param = match_command(token);
       
            break;
default:
break;
    }
    return (cmd);
}
