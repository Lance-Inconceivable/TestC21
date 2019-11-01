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

typedef struct _cli {
  char *text;
  char *help;
  uint16_t cmd;
  char *verbose;
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
    {"xreader", "\txreader\r\n",                   CMD_XREADER,
            "\t! Turn off CAN read task\r\n"},
    {"led", "\tled  <led number>\r\n",             CMD_LED,
            "\t! Turn on LED (TBD)\r\n"},
    {"xled", "\txled  <led number>\r\n",           CMD_XLED,
            "\t! Turn off LED (TBD)\r\n"},
};

char *get_help(uint8_t cmd)
{
    uint8_t i = 0;
    char *rval = NULL;

    for (i = 0; i < NUM_COMMANDS; i++) {
       if (cmd == cli[i].cmd) {
           rval = cli[i].help;
           break;
       }
    }
    return (rval);
}

char *get_help_verbose(uint8_t cmd)
{
    uint8_t i = 0;
    char *rval = NULL;

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
int16_t get_num(char *token, uint16_t *param)
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
int16_t get_command(char *buf, uint16_t *param, uint16_t *param2)
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
        cmd == CMD_XREADER || 
        cmd == CMD_SEND)  
        return (cmd);

    switch (cmd) {              /* Switch on command */

        case CMD_BAUD:
        case CMD_LOOP:
        case CMD_LED:
        case CMD_XLED:
        case CMD_PING:

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
