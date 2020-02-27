/*
 * cli.h
 *
 */

#ifndef CLI_H_
#define CLI_H_

#define CMD_HELP 1
#define CMD_LED  2
#define CMD_XLED 3
#define CMD_CAN  4
#define CMD_BAUD 5
#define CMD_PING 6
#define CMD_LOOP 7
#define CMD_XCAN 8
#define CMD_READER 9
#define CMD_SEND 10
#define CMD_DETECT 11
#define CMD_NVM  12
#define CMD_EEP 13
#define CMD_SHIFT 14
#define CMD_GEN   15
#define CMD_XGEN  16
#define CMD_ADCTEST 17
#define CMD_SDTEST 18
#define CMD_FREQM 19
#define CMD_FREQCOUNT 20
#define CMD_FREQSHOW 21
#define CMD_J1939ARBITRATION 22
#define CMD_J1939CLAIM 23

#define NUM_COMMANDS 23

int16_t get_command(char *buf, uint32_t *param, uint32_t *param2);
const char *get_help(uint8_t cmd);
const char *get_help_verbose(uint8_t cmd);


#endif /* CLI_H_ */
