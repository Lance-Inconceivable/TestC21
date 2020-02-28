/*
 * J1939_communication.h
 *
 * Created: 2/13/2020 5:01:21 PM
 *  Author: U103
 */ 


#ifndef J1939_COMMUNICATION_H_
#define J1939_COMMUNICATION_H_

#include <compiler.h>
#include <system.h>
#include <conf_can.h>


//J1939addressclaimed:  //ONLY allowed to use J1939 bus when an address has been successfully claimed.
#define J1939_NOTINITIALIZED		0
#define J1939_ADDRESSCLAIMED		1
#define J1939_CANNOTCLAIMADDRESS	-1

int J1939_Request_Data_Transfer(uint32_t PDU, unsigned char* data);
int J1939_AddressArbitration(void);
int J1939_AddressArbitration_RequestAutoAgentAddrClaim(unsigned int timeout);
void J1939_InitializeNAME(void);
int8_t J1939_NetworkMgmt_AddressClaim(void);
int J1939_Send(unsigned int PGN, unsigned char destination, unsigned char source, unsigned char *data, unsigned int len);
int J1939_Receive(unsigned int PGN, unsigned char destination, unsigned char source, unsigned char *RxMesBuff, uint16_t timeout);
int J1939_ReceivePacket(uint32_t rxEID, uint8_t* buffer, unsigned int mstimeout);
int J1939_ReceiveTP_CTS(unsigned int rxEID, uint32_t PGN, unsigned char* totalPackets, unsigned char* currentPacket);
int J1939_ReceiveTP_EndofMsgACK(unsigned int rxEID, uint32_t PGN, uint16_t* totalBytes, unsigned char* totalPackets);
int J1939_PGNDataRequest(unsigned int PGN, unsigned char ecm, unsigned char tool, unsigned char *RxMesBuff);
int J1939_SendPGNRequest(unsigned int PGN, unsigned char destination, unsigned char source);
void J1939_Set_Priority(int priority);
int J1939_Get_Priority(void);
void J1939_ClearRXdmsgs(void);

void J1939_NetworkMgmt_Task_Entry(void *argv);
int8_t getJ1939addressclaimed(void); //ONLY allowed to use J1939 bus when an address has been successfully claimed.
void set_contesting_NAME(uint8_t* buffer);
void J1939_NetworkMgmt_Task_Init(void);
void J1939_NetworkMgmt_IncomingAddressClaim(void);
void J1939_NetworkMgmt_IncomingRequestforAddressesClaimed(void);

#define AA_SA 0xAA
extern unsigned char 	SENSOR_SA;		//= 0xF9;	//Can change during startup(not implemented)
//extern CAN_HW_FILTER J1939Filter_TP;	//TP is Transfer Protocol: 0xE800, EA, EB, EC, ED -- also works for xEF00(proprietary messages)




#endif /* J1939_COMMUNICATION_H_ */