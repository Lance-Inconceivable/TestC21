/*
 * J1939_communication.c
 *
 * Created: 2/13/2020 5:00:12 PM
 *  Author: U103
 */ 
  
#include "J1939_communication.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cli.h"
#include <asf.h>
#include <string.h>
#include "UARTCommandConsole.h"
#include "can_utils.h"
#include <sys\_stdint.h>
#include "adctest.h"
#include "frequency.h"
#include "conf_error.h"
#include "UARTCommandConsole.h"


//#define 	  AA_SA 	   0x00 //declared in header
unsigned char SENSOR_SA    = 0xF9;	//Can be any address 0xF9 to 0xFD
uint64_t AS_NAME = 0x0000810082200000LL;
uint64_t contesting_NAME;
/*	Start Position	Length		Parameter Name 				Value(hex)
	1 - 3.1 		21 bits		Identity Number 			0xsnsns (Last 21bits of AA serial number)
	3.6 - 4 		11 bits 	Manufacturer Code			0x411 (EZ Lynk)
	5.1 			3 bits 		ECU Instance				0 (First Instance)
	5.4 			5 bits 		Function Instance			0 (First Instance)
	6 				8 bits 		Function					0x81 (Off-board diagnostic-service tool)
	7.1 			1 bit		Reserved					0 (N/A)
	7.2 			7 bits 		Vehicle System				0 (Non-specific System)
	8.1 			4 bits		Vehicle System Instance		0 (First Instance)
	8.5				3 bits 		Industry Group				0 (Global, applies to all)
	8.8 			1 bit 		Arbitrary Address Capable	0 (single address CA-Controller Application)
 */

int8_t J1939addressclaimed = 0; //ONLY allowed to use J1939 bus when an address has been successfully claimed.

#define CAN_EXTENDED_ID 1

struct can_tx_element           J1939_Tx_Packet;
struct can_rx_element_fifo_0    J1939_Rx_Data_Packet;
#define  RxDP  J1939_Rx_Data_Packet
//extern NU_EVENT_GROUP     CAN_Event_Group;

int J1939_defaultPriority = 6;

int J1939Send_Conn_Abort(uint8_t reason, uint32_t PGN);

extern int getJ1939can_dev(int id);
extern void fetchBoardSerialNumber(char *serial);
extern uint32_t Buf2UIntLSB(unsigned char *buff, int length);

/* Get current time in ms adding a constant offset (in ms) */
#define A_GET_MS(offset)   ((microtimer_read() >> 5) + (offset))
#define DelayMS(ms)        (ms / portTICK_PERIOD_MS))

TaskHandle_t J1939NMT_TaskHandle = (TaskHandle_t) 0;

void J1939_Set_Priority(int priority)
{
	J1939_defaultPriority = priority;
}

int J1939_Get_Priority(void)
{
	return J1939_defaultPriority;
}


int J1939_AddressArbitration(void)
{
	if(J1939_NetworkMgmt_AddressClaim() == J1939_ADDRESSCLAIMED)
		return SUCCESS;
	else
		return RETURN_ERROR;
}

int8_t getJ1939addressclaimed(void)
{
	return J1939addressclaimed;
}

void J1939_InitializeNAME(void)
{
	/* Grab last 21 bits of serial number and stick in last 21 bits of AS_NAME */
    //char serialnum[17];
	//fetchBoardSerialNumber(serialnum);
	//uint32_t TEMP = HexString2Long(serialnum+8, 16);
	uint32_t TEMP = 0xDEADBEEF;
	AS_NAME = (AS_NAME & 0xFFFFFFFFFFE00000LL) | (TEMP & 0x1FFFFF);
}

void set_contesting_NAME(uint8_t* buffer)
{
	contesting_NAME = Buf2UIntLSB(buffer, 4);
	contesting_NAME |= ((uint64_t)Buf2UIntLSB(buffer+4, 4) << 32);
}

/*Request Auto Agent's NAME from its source address*/ 
int J1939_AddressArbitration_RequestAutoAgentAddrClaim(unsigned int timeout)
{
	int status;
	int status_tx;
	unsigned char rxbuffer[0x8];
	unsigned int PGN_AddrClaim = 0x00EE00;

	J1939_ClearRXdmsgs();
//	status_tx = J1939_SendPGNRequest(PGN_AddrClaim, 0xFF, 0xFE); //Request for all claims on J1939 bus);
	status_tx = J1939_SendPGNRequest(PGN_AddrClaim, 0/*AA_SA*/, 0xFE); //Request for only Auto Agent on J1939 bus);
	status = J1939_Receive(PGN_AddrClaim, 0xFF, 0/*AA_SA*/, rxbuffer, timeout); //Waiting for the AA(addr 0xAA) to respond
	if(status > 0){
		status = SUCCESS;
	}else{
		debug_msg("J1939_AddrClaim check for AA fail: tx status ");
		printhex(status_tx, 1);
	}
	return status;
}

int8_t J1939_NetworkMgmt_AddressClaim(void)
{
	int status = SUCCESS;
	uint64_t othertool_NAME = 0;
//	unsigned int PGN_AddrClaim = 0x00EE00;
	unsigned char AS_namebuffer[8];
	unsigned char rxbuffer[0x8];
    int i;
    uint randomdelay;

    //No need to reclaim address
    if(J1939addressclaimed == J1939_ADDRESSCLAIMED)
      	return J1939_ADDRESSCLAIMED;

    if(J1939addressclaimed == J1939_NOTINITIALIZED)
    {
    	J1939_InitializeNAME();
    }

	for(i=0; i<8; i++)
	{
		AS_namebuffer[i] = AS_NAME >> (i*8);
	}
	
	J1939_ClearRXdmsgs();
	/* PGN 0xEA00 (59904): PGN data Request for Address Claimed. Data: 0x00EE00 PGN 60928 - Address claimed
	 * Request to see if AA source address has already been claimed.
	 * Request is not really needed though it could be useful to see if our address is already taken
	 */
//	J1939_SendPGNRequest(PGN_AddrClaim, 0xFF, 0xFE); //Global request for all from NULL address
//	J1939_SendPGNRequest(PGN_AddrClaim, SENSOR_SA, 0xFE); //Specific request for an address in use
//	NU_Sleep(25); //wait 250ms for responses

	J1939addressclaimed = J1939_NOTINITIALIZED;
	do{
		/* PGN 0xEE00 (60928): Address Claimed.  Data: NAME - 64 bit controller descriptor
		 * Request to claim address.
		 */
		J1939_Send(0x00EE00, 0xFF, SENSOR_SA, AS_namebuffer, 8); //Sent to global address from desired source address

		/* wait for address already claimed response, else assume it is ours */
		status = J1939_ReceivePacket(0x00EEFF00 | SENSOR_SA, rxbuffer, 250);	//Not required to wait 250ms, but do anyway
		if(status == SUCCESS)	//Packet received, check for lower/equal NAME priority and try another address
		{
			set_contesting_NAME(rxbuffer);
			if(AS_NAME > othertool_NAME) //we don't have priority, choose another.
			{
				if(SENSOR_SA >= 0xFD){ //exhausted all options, fail.
					J1939addressclaimed = J1939_CANNOTCLAIMADDRESS;
					SENSOR_SA = 0xFA;
				} else {
					randomdelay = rand() & 0xFF;
					randomdelay *= 600; //rand * 600 = delay in microseconds (.6ms is max time to send one message)
					microtimer_delayus(randomdelay);
					J1939_Send(0x00EE00, 0xFF, 0xFE, AS_namebuffer, 8); //Sent to global address from NULL address
					//Send new address claim after another random delay
					randomdelay = rand() & 0xFF;
					randomdelay *= 600; //rand * 600 = delay in microseconds (.6ms is max time to send one message)
					microtimer_delayus(randomdelay);
					++SENSOR_SA;
				}
			}
		}
		else{
			J1939addressclaimed = J1939_ADDRESSCLAIMED;
//			J1939_NetworkMgmt_Task_Init();
		}
	}while(J1939addressclaimed == J1939_NOTINITIALIZED);

	return J1939addressclaimed;
}

//extern NU_MEMORY_POOL *WiTune_Memory_Pool;
//static NU_TASK J1939_NetworkMgmt_Task;
//static NU_EVENT_GROUP J1939NetworkMgmtEvents;
//#define J1939_NetworkMgmt_STACKSIZE 3000
//#define J1939_NetworkMgmt_PRIORITY 15

#define J1939_NetworkMgmt_ADDCLAIM		1<<0
#define J1939_NetworkMgmt_REQADDCLAIM	1<<1

    /* Declare a variable to hold the created event group. */
    EventGroupHandle_t J1939NetworkMgmtEvents;

	
void J1939_NetworkMgmt_Task_Entry(void *argv)
{
//	uint vec=0;
	uint i;
	uint32_t ulFlagValue;
	BaseType_t xResult;
	unsigned char AS_namebuffer[8];
	uint64_t contesting_NAME_copy;
    uint randomdelay;
//	uint8_t J1939NetworkMgmtEvents = 0;
	
	debug_msg("J1939_NetworkMgmt_Task_Entry!\r\n");


    /* Attempt to create the event group. */
    J1939NetworkMgmtEvents = xEventGroupCreate();
    if( J1939NetworkMgmtEvents == NULL )
    {
        /* The event group was not created because there
		was insufficient FreeRTOS heap available. */
		debug_msg("J1939_NetworkMgmt_Task_Entry, fail to create group\r\n");
    }
	
    if(J1939addressclaimed == J1939_NOTINITIALIZED)
    {
    	J1939_InitializeNAME();
    }

	for(i=0; i<8; i++)
	{
		AS_namebuffer[i] = AS_NAME >> (i*8);
	}

	while(1)
	{	
		/* Wait to be notified of a flag. */
		xResult = xTaskNotifyWait(
		pdFALSE,		  /* Don't clear bits on entry. */
		-1,        /* Clear all bits on exit. */
		&ulFlagValue,	  /* Stores the notified value. */
		portMAX_DELAY );
		
		//vec = xEventGroupWaitBits(J1939NetworkMgmtEvents, -1, pdTRUE, pdFALSE, portMAX_DELAY);
		debug_msg("J1939_NetworkMgmt_Task Running! flags ");
		printhex(ulFlagValue, 1);
		if (ulFlagValue & J1939_NetworkMgmt_ADDCLAIM)  //in PSTATE_RUN, _DISABLED, or any state really.
		{
			if(1)//J1939addressclaimed == J1939_ADDRESSCLAIMED) //check if we already have an address and need to reply
			{
				contesting_NAME_copy = contesting_NAME; //so it is not overwritten
//				    //WiTune_Log(LOGLEV_INFO, "J1939_NetworkMgmt_Task: Found another device using SA!\r\n"
//				    		"SENSOR_SA 0x%X AS_NAME 0x%.16llX other_NAME 0x%.16llX\r\n", SENSOR_SA, AS_NAME, contesting_NAME_copy);

				randomdelay = rand() & 0xFF;
				randomdelay *= 600; //rand * 600 = delay in microseconds (.6ms is max time to send one message)
				microtimer_delayus(randomdelay);

				if(contesting_NAME_copy <= AS_NAME) //other CA has priority
				{
					debug_msg("J1939_NetworkMgmt_Task: Lost NAME priority, sending cannot claim address\r\n");
					J1939_Send(0x00EE00, 0xFF, 0xFE, AS_namebuffer, 8); //Sent to global address from NULL address
					J1939addressclaimed = J1939_CANNOTCLAIMADDRESS;
					randomdelay = rand() & 0xFF;
					randomdelay = randomdelay * 6 / 100; //rand * .06 = delay in hundredths of seconds (.6ms is max time to send one message)
					microtimer_delayus(randomdelay);
					++SENSOR_SA;
					//WiTune_Log(LOGLEV_INFO, "J1939_NetworkMgmt_Task: Starting Address claim process again with SA: 0x%.2X\r\n", SENSOR_SA);
					J1939_NetworkMgmt_AddressClaim();
					//J1939addressclaimed = J1939_NOTINITIALIZED;
				}
				else	//we have priority and resend address claim message
				{
					debug_msg("J1939_NetworkMgmt_Task: We have priority! Resending claim msg\r\n");
					J1939_Send(0x00EE00, 0xFF, SENSOR_SA, AS_namebuffer, 8); //Sent to global address from desired source address
				}
			}
		}
		if (ulFlagValue & J1939_NetworkMgmt_REQADDCLAIM)
		{
			if(1)//J1939addressclaimed == J1939_ADDRESSCLAIMED) //check if we already have an address and need to reply
			{
				debug_msg("J1939_NetworkMgmt_Task: Someone requesting our NAME to address claim.\r\n");
				J1939_Send(0x00EE00, 0xFF, SENSOR_SA, AS_namebuffer, 8); //Sent to global address from claimed source address
			}
		}
	}//while(1)
	
	vTaskDelete(J1939NMT_TaskHandle);
}

void J1939_NetworkMgmt_Task_Init(void)
{
	xTaskCreate(J1939_NetworkMgmt_Task_Entry,	/* Task entry point. */
	"J1939NMT",							/* Task name */
	configMINIMAL_STACK_SIZE *3,		/* Stack size */
	NULL,								/* The parameter passed to the task */
	tskIDLE_PRIORITY + 1,               /* The priority assigned to the task. */
	&J1939NMT_TaskHandle	            /* Not used.  Just illustrates creating sync handle */
	);
}

void J1939_NetworkMgmt_IncomingAddressClaim(void)
{
//	//WiTune_Log(LOGLEV_DBG, "J1939_NetworkMgmt: Address Claimed 0x%X\r\n", J1939_NetworkMgmt_ADDCLAIM);
//	xEventGroupSetBits(J1939NetworkMgmtEvents, J1939_NetworkMgmt_ADDCLAIM);
	xTaskNotify( J1939NMT_TaskHandle, J1939_NetworkMgmt_ADDCLAIM, eSetBits );
}

void J1939_NetworkMgmt_IncomingRequestforAddressesClaimed(void)
{
//	//WiTune_Log(LOGLEV_DBG, "J1939_NetworkMgmt: Request for addresses claimed 0x%X\r\n", J1939_NetworkMgmt_REQADDCLAIM);
//	xEventGroupSetBits(J1939NetworkMgmtEvents, J1939_NetworkMgmt_REQADDCLAIM);
	xTaskNotify( J1939NMT_TaskHandle, J1939_NetworkMgmt_REQADDCLAIM, eSetBits );
}


int J1939_Request_Data_Transfer(uint32_t PDU, unsigned char* data)
{
	int      status = SUCCESS;
//	uint    events = 0;
	//always use message priority 6, EDP - 0, DP - 0or1, depends on PDU

	PDU = (PDU & 0x01FFFFFF) | (J1939_defaultPriority << 26);
	J1939_Tx_Packet.T0.bit.ID = PDU;
	J1939_Tx_Packet.T0.bit.XTD = CAN_EXTENDED_ID;
	//J1939_Tx_Packet.can_msg_length = 8;	//Always DLC of 8 except for specific situations
	J1939_Tx_Packet.data[0] = data[0];
	J1939_Tx_Packet.data[1] = data[1];
	J1939_Tx_Packet.data[2] = data[2];
	J1939_Tx_Packet.data[3] = data[3];
	J1939_Tx_Packet.data[4] = data[4];
	J1939_Tx_Packet.data[5] = data[5];
	J1939_Tx_Packet.data[6] = data[6];
	J1939_Tx_Packet.data[7] = data[7];

	/* Request to send the data message. */
	status = can_send(PDU, data);
	//if (status == SUCCESS)
		//status = NU_Retrieve_Events(&CAN_Event_Group, CAN_TX_COMPLETE, NU_OR_CONSUME, &events, NU_SUSPEND);
	return status;
}

int J1939_ReceivePacket(uint32_t rxEID, uint8_t* buffer, unsigned int mstimeout)
{
	int status = SUCCESS;
	uint32_t timeWaiting = 0, timeWaitingEnd;
	int wait = 0;

	if(mstimeout == 0)
	{
//may be an issue --	wait = 0;	//dont' suspend, just check for messages.
		wait = 0;	//don't suspend, just check for messages.
		timeWaitingEnd = 0; //only loop once.
	}
	else
		timeWaitingEnd = A_GET_MS(mstimeout);

	do{
		RxDP.R0.reg = 0x00; //  Clean out previous CAN receive
		status = can_fifo_read(&RxDP, wait);	//Timeout should be 1250ms, but for receiving the correct message, not just for receiving any message.
		if(status == SUCCESS)
		{
			/* check pF/pS/sA, not Priority/Reserved/Data page 
			 * PF:PDU Format, PS: PDU Specific(or DA:Dest addr), SA: Source Address
			 * format: xxxP PPRD FFFF FFFF SSSS SSSS AAAA AAAA
			 */ 
			if( (RxDP.R0.bit.ID & 0x01FFFFFF) == (rxEID & 0x01FFFFFF))	//check pF/pS/sA, not Priority/Reserved/Data page - xxxP PPRD FFFF FFFF SSSS SSSS AAAA AAAA
			//if( (RxDP.R0.bit.ID & 0x00FFFF00) == (PGN<<8))	//check pF/pS, not Priority/Reserved/Data page/source Address - xxxP PPRD FFFF FFFF SSSS SSSS AAAA AAAA
			{												//Does not work with Extended Data Page
				CAN_Copy_RxBuf(RxDP.data, buffer, 8);
					return SUCCESS;
			}
		}
		timeWaiting = A_GET_MS(0);
	} while(timeWaiting < timeWaitingEnd);

	return RX_TIMEOUT;
}

/*******************************************************
 * Connection Mode Clear to Send (TP.CM_CTS): Destination Specific
 * Byte: 1 Control byte = 17, Destination Specific Clear_To_Send (CTS)
 * 2 Number of packets that can be sent. This value shall be no larger than the value in byte 5 of the RTS message.
 * 3 Next packet number to be sent
 * 4-5 Reserved for assignment by SAE, these bytes should
 *******************************************************/
int J1939_ReceiveTP_CTS(unsigned int rxEID, uint32_t PGN, unsigned char* totalPackets, unsigned char* currentPacket)
{
	int status = SUCCESS;
	uint32_t timeWaiting = 0, timeWaitingEnd;

	timeWaitingEnd = A_GET_MS(1250);
	do{
		RxDP.R0.reg = 0x00; 	// Clean out previous CAN receive
		status = can_fifo_read(&RxDP, 1);	//Timeout should be 1250ms, but for receiving the correct message, not just for receiving any message.
		if(status == SUCCESS)
		{
			if( (RxDP.R0.bit.ID & 0x01FFFFFF) == (rxEID & 0x01FFFFFF))
			{
				if( PGN == Buf2UIntLSB(RxDP.data+5,3) )	//PGN of the packeted message, verify it is the correct PGN
				{
					if(RxDP.data[0] == 0x11) 		//TP.CM_CTS: Control byte = 17, Destination Specific Clear_To_Send (CTS)
					{
						*totalPackets = RxDP.data[1];		//Max number of packets that can be sent
						*currentPacket = RxDP.data[2];		//Packet # to start sending.
						break;
					}
					/*ECM is trying to send a message the same time AA is trying to send a message.  what is the best solution? */
					if(RxDP.data[0] == 0x10) 		//TP.RTS: Control byte = 16, request to send
					{
						J1939Send_Conn_Abort(1, PGN);
						//WiTune_Log(LOGLEV_INFO, "CTS Connection abort! rx'd: 0x%.8X\r\n", AA_Buf2UInt(RxDP.data, 4));//wait three second timeout, status: %d\r\n", status);
						//NU_Sleep(300); //wait for problem to timeout, then try again
						return RX_UNEXPECTED_MSG;
					}
					if(RxDP.data[0] == 0xFF) 		//TP.Conn_Abort: Control byte = 255, Connection Abort
					{
						return RX_UNEXPECTED_MSG;
					}
				}
			}
		}
		timeWaiting = A_GET_MS(0);

	} while(timeWaiting <= timeWaitingEnd);

	if(timeWaiting > timeWaitingEnd){
		status = TX_ERROR;
		//WiTune_Log(LOGLEV_ERR, "ReceiveTP_CTS timeout! RxDP.R0.bit.ID 0x%.8lX, data:  0x%.8lX, time %ld timeEnd %ld\r\n",
		//						RxDP.R0.bit.ID, AA_Buf2UInt(RxDP.data,4), timeWaiting, timeWaitingEnd);
	}

	return status;
}

/*******************************************************
 * End of Message Acknowledgment (TP.CM_EndOfMsgACK): Destination Specific
 * Byte: 1 Control byte = 19, End_of_Message Acknowledge
 * 2,3 Total message size, number of bytes
 * 4 Total number of packets
 * 5 Reserved for assignment by SAE, this byte should be filled with FF16
 * 6-8 Parameter Group Number of the packeted message
 *******************************************************/
int J1939_ReceiveTP_EndofMsgACK(unsigned int rxEID, uint32_t PGN, uint16_t* totalBytes, unsigned char* totalPackets)
{
	int status = SUCCESS;
	uint32_t timeWaiting=0, timeWaitingEnd;

	timeWaitingEnd = A_GET_MS(1250);
	do{
		RxDP.R0.bit.ID = 0x00; 	// Clean out previous CAN receive
		status = can_fifo_read(&RxDP, 1);	//Timeout should be 1250ms, but for receiving the correct message, not just for receiving any message.
		if(status == SUCCESS)
		{
			if( (RxDP.R0.bit.ID & 0x01FFFFFF) == rxEID)
			{
				if( PGN == Buf2UIntLSB(RxDP.data+5,3) )	//PGN of the packeted message, verify it is the correct PGN
				{
					if(RxDP.data[0] == 0x13) 			//TP.CM_EndofMsgACK: Control byte = 19, End_of_Message Acknowledge
					{
						*totalBytes = (((unsigned int)RxDP.data[2])<<8) | RxDP.data[1];		//Total bytes received
						*totalPackets = RxDP.data[3];		//Total Packets received
						break;
					}
					if(RxDP.data[0] == 0xFF) 	//TP.Conn_Abort: Control byte = 255, Connection Abort
					{
						status = RX_UNEXPECTED_MSG;
						break;
					}
				}
			}
		}
		timeWaiting = A_GET_MS(0);

	} while(timeWaiting <= timeWaitingEnd);

	if(timeWaiting > timeWaitingEnd)
		status = TX_ERROR;

	return status;
}


/*******************************************************
 * End of Message Acknowledgment (TP.CM_EndOfMsgACK): Destination Specific
 * Byte: 1 Control byte = 255, Connection Abort
 * 2 Connection Abort reason
 * 3-5 Reserved for assignment by SAE, these bytes should be filled with FF16
 * 6-8 Parameter Group Number of the packeted message
 *******************************************************/
int J1939Send_Conn_Abort(uint8_t reason, uint32_t PGN)
{
	int status = SUCCESS;
	//uint    events = 0;
	J1939_Tx_Packet.data[0] = 0xFF; 		//Control byte = 255, Connection Abort
	J1939_Tx_Packet.data[1] = reason;		//Connection Abort reason; 3 - timeout
	J1939_Tx_Packet.data[2] = 0xFF;		//Reserved for assignment by SAE, these bytes should be filled with FF
	J1939_Tx_Packet.data[3] = 0xFF;		//Reserved for assignment by SAE, these bytes should be filled with FF
	J1939_Tx_Packet.data[4] = 0xFF;		//Reserved for assignment by SAE, these bytes should be filled with FF
	J1939_Tx_Packet.data[5] = (unsigned char)PGN;	//Parameter Group Number of the packeted message in LSB
	J1939_Tx_Packet.data[6] = (unsigned char)(PGN>>8);
	J1939_Tx_Packet.data[7] = (unsigned char)(PGN>>16);
	status = can_send(J1939_Tx_Packet.T0.reg, J1939_Tx_Packet.data);
	//if (status == SUCCESS)
		//status = NU_Retrieve_Events(&CAN_Event_Group, CAN_TX_COMPLETE, NU_OR_CONSUME, &events, 5);

	//WiTune_Log(LOGLEV_ERR, "\r\nSending connection abort! status %d, reason %u PGN 0x%X\r\n", status, reason, PGN);
	return status;
}

int J1939_SendPGNRequest(unsigned int PGN, unsigned char destination, unsigned char source)
{
	int      status = SUCCESS;
	//uint    events = 0;

	J1939_Tx_Packet.T0.bit.ID = 0xEA0000 | (((unsigned int)destination)<<8) | source | (J1939_defaultPriority << 26);
	J1939_Tx_Packet.T0.bit.XTD = 1; //CAN_EXTENDED_ID;
	J1939_Tx_Packet.T0.bit.RTR = 0; //CAN_DATA_MSG;
	//J1939_Tx_Packet.can_msg_length = 3;	//Always DLC of 3 on PGN request
	J1939_Tx_Packet.data[0] = (unsigned char)PGN;
	J1939_Tx_Packet.data[1] = (unsigned char)(PGN>>8);
	J1939_Tx_Packet.data[2] = (unsigned char)(PGN>>16);
	J1939_Tx_Packet.data[3] = 0xFF;
	J1939_Tx_Packet.data[4] = 0xFF;
	J1939_Tx_Packet.data[5] = 0xFF;
	J1939_Tx_Packet.data[6] = 0xFF;
	J1939_Tx_Packet.data[7] = 0xFF;

	status = can_send(J1939_Tx_Packet.T0.reg, J1939_Tx_Packet.data);
	//if (status == SUCCESS)
		//status = NU_Retrieve_Events(&CAN_Event_Group, CAN_TX_COMPLETE,NU_OR_CONSUME, &events, 5);
	return status;
}

#if 0
int J1939_PGNDataRequest(unsigned int PGN, unsigned char ecm, unsigned char tool, unsigned char *RxMesBuff)
{
	int length, status_tx;
	status_tx = J1939_SendPGNRequest(PGN, ecm, tool);
	length = J1939_Receive(PGN, tool, ecm, RxMesBuff, 1260);
	if(length == 8)
	{
		/* J1939-21 5.4.4 */
		if(PGN == Buf2UIntLSB(RxMesBuff+5,3))
		{
			if(RxDP.data[0] == 0x00)      // 0 - Positive Acknowledgment (ACK)
				length = RX_ACK;
			else if(RxDP.data[0] == 0x01) // 1 - Negative Acknowledgment (NACK)
				length = RX_NACK;
			else if(RxDP.data[0] == 0x02) // 2 - Access Denied
				length = RX_SECURITY_ERROR;
			else if(RxDP.data[0] == 0x03) // 3 - Cannot Respond
				length = RX_CANNOT_RESPOND;				
		}
	}
	if (status_tx < 0 || length < 0){
	}
	return length;
}
#endif

int J1939_Send(unsigned int PGN, unsigned char destination, unsigned char source, unsigned char *data, unsigned int dataLength)
{
	int status = SUCCESS;
	//uint events = 0;
	unsigned char packet_data[8];
    //unsigned int byteCount = 0;
    unsigned char packetCount=1, maxPackets=0xFF, numPackets=0;
    int bytesLeft = 0;
    uint16_t bytesReceived = 0;
    uint8_t packetsReceived;
    //const uint32_t EID_PGN		= 0x00000000 | (PGN<<8) | source;	//PGN is two bytes
    //const uint32_t EID_ACKM		= 0x00E80000 | 0xFF00 | source;	//ACK or NACK, though generally a NACK.
    //const uint32_t EID_RQST		= 0x00EA0000 | 0xFF00 | source; //NOT A RESPONSE
    const uint32_t EID_TPCM		    = 0x00EC0000 | (((uint)destination)<<8) | source;//Transport Protocol - Connection Management
    const uint32_t EID_TPCM_Rx	    = 0x00EC0000 | (((uint)source)<<8) | destination;//Transport Protocol - Connection Management
    //const uint32_t EID_TPCMglobal	= 0x00EC0000 | 0xFF00 | source; //0xFF00 = global address, could be BAM message(data byte 1 is 0x20)
    uint32_t EID_DATA				= 0x00EB0000 | (((uint)destination)<<8) | source;	//Transport Protocol - Data Transfer
	uint32_t EID;


	if(dataLength > 1785)	//0xFFF) J1939-21:5.10.1.1 says 1785 is max length, but proprietary stuff could be longer
		return -1;
	
	if(dataLength <= 8)	// no Transport Protocol involved, just send it!
	{
		EID = (J1939_defaultPriority << 26) | (PGN<<8) | (((unsigned int)destination)<<8) | source;
		CAN_Copy_TxBuf(data, dataLength, packet_data, 0, 0xFF);

		status = can_send(EID, packet_data);	// Request to send the data message.
		//if (status == SUCCESS)
			//status = NU_Retrieve_Events(&CAN_Event_Group, CAN_TX_COMPLETE,NU_OR_CONSUME, &events, 5);
		return SUCCESS;
	}
	else	//Will need handshaking to properly send multipacket message.
	{
		numPackets = dataLength/7;
		if(dataLength%7)
			++numPackets;
//Send TP.CM_RTS (destination Specific)0xEC00 0x10
		packet_data[0] = 0x10;						//byte 1:	0x10 - Control Byte, Destination Specific Request_To_Send (RTS)
		packet_data[1] = (unsigned char)dataLength;			//byte 2,3:	total size, # of bytes
		packet_data[2] = (unsigned char)(dataLength>>8);
		packet_data[3] = numPackets;					//byte 4:	Total # of packets
		packet_data[4] = 0xFF;						//byte 5:	max # or packets that can be sent in response to one CTS, 0xFF indicates no limit
		packet_data[5] = (unsigned char)(PGN);		//byte 6-8: PGN in LSB
		packet_data[6] = (unsigned char)(PGN>>8);
		packet_data[7] = (unsigned char)(PGN>>16);
		J1939_Request_Data_Transfer(EID_TPCM, packet_data);
		if(destination != 0xFF)	//don't need a response for BAM, skip this part
		{
			status = J1939_ReceiveTP_CTS(EID_TPCM_Rx, PGN, &maxPackets, &packetCount);
			/* if(maxPacket < numPackets)
			 * maxPacket should be tested against the message length(in packets) and if the max allowed is shorter than the message
			 * then the message it will have to be sent over multiple multipacket messages. This is not implemented... yet.... or ever... lol.
			 */
			if(status != SUCCESS)
			{
				return status;
			}
		}

		bytesLeft = dataLength;
		do{
			packet_data[0] = packetCount;
			if(bytesLeft >= 7)
			{
				CAN_Copy_TxBuf(data+(packetCount-1)*7, 7, packet_data, 1, 0xFF);
				bytesLeft = bytesLeft - 7;
			}
			else
				CAN_Copy_TxBuf(data+(packetCount-1)*7, bytesLeft, packet_data, 1, 0xFF);

			status = can_send(EID_DATA, packet_data);
			if(status == SUCCESS)
			{
				++packetCount;
			}
		}while (packetCount <= numPackets);

		if(destination == 0xFF)//don't need a response for BAM
		{
			return SUCCESS;
		}

		status = J1939_ReceiveTP_EndofMsgACK(EID_TPCM_Rx, PGN, &bytesReceived, &packetsReceived);

		if (status == SUCCESS)
		{
			if(bytesReceived == dataLength)	//or (packetCount-1 == packetsReceived)
				status = SUCCESS;
			else
				status = TX_ERROR;

			return status;
		}

	}

	return status;
}

int J1939_Receive(unsigned int PGN, unsigned char destination, unsigned char source, unsigned char *RxMesBuff, uint16_t timeout)
{
	int status = SUCCESS;
	unsigned int  messageType = 0;
	//unsigned char packet_data[8];
    int numBytes = -1, byteCount = 0;
	uint32_t timeWaiting = 0, timeWaitingEnd;
    unsigned int packetCount, numPackets;//, maxPackets;
    uint32_t msgPGN = 0;
    uint32_t Priority = 0x00000000;
    const uint32_t EID_PGN		  = 0x00000000 | (PGN<<8) | (((unsigned int)destination)<<8) | source;	//PGN is two bytes, low byte is destination
    const uint32_t EID_ACKMglobal = 0x00E80000 | 0xFF00 | source;	//ACK or NACK, though generally a NACK.
    const uint32_t EID_ACKM		  = 0x00E80000 | (((unsigned int)destination)<<8) | source;	//ACK or NACK, though generally a NACK.
    //const uint32_t EID_RQST     = 0x00EA0000 | 0xFF00 | source; //NOT A RESPONSE
    uint32_t EID_DATA		      = 0x00EB0000 | (((unsigned int)destination)<<8) | source;	//Transport Protocol - Data Transfer
    const uint32_t EID_TPCM		  = 0x00EC0000 | (((unsigned int)destination)<<8) | source;//Transport Protocol - Connection Management
    const uint32_t EID_TPCMglobal = 0x00EC0000 | 0xFF00 | source; //0xFF00 = global address, could be BAM message(data byte 1 is 0x20)

//EID =	 0xXXPGN|DA (<= 8byte response, rest are RTC/CTS or BAM protocol)
//or 0xXXEC|DA|SA(destinations specific)
//or 0xXXEC|FF|SA(global)
//or 0xXXE8|FF|SA(ack/nack only)

    timeWaitingEnd = A_GET_MS(timeout);
    do{
    	RxDP.R0.reg = 0x00; 	// Clean out previous CAN receive
    	status = can_fifo_read(&RxDP, 1);	//Timeout should be 1250ms, but for receiving the correct message, not just for receiving any message.
    	if(status == SUCCESS)
    	{
    		if( (RxDP.R0.bit.ID & 0x01FFFFFF) == EID_PGN)
    		{
    			if( ((RxDP.R0.bit.ID>>16) & 0x00FF) >= 0x00F0) //PDU2 format
    				messageType = 2;	//PDU2 format; PF 240-255, PS(GE) Group Extension, only globaly addressed
    			else
    				messageType = 1;	//PDU1 format; PF 0-239, PS - destination addr(specific or global(FF))
    		}
    		else if( (RxDP.R0.bit.ID & 0x01FFFFFF) == EID_ACKMglobal || 
    				 (RxDP.R0.bit.ID & 0x01FFFFFF) == EID_ACKM)
    		{
    	    	/* byte 6-8 PGN requested */ //Make sure it is the response to the requested message
    	    	msgPGN = Buf2UIntLSB(RxDP.data+5,3);	//PGN of the packeted message, verify it is the correct PGN
    	    	if(msgPGN == PGN)
    				messageType = 3;
    		}
    		else if( (RxDP.R0.bit.ID & 0x01FFFFFF) == EID_TPCM)
    		{
    	    	msgPGN = Buf2UIntLSB(RxDP.data+5,3);	//PGN of the packeted message, verify it is the correct PGN
    	    	if(msgPGN == PGN && RxDP.data[0] == 0x10)	//(0x10 = RTS, anything else is invalid at this point)
    				messageType = 4;
    		}
    		else if( (RxDP.R0.bit.ID & 0x01FFFFFF) == EID_TPCMglobal)
    		{
    	    	msgPGN = Buf2UIntLSB(RxDP.data+5,3);	//PGN of the packeted message, verify it is the correct PGN
    	    	if(msgPGN == PGN)
    	    	{
					if(RxDP.data[0] == 0x20) //BAM message
					{
						messageType = 5;
						EID_DATA = EID_DATA | 0x0000FF00; //Global data message
					}
					else
						messageType = 6;	//?????not sure
    	    	}
    		}
    		else
    			messageType = 0;
    	}
    	timeWaiting = A_GET_MS(0);

    } while(messageType == 0 && timeWaiting < timeWaitingEnd);

    if(messageType == 1 || messageType == 2)	//PDU1 or PDU2 response
    {
    	CAN_Copy_RxBuf(RxDP.data,RxMesBuff,8);
    	byteCount = 8;
    	numBytes = 8;
    }
    else if( messageType == 3)	//Acknowledgment response(normally negative)
    {
    	/* byte 1
    	0 - Positive Acknowledgment (ACK)
    	1 - Negative Acknowledgment (NACK)
    	2 - Access Denied
    	3 - Cannot Respond
    	*/
    	CAN_Copy_RxBuf(RxDP.data,RxMesBuff,8);
    	byteCount = 8;
    	numBytes = 8;
    }
    else if(messageType >= 4)	//TP.CM - Transport Protocol - Connection Management
	{
		numBytes = Buf2UIntLSB(RxDP.data+1,2);	//# of bytes that will be sent.
		numPackets = RxDP.data[3];				//# of packets that the bytes will be sent in.
		Priority = RxDP.R0.bit.ID & (7 << 26);
		//maxPackets = RxDP.data[4];				//Maximum number of packets that can be sent in response to one CTS. FF16 indicates that no limit exists for the originator.

		if(messageType == 4)//if(RxDP.data[0] == 0x10)	//TP.CM_RTS
		{
			/* Send CTS response to begin data transfer */ //24 26
			J1939_Tx_Packet.T0.bit.ID = 0xEC0000 | Priority | (((unsigned int)source)<<8) | destination;	//Source and destination are switch, they are receive relative
			//J1939_Tx_Packet.can_msg_length = 8;
			J1939_Tx_Packet.T0.bit.XTD = 1; //CAN_EXTENDED_ID;
			J1939_Tx_Packet.T0.bit.RTR = 0; //CAN_DATA_MSG;
			J1939_Tx_Packet.data[0] = 0x11; 			//Control byte = 17, Destination Specific Clear_To_Send (CTS)
			J1939_Tx_Packet.data[1] = numPackets;		//Number of packets that can be sent. This value shall be no larger than the value in byte 5 of the RTS message
			J1939_Tx_Packet.data[2] = 0x01;			//Next packet number to be sent.  IF we could recover in the middle of a transmission we would use this differently
			J1939_Tx_Packet.data[3] = 0xFF;			//Reserved for assignment by SAE, these bytes should be filled with FF
			J1939_Tx_Packet.data[4] = 0xFF;			//Reserved for assignment by SAE, these bytes should be filled with FF
			J1939_Tx_Packet.data[5] = (unsigned char)PGN;	//Parameter Group Number of the packeted message in LSB
			J1939_Tx_Packet.data[6] = (unsigned char)(PGN>>8);
			J1939_Tx_Packet.data[7] = (unsigned char)(PGN>>16);
			status = can_send(J1939_Tx_Packet.T0.reg, J1939_Tx_Packet.data);
			//if (status == SUCCESS)
				//status = NU_Retrieve_Events(&CAN_Event_Group, CAN_TX_COMPLETE, NU_OR_CONSUME, &events, NU_SUSPEND);
		}

		//wait for data packets... 00EB0000
		packetCount=0;
		byteCount=0;

		while(byteCount < numBytes)//(packetCount < numPackets)
		{
			//RxDP.R0.bit.ID = 0;
		    timeWaitingEnd = A_GET_MS(200);
			do{
				RxDP.R0.bit.ID = 0;
				status = can_fifo_read(&RxDP, 1);//(int)timeOut);
				timeWaiting = A_GET_MS(0);
			}while((RxDP.R0.bit.ID & 0x01FFFFFF) != EID_DATA && timeWaiting <= timeWaitingEnd);

			if((RxDP.R0.bit.ID & 0x01FFFFFF) != EID_DATA)//status != SUCCESS)
			{
				if (messageType == 4)	//Send //TP.Conn_Abort
					J1939Send_Conn_Abort(4, PGN);

				return RX_TIMEOUT;	//RX timeout
			}
			CAN_Copy_RxBuf(RxDP.data+1, (RxMesBuff + byteCount), 7);
			byteCount += 7;
			packetCount++;
		}
		if(messageType == 4) 	// if(RxDP.data[0] == 0x10)	//TP.CM_RTS
		{
			J1939_Tx_Packet.data[0] = 0x13; 						//Control byte = 19, End_of_Message Acknowledge
			J1939_Tx_Packet.data[1] = (unsigned char)numBytes;	//Total message size, number of bytes, LSB
			J1939_Tx_Packet.data[2] = numBytes>>8;				//Total message size, number of bytes, MSB
			J1939_Tx_Packet.data[3] = packetCount;				//Total number of packets
			J1939_Tx_Packet.data[4] = 0xFF;						//Reserved for assignment by SAE, this byte should be filled with FF
			J1939_Tx_Packet.data[5] = (unsigned char)PGN;			//Parameter Group Number of the packeted message
			J1939_Tx_Packet.data[6] = (unsigned char)(PGN>>8);
			J1939_Tx_Packet.data[7] = (unsigned char)(PGN>>16);
			status = can_send(J1939_Tx_Packet.T0.reg, J1939_Tx_Packet.data);
			//if (status == SUCCESS)
				//status = NU_Retrieve_Events(&CAN_Event_Group, CAN_TX_COMPLETE, NU_OR_CONSUME, &events, NU_SUSPEND);
		}
	}
    return numBytes;	//Is -1 if nothing received.
}

void J1939_ClearRXdmsgs(void)
{
	can_reset_fifo();
}

void send_GeneralInfo(void)
{
	/*
	PGN 0x00FF00 + SA(source address of the AS).
	Byte 1:		0x12 - Information message ID
	Byte 2-3:	Current firmware version
	Byte 4-5:	Current transmit rate in ms
	Byte 6: 	Number of sensors connected
	Byte 7:		Reserved
	Byte 8:		Reserved
	*/
}

void send_SensorData(void)
{
	/*
	PGN 0x00FF01 + SA(source address of the AS).
	Byte 1.1-1.4:	data type (0 - currently only one data type)
	Byte 1.5-1.6: 	sensor number (0-3)
	Byte 1.7-1.8: 	reserved
	Byte 2: 	    reserved
	Byte 3-4:	    sensor data (0 - 65535)
	Byte 5.1-5.4:	data type (0 - currently only one data type)
	Byte 5.5-5.6: 	sensor number (0-3)
	Byte 5.7-5.8: 	reserved
	Byte 6: 	    reserved
	Byte 7-8:	    sensor data (0 - 65535)
	*/
}