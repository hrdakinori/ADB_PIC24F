/*

*/

#include <string.h>
#include <stdlib.h>
#include "adb.h"

#include "GenericTypeDefs.h"
#include "HardwareProfile.h"
#include "usb_config.h"
#include "USB/usb.h"
#include "USB/usb_host_generic.h"


extern BYTE deviceAddress;

// Write Message
int adb_writeMessage(unsigned long command, unsigned long arg0, unsigned long arg1, unsigned long length, unsigned char * data)
{
	unsigned long count = 0;
	unsigned long sum = 0;
	unsigned char* x;
	unsigned char rcode;
	ADB_MESSAGE msg;

	// checksum
    count = length;
    x = data;
    while(count-- > 0) sum += *x++;

	msg.header.command = command;
	msg.header.arg0 = arg0;
	msg.header.arg1 = arg1;
	msg.header.data_length = length;
	msg.header.data_crc32 = sum;
	msg.header.magic = command ^ 0xffffffff;

	#ifdef DEBUG_MODE
	adb_header("W",&msg);
	#endif

	memcpy(msg.payload,data,length > MAX_PAYLOAD?MAX_PAYLOAD:length);

	rcode = USBHostGenericWrite2(deviceAddress,(unsigned char*)&msg,sizeof(ADB_HEADER)+length);

	return rcode;
}

// Write String Message
int adb_writeStringMessage(unsigned long command, unsigned long arg0, unsigned long arg1, char * str)
{
	return adb_writeMessage(command, arg0, arg1, strlen(str) + 1, (unsigned char*)str);
}

// CONNECT(version, maxdata, "system-identity-string")
int adb_connect(unsigned long version,unsigned long maxdata, char* identity)
{
	return adb_writeStringMessage(A_CNXN, version, maxdata, identity);
}

// OPEN(local-id, 0, "destination")
int adb_open(unsigned long local_id, char* destination)
{
	return adb_writeStringMessage(A_OPEN, local_id, 0, destination);
}

// CLOSE(local-id, remote-id, "")
int adb_close(unsigned long local_id,unsigned long remote_id)
{
	return adb_writeStringMessage(A_CLSE, local_id, remote_id, "");
}

// WRITE(0, remote-id, "data")
int adb_write(unsigned long remote_id, char* data)
{
	return adb_writeStringMessage(A_WRTE, 0, remote_id, data);
}

// READY(local-id, remote-id, "")
int adb_ready(unsigned long local_id,unsigned long remote_id)
{
	return adb_writeStringMessage(A_OKAY, local_id, remote_id, "");
}

// SYNC(online, sequence, "")
int adb_sync(unsigned long online,unsigned long sequence)
{
	return adb_writeStringMessage(A_SYNC, online, sequence, "");
}

#ifdef DEBUG_MODE
void adb_header(char* d,ADB_MESSAGE* adbmsg)
{
	UART2PrintString( d );
	switch(adbmsg->header.command)
	{
		case A_SYNC:
			UART2PrintString( " SYNC " );
			UART2PutHexDWord(adbmsg->header.arg0);
			UART2PrintString( " " );
			UART2PutHexDWord(adbmsg->header.arg1);
			UART2PrintString( "\r\n" );
		break;
		case A_CNXN:
			UART2PrintString( " CNXN " );
			UART2PutHexDWord(adbmsg->header.arg0);
			UART2PrintString( " " );
			UART2PutHexDWord(adbmsg->header.arg1);
			UART2PrintString( "\r\n" );
		break;
		case A_OPEN:
			UART2PrintString( " OPEN " );
			UART2PutHexDWord(adbmsg->header.arg0);
			UART2PrintString( " " );
			UART2PutHexDWord(adbmsg->header.arg1);
			UART2PrintString( "\r\n" );
		break;
		case A_OKAY:
			UART2PrintString( " OKAY " );
			UART2PutHexDWord(adbmsg->header.arg0);
			UART2PrintString( " " );
			UART2PutHexDWord(adbmsg->header.arg1);
			UART2PrintString( "\r\n" );
		break;
		case A_CLSE:
			UART2PrintString( " CLSE " );
			UART2PutHexDWord(adbmsg->header.arg0);
			UART2PrintString( " " );
			UART2PutHexDWord(adbmsg->header.arg1);
			UART2PrintString( "\r\n" );
		break;
		case A_WRTE:
			UART2PrintString( " WRTE " );
			UART2PutHexDWord(adbmsg->header.arg0);
			UART2PrintString( " " );
			UART2PutHexDWord(adbmsg->header.arg1);
			UART2PrintString( "\r\n" );
		break;
		default:
			UART2PrintString( " ???? " );
			UART2PutHexDWord(adbmsg->header.command);
			UART2PrintString( " " );
			UART2PutHexDWord(adbmsg->header.arg0);
			UART2PrintString( " " );
			UART2PutHexDWord(adbmsg->header.arg1);
			UART2PrintString( "\r\n" );
		break;
	}
}
#endif

