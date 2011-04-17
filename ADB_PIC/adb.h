/*

*/
#ifndef __ADB_H__
#define __ADB_H__

#define MAX_PAYLOAD 40

#define A_SYNC 0x434e5953
#define A_CNXN 0x4e584e43
#define A_OPEN 0x4e45504f
#define A_OKAY 0x59414b4f
#define A_CLSE 0x45534c43
#define A_WRTE 0x45545257

typedef struct {
    unsigned long command;       /* command identifier constant      */
    unsigned long arg0;          /* first argument                   */
    unsigned long arg1;          /* second argument                  */
    unsigned long data_length;   /* length of payload (0 is allowed) */
    unsigned long data_crc32;    /* crc32 of data payload            */
    unsigned long magic;         /* command ^ 0xffffffff             */
} ADB_HEADER;

typedef struct {
	ADB_HEADER header; 
	unsigned char payload[MAX_PAYLOAD];       /* payload */
} ADB_MESSAGE;

int adb_connect(unsigned long version,unsigned long maxdata,char* identity);
int adb_open(unsigned long local_id,char* destination);
int adb_close(unsigned long local_id,unsigned long remote_id);
int adb_write(unsigned long remote_id,char* data);
int adb_ready(unsigned long local_id,unsigned long remote_id);
int adb_sync(unsigned long online,unsigned long sequence);
void adb_header(char* d, ADB_MESSAGE* adbmsg);

#endif
