/*
 * IMUReader.c
 *
 *  Created on: Feb 22, 2015
 *      Author: will
 *
 *      gcc -o IMUReader IMUReader.c -ldl -lpthread libftd2xx.a
 */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "ftd2xx.h"
#include <stdint.h>

#define BUF_SIZE 0x60

#define MAX_DEVICES		1

typedef struct UM7_packet_struct {
	uint8_t Address;
	uint8_t PT;
	uint16_t Checksum;
	uint8_t data_length;
	uint8_t data[30];
} UM7_packet;

UM7_packet new_packet;
FT_STATUS ftStatus;
FT_HANDLE ftHandle[MAX_DEVICES];

int parse_serial_data(uint8_t* rx_data, uint8_t rx_length,
		UM7_packet* packet);

double printBytesRead(uint8_t* rx_data, uint8_t rx_data_length);

int initializeIMUReader() {

	char * pcBufLD[MAX_DEVICES + 1];
	char cBufLD[MAX_DEVICES][64];
	int iNumDevs = 0;
	int i;

	for (i = 0; i < MAX_DEVICES; i++) {
		pcBufLD[i] = cBufLD[i];
	}

	pcBufLD[MAX_DEVICES] = NULL;

	ftStatus = FT_ListDevices(pcBufLD, &iNumDevs,
	FT_LIST_ALL | FT_OPEN_BY_SERIAL_NUMBER);

	if (ftStatus != FT_OK) {
		printf("Error: FT_ListDevices returned %d\n", (int) ftStatus);
		return 1;
	}

	for (i = 0; ((i < MAX_DEVICES) && (i < iNumDevs)); i++) {
		printf("Device %d Serial Number - %s\n", i, cBufLD[i]);
	}

	/* Setup */
	if ((ftStatus = FT_OpenEx(cBufLD[0], FT_OPEN_BY_SERIAL_NUMBER, &ftHandle[0]))
			!= FT_OK) {
		/*
		 This can fail if the ftdi_sio driver is loaded
		 use lsmod to check this and rmmod ftdi_sio to remove
		 also rmmod usbserial
		 */
		printf("Error: FT_OpenEx returned %d for device %d\n", (int) ftStatus,
				0);
		return 1;
	}

	printf("Opened device %s\n", cBufLD[0]);

	if ((ftStatus = FT_SetBaudRate(ftHandle[0], 115200)) != FT_OK) {
		printf("Error: FT_SetBaudRate returned %d, cBufLD[i] = %s\n",
				(int) ftStatus, cBufLD[0]);
		return 1;
	}
	return 0;
}

void *readIMUData(void *arg){
		/* Read */
	while(1){

		double * doublePtr =  (double*) arg;
		uint8_t * pcBufRead = NULL;
		DWORD dwRxSize = 0;
		DWORD dwBytesRead;
		dwRxSize = 0;
		while ((dwRxSize < BUF_SIZE) && (ftStatus == FT_OK)) {
			ftStatus = FT_GetQueueStatus(ftHandle[0], &dwRxSize);
		}

		if (ftStatus == FT_OK) {
			pcBufRead = (uint8_t*) realloc(pcBufRead, dwRxSize);
			if ((ftStatus = FT_Read(ftHandle[0], pcBufRead, dwRxSize,
					&dwBytesRead)) != FT_OK) {
				printf("Error: FT_Read returned %d\n", (int) ftStatus);
			} else {
//				printf("FT_Read read %d bytes\n", (int) dwBytesRead);
				*doublePtr = printBytesRead(pcBufRead, (uint8_t) dwBytesRead);
			}
		} else {
			printf("Error: FT_GetQueueStatus returned %d\n", (int) ftStatus);
		}
	}
		// sleep(1);
		return NULL;
	}

//uint8_t parse_serial_data(uint8_t* rx_data, uint8_t rx_length, UM7_packet* packet_data){
int parse_serial_data(uint8_t* rx_data, uint8_t rx_length,
		UM7_packet* packet) {
	uint8_t index = 0;

	// Make sure that the data buffer provided is long enough to contain a full packet
	// The minimum packet length is 7 bytes

	if (rx_length < 7) {
		printf("ERROR: Packet length too short. %d \n", -1);
		return 1;
	}

	uint8_t limit = (rx_length - 2);
	// Try to find the ‘snp’ start sequence for the packet
	for (index = 0; index < limit; index++) {

		// Check for ‘snp’. If found, immediately exit the loop
		if (rx_data[index] == 's' && rx_data[index + 1] == 'n'
				&& rx_data[index + 2] == 'p') {
			break;
		}

	}
	uint8_t packet_index = index;
	uint8_t limit2 = rx_length - packet_index;
	// Check to see if the variable ‘packet_index’ is equal to (rx_length - 2). If it is, then the above
	// loop executed to completion and never found a packet header.
	if (packet_index == limit) {
		return 2;
	}
	// If we get here, a packet header was found. Now check to see if we have enough room
	// left in the buffer to contain a full packet. Note that at this point, the variable ‘packet_index’
	// contains the location of the ‘s’ character in the buffer (the first byte in the header)
	if (limit2 < 7) {
		return 3;
	}
	// We’ve found a packet header, and there is enough space left in the buffer for at least
	// the smallest allowable packet length (7 bytes). Pull out the packet type byte to determine
	// the actual length of this packet
	uint8_t PT = rx_data[packet_index + 3];

	// Do some bit-level manipulation to determine if the packet contains data and if it is a batch
	// We have to do this because the individual bits in the PT byte specify the contents of the
	// packet.
	uint8_t packet_has_data = (PT >> 7) & 0x01; // Check bit 7 (HAS_DATA)
	uint8_t packet_is_batch = (PT >> 6) & 0x01; // Check bit 6 (IS_BATCH)
	uint8_t batch_length = (PT >> 2) & 0x0F; // Extract the batch length (bits 2 through 5)

	// Now finally figure out the actual packet length
	uint8_t data_length = 0;
	if (packet_has_data) {
		if (packet_is_batch) {
			// Packet has data and is a batch. This means it contains ‘batch_length' registers, each
			// of which has a length of 4 bytes
			data_length = 4 * batch_length;
		} else // Packet has data but is not a batch. This means it contains one register (4 bytes)
		{
			data_length = 4;
		}
	} else // Packet has no data
	{
		data_length = 0;
	}
	// At this point, we know exactly how long the packet is. Now we can check to make sure
	// we have enough data for the full packet.
	if (limit2 < (data_length + 5)) {
		return 3;
	}
	// If we get here, we know that we have a full packet in the buffer. All that remains is to pull
	// out the data and make sure the checksum is good.
	// Start by extracting all the data
	packet->Address = rx_data[packet_index + 4];
	packet->PT = PT;

	// Get the data bytes and compute the checksum all in one step
	packet->data_length = data_length;
	uint16_t computed_checksum = 's' + 'n' + 'p' + packet->PT + packet->Address;
	for (index = 0; index < data_length; index++) {
		// Copy the data into the packet structure’s data array
		packet->data[index] = rx_data[packet_index + 5 + index];
		// Add the new byte to the checksum
		computed_checksum += packet->data[index];
	}
	// Now see if our computed checksum matches the received checksum
	// First extract the checksum from the packet
	uint16_t received_checksum = (rx_data[packet_index + 5 + data_length] << 8);

	received_checksum |= rx_data[packet_index + 6 + data_length];
	// Now check to see if they don’t match
	if (received_checksum != computed_checksum) {
		return 4;
	}
	// At this point, we’ve received a full packet with a good checksum. It is already
	// fully parsed and copied to the ‘packet’ structure, so return 0 to indicate that a packet was
	// processed.
	return 0;
}


double printBytesRead(uint8_t* rx_data, uint8_t rx_data_length) {

	uint8_t addressTofind = 0x70;
	uint8_t pitch[2];
	uint8_t roll[2];
	// Call the parse_serial_data function to handle the incoming serial data. The serial data should
	// be placed in ’rx_data’ and the length in ‘rx_data_length’ before this function is called.
	if (parse_serial_data(rx_data, rx_data_length, &new_packet) == 0) {
		if (new_packet.Address == addressTofind) {

			roll[0] = new_packet.data[2];
			roll[1] = new_packet.data[3];

			pitch[0] = new_packet.data[0];
			pitch[1] = new_packet.data[1];

			int16_t roll16bit = ((int16_t) roll[0] << 8) | roll[1];
			int16_t pitch16bit = ((int16_t) pitch[0] << 8) | pitch[1];

			__bswap_16(roll16bit);
			__bswap_16(pitch16bit);

			double rollValue = roll16bit / 91.02222;
			double pitchValue = pitch16bit / 91.02222;

//			printf("SIZEOF roll: %d SIZEOF pitch: %d \n", sizeof roll, sizeof pitch);
			printf("roll: %f [DEG], pitch: %f [DEG] \n", rollValue, pitchValue);
			return rollValue;
		} else {
			printf("Packet of specified address %d not found \n", addressTofind);
			printf("The address found this time was %d \n", new_packet.Address);
			return -1;
		}
	}else{
		return -1;
	}
}
