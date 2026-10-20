// RCOM 2026/2027
//
// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>
#include <string.h>

// Temporary test of llSend/llReceive (the file transfer comes later):
// Tx sends TEST_PACKETS packets with known content and Rx checks them.
#define TEST_PACKETS 20

static void fillTestPacket(unsigned char *packet, int number)
{
    for (int i = 0; i < MAX_PAYLOAD_SIZE; i++)
        packet[i] = (number + i) % 256; // every byte value, including FLAG and ESC
}

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayer llParameters = {
        .baudRate = baudRate,
        .nRetransmissions = nTries,
        .timeout = timeout,
    };
    strcpy(llParameters.serialPort, serialPort);

    unsigned char packet[MAX_PAYLOAD_SIZE];
    unsigned char expected[MAX_PAYLOAD_SIZE];

    if (strcmp(role, "tx") == 0)
    {
        if (llOpenTx(llParameters) < 0)
        {
            printf("Could not establish the connection\n");
            return;
        }
        for (int i = 0; i < TEST_PACKETS; i++)
        {
            fillTestPacket(packet, i);
            if (llSend(packet, MAX_PAYLOAD_SIZE) < 0)
            {
                printf("Packet %d was not delivered\n", i);
                break;
            }
            printf("Packet %d sent\n", i);
        }
        llCloseTx();
    }
    else if (strcmp(role, "rx") == 0)
    {
        if (llOpenRx(llParameters) < 0)
        {
            printf("Could not establish the connection\n");
            return;
        }
        for (int i = 0; i < TEST_PACKETS; i++)
        {
            int size = llReceive(packet);
            fillTestPacket(expected, i);
            if (size == MAX_PAYLOAD_SIZE && memcmp(packet, expected, size) == 0)
                printf("Packet %d received OK\n", i);
            else
                printf("Packet %d is WRONG (%d bytes)\n", i, size);
        }
        llCloseRx();
    }
    else
    {
        printf("Invalid role: %s. Must be 'tx' or 'rx'.\n", role);
    }
}