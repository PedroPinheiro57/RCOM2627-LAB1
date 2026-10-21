// RCOM 2026/2027
//
// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>
#include <string.h>
#include <time.h>

// Packet control field (C)
#define C_START 1
#define C_DATA 2
#define C_END 3

// Control packet parameters (T of each TLV)
#define T_FILE_SIZE 0
#define T_FILE_NAME 1

// File bytes per data packet (a data packet has a 3-byte header: C, L2, L1)
#ifndef FRAGMENT_SIZE
#define FRAGMENT_SIZE (MAX_PAYLOAD_SIZE - 3)
#endif
#if FRAGMENT_SIZE < 1 || FRAGMENT_SIZE > MAX_PAYLOAD_SIZE - 3
#error "FRAGMENT_SIZE must be between 1 and MAX_PAYLOAD_SIZE - 3"
#endif

// Builds a START or END packet: C | T=0 L=4 file size | T=1 L file name
static int buildControlPacket(unsigned char *packet, unsigned char c, long fileSize, const char *fileName)
{
    int nameLength = strlen(fileName);
    if (nameLength > 255)
        nameLength = 255;

    int size = 0;
    packet[size++] = c;

    packet[size++] = T_FILE_SIZE;
    packet[size++] = 4;
    for (int i = 3; i >= 0; i--)
        packet[size++] = (fileSize >> (8 * i)) & 0xFF;

    packet[size++] = T_FILE_NAME;
    packet[size++] = nameLength;
    memcpy(&packet[size], fileName, nameLength);
    return size + nameLength;
}

// Reads the TLVs of a START or END packet. Returns 0 if the file size was found.
static int parseControlPacket(const unsigned char *packet, int size, long *fileSize, char *fileName)
{
    *fileSize = -1;
    fileName[0] = '\0';

    int i = 1;
    while (i + 2 <= size)
    {
        unsigned char type = packet[i];
        unsigned char length = packet[i + 1];
        const unsigned char *value = &packet[i + 2];
        if (i + 2 + length > size)
            return -1;

        if (type == T_FILE_SIZE)
        {
            *fileSize = 0;
            for (int k = 0; k < length; k++)
                *fileSize = (*fileSize << 8) | value[k];
        }
        else if (type == T_FILE_NAME)
        {
            memcpy(fileName, value, length);
            fileName[length] = '\0';
        }
        i += 2 + length;
    }
    return *fileSize < 0 ? -1 : 0;
}

// Sends the START packet, the file in data packets and the END packet.
static int sendFile(FILE *file, const char *filename)
{
    fseek(file, 0, SEEK_END);
    long fileSize = ftell(file);
    rewind(file);

    const char *name = strrchr(filename, '/'); // send the name without the path
    name = (name == NULL) ? filename : name + 1;

    unsigned char packet[MAX_PAYLOAD_SIZE];
    int size = buildControlPacket(packet, C_START, fileSize, name);
    if (llSend(packet, size) < 0)
        return -1;

    long sent = 0;
    int bytes;
    while ((bytes = fread(&packet[3], 1, FRAGMENT_SIZE, file)) > 0)
    {
        packet[0] = C_DATA;
        packet[1] = bytes / 256; // L2
        packet[2] = bytes % 256; // L1
        if (llSend(packet, bytes + 3) < 0)
            return -1;
        sent += bytes;
        printf("Sent %ld/%ld bytes\n", sent, fileSize);
    }

    size = buildControlPacket(packet, C_END, fileSize, name);
    return llSend(packet, size) < 0 ? -1 : 0;
}

// Receives packets until the END packet. Returns 0 if the whole file arrived.
static int receiveFile(const char *filename, int baudRate)
{
    unsigned char packet[MAX_PAYLOAD_SIZE];
    char name[256], endName[256];
    long fileSize = -1, endSize = -1, received = 0;
    struct timespec start, end;
    FILE *file = NULL;
    int result = -1;

    while (TRUE)
    {
        int size = llReceive(packet);
        if (size <= 0)
            break;

        if (packet[0] == C_START && file == NULL)
        {
            if (parseControlPacket(packet, size, &fileSize, name) < 0)
                break;
            file = fopen(filename, "wb");
            if (file == NULL)
            {
                perror(filename);
                break;
            }
            clock_gettime(CLOCK_MONOTONIC, &start);
            printf("Receiving %s (%ld bytes)\n", name, fileSize);
        }
        else if (packet[0] == C_DATA && file != NULL)
        {
            int dataSize = size - 3;
            if (dataSize < 0 || 256 * packet[1] + packet[2] != dataSize)
            {
                printf("Invalid data packet\n");
                break;
            }
            if (fwrite(&packet[3], 1, dataSize, file) != (size_t)dataSize)
            {
                perror(filename);
                break;
            }
            received += dataSize;
            printf("Received %ld/%ld bytes\n", received, fileSize);
        }
        else if (packet[0] == C_END && file != NULL)
        {
            clock_gettime(CLOCK_MONOTONIC, &end);
            if (parseControlPacket(packet, size, &endSize, endName) < 0 ||
                endSize != fileSize || strcmp(endName, name) != 0)
                printf("END packet does not match the START packet\n");
            else if (received != fileSize)
                printf("Received %ld bytes, expected %ld\n", received, fileSize);
            else
            {
                double seconds = (end.tv_sec - start.tv_sec) + (end.tv_nsec - start.tv_nsec) / 1e9;
                double bitrate = received * 8 / seconds;
                printf("Transfer time: %.2f s | R = %.0f bit/s | S = R/C = %.3f\n",
                       seconds, bitrate, bitrate / baudRate);
                result = 0;
            }
            break;
        }
    }

    if (file != NULL)
        fclose(file);
    return result;
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

    if (strcmp(role, "tx") == 0)
    {
        FILE *file = fopen(filename, "rb");
        if (file == NULL)
        {
            perror(filename);
            return;
        }
        if (llOpenTx(llParameters) < 0)
        {
            printf("Could not establish the connection\n");
            fclose(file);
            return;
        }
        if (sendFile(file, filename) == 0)
            printf("File sent\n");
        else
            printf("File transfer failed\n");
        fclose(file);
        llCloseTx();
    }
    else if (strcmp(role, "rx") == 0)
    {
        if (llOpenRx(llParameters) < 0)
        {
            printf("Could not establish the connection\n");
            return;
        }
        if (receiveFile(filename, baudRate) == 0)
            printf("File received\n");
        else
            printf("File transfer failed\n");
        llCloseRx();
    }
    else
    {
        printf("Invalid role: %s. Must be 'tx' or 'rx'.\n", role);
    }
}