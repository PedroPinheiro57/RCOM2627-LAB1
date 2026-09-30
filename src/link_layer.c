// RCOM 2026/2027
//
// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

#include <stdio.h>
#include <unistd.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source
#define BUF_SIZE 256

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llOpenTx(LinkLayer llParameters)
{
    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }

    printf("Serial port %s opened\n", llParameters.serialPort);

    // SET frame: FLAG | A | C | BCC1 | FLAG
    unsigned char setFrame[5];

    setFrame[0] = 0x7E;                   
    setFrame[1] = 0x03;                   
    setFrame[2] = 0x03;                    
    setFrame[3] = setFrame[1] ^ setFrame[2]; 
    setFrame[4] = 0x7E;                   

    int bytes = writeBytesSerialPort(setFrame, 5);

    printf("SET frame sent: %d bytes\n", bytes);

    for (int i = 0; i < 5; i++)
    {
        printf("TX byte = 0x%02X\n", setFrame[i]);
    }

    // Receive UA
    unsigned char uaFrame[5];

    for (int i = 0; i < 5; i++)
    {
        bytes = readByteSerialPort(&uaFrame[i]);

        if (bytes <= 0)
        {
            printf("Error receiving UA frame\n");
            return -1;
        }

        printf("RX byte = 0x%02X\n", uaFrame[i]);
    }

    // Check UA frame
    unsigned char expectedBCC = uaFrame[1] ^ uaFrame[2];

    if (uaFrame[0] == 0x7E &&
        uaFrame[1] == 0x01 &&
        uaFrame[2] == 0x07 &&
        uaFrame[3] == expectedBCC &&
        uaFrame[4] == 0x7E)
    {
        printf("Valid UA frame received\n");
    }
    else
    {
        printf("Invalid UA frame received\n");
        return -1;
    }

    printf("Connection established!\n");

    return 0;
}

int llOpenRx(LinkLayer llParameters)
{
    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }

    printf("Serial port %s opened\n", llParameters.serialPort);

    // Receive SET
    unsigned char setFrame[5];

    for (int i = 0; i < 5; i++)
    {
        int bytes = readByteSerialPort(&setFrame[i]);

        if (bytes <= 0)
        {
            printf("Error receiving SET frame\n");
            return -1;
        }

        printf("RX byte = 0x%02X\n", setFrame[i]);
    }

    // Check SET frame
    unsigned char expectedBCC = setFrame[1] ^ setFrame[2];

    if (setFrame[0] == 0x7E &&
        setFrame[1] == 0x03 &&
        setFrame[2] == 0x03 &&
        setFrame[3] == expectedBCC &&
        setFrame[4] == 0x7E)
    {
        printf("Valid SET frame received\n");
    }
    else
    {
        printf("Invalid SET frame received\n");
        return -1;
    }

    // UA frame: FLAG | A | C | BCC1 | FLAG
    unsigned char uaFrame[5];

    uaFrame[0] = 0x7E;
    uaFrame[1] = 0x01;
    uaFrame[2] = 0x07;
    uaFrame[3] = uaFrame[1] ^ uaFrame[2];
    uaFrame[4] = 0x7E;

    // Send UA
    int bytes = writeBytesSerialPort(uaFrame, 5);

    printf("UA frame sent: %d bytes\n", bytes);

    for (int i = 0; i < 5; i++)
    {
        printf("TX byte = 0x%02X\n", uaFrame[i]);
    }

    printf("Connection established!\n");

    return 0;
}

////////////////////////////////////////////////
// LLSEND
////////////////////////////////////////////////
int llSend(const unsigned char *buf, int bufSize)
{
    // TODO: Implement this function

    return 0;
}

////////////////////////////////////////////////
// LLRECEIVE
////////////////////////////////////////////////
int llReceive(unsigned char *packet)
{
    // TODO: Implement this function

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llCloseTx()
{
    // TODO: Implement this function

    return 0;
}

int llCloseRx()
{
    // TODO: Implement this function

    return 0;
}
