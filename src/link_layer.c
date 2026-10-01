// RCOM 2026/2027
//
// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

#include <signal.h>
#include <stdio.h>
#include <unistd.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source
#define BUF_SIZE 256

// Frame fields
#define FLAG 0x7E
#define A_TX 0x03 // commands sent by Tx / replies sent by Rx
#define A_RX 0x01 // commands sent by Rx / replies sent by Tx
#define C_SET 0x03
#define C_UA 0x07

typedef enum
{
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    STOP
} State;

////////////////////////////////////////////////
// ALARM
////////////////////////////////////////////////
int alarmEnabled = FALSE;
int alarmCount = 0;

void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;
    printf("Alarm #%d received\n", alarmCount);
}

////////////////////////////////////////////////
// SUPERVISION FRAME HELPERS
////////////////////////////////////////////////

static int sendSupervisionFrame(unsigned char a, unsigned char c)
{
    unsigned char frame[5] = {FLAG, a, c, a ^ c, FLAG};

    int bytes = writeBytesSerialPort(frame, 5);

    printf("Frame sent: %d bytes\n", bytes);
    for (int i = 0; i < 5; i++)
    {
        printf("TX byte = 0x%02X\n", frame[i]);
    }

    return bytes == 5 ? 0 : -1;
}

static int readSupervisionFrame(unsigned char expectedA, unsigned char expectedC)
{
    State state = START;
    unsigned char byte;

    while (state != STOP)
    {
        int bytes = readByteSerialPort(&byte);

        if (bytes <= 0)
        {
            printf("Error reading from serial port\n");
            return -1;
        }

        printf("RX byte = 0x%02X\n", byte);

        switch (state)
        {
        case START:
            if (byte == FLAG)
                state = FLAG_RCV;
            // Other_RCV: stay in START
            break;

        case FLAG_RCV:
            if (byte == expectedA)
                state = A_RCV;
            else if (byte == FLAG)
                state = FLAG_RCV;
            else
                state = START;
            break;

        case A_RCV:
            if (byte == expectedC)
                state = C_RCV;
            else if (byte == FLAG)
                state = FLAG_RCV;
            else
                state = START;
            break;

        case C_RCV:
            if (byte == (expectedA ^ expectedC)) // A ^ C = BCC
                state = BCC_OK;
            else if (byte == FLAG)
                state = FLAG_RCV;
            else
                state = START;
            break;

        case BCC_OK:
            if (byte == FLAG)
                state = STOP;
            else
                state = START;
            break;

        case STOP:
            break;
        }
    }

    return 0;
}

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

    // Install the alarm handler
    struct sigaction act = {0};
    act.sa_handler = &alarmHandler;
    if (sigaction(SIGALRM, &act, NULL) == -1)
    {
        perror("sigaction");
        return -1;
    }

    int tries = 0;
    while (tries <= llParameters.nRetransmissions)
    {
        // Send SET and start the timer
        sendSupervisionFrame(A_TX, C_SET);
        alarmEnabled = TRUE;
        alarm(llParameters.timeout);

        // Wait for UA. If the alarm fires, read() is interrupted
        // and readSupervisionFrame returns -1.
        if (readSupervisionFrame(A_RX, C_UA) == 0)
        {
            alarm(0); // UA received: cancel the timer
            printf("Connection established!\n");
            return 0;
        }

        tries++;
        printf("No UA, retransmitting SET\n");
    }

    printf("Failed after %d tries\n", tries);
    closeSerialPort();
    return -1;
}

int llOpenRx(LinkLayer llParameters)
{
    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }

    printf("Serial port %s opened\n", llParameters.serialPort);

    // Wait for SET
    if (readSupervisionFrame(A_TX, C_SET) < 0)
    {
        printf("Error receiving SET frame\n");
        return -1;
    }

    printf("Valid SET frame received\n");

    // Send UA
    if (sendSupervisionFrame(A_RX, C_UA) < 0)
    {
        printf("Error sending UA frame\n");
        return -1;
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
