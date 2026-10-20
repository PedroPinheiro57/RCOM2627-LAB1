// RCOM 2026/2027
//
// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source
#define BUF_SIZE 256

// Frame fields
#define FLAG 0x7E
#define ESC 0x7D
#define A_TX_CMD 0x03 // commands sent by Tx and replies sent by Rx
#define A_RX_CMD 0x01 // commands sent by Rx and replies sent by Tx
#define C_SET 0x03
#define C_UA 0x07
#define C_I(n) ((n) << 7)     // I0 = 0x00, I1 = 0x80
#define C_RR(n) (0xAA | (n))  // RR0 = 0xAA, RR1 = 0xAB
#define C_REJ(n) (0x54 | (n)) // REJ0 = 0x54, REJ1 = 0x55

typedef enum
{
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK, // header checked: reading the data field until the closing FLAG
    ESC_RCV // inside the data field, after an ESC byte
} State;

typedef enum
{
    FRAME_OK,
    FRAME_BAD_DATA, // I frame with a valid header but an error in the data field
    FRAME_TIMEOUT,
    FRAME_ERROR
} FrameResult;

static LinkLayer params;
static int ns = 0; // Tx: number of the next I frame to send
static int nr = 0; // Rx: number of the next I frame expected

static unsigned char frameData[MAX_PAYLOAD_SIZE + 1]; // data field + BCC2 of the last frame read
static int frameDataSize = 0;                         // data bytes, without BCC2

////////////////////////////////////////////////
// ALARM
////////////////////////////////////////////////
static volatile sig_atomic_t timedOut = FALSE;

static void alarmHandler(int signal)
{
    timedOut = TRUE;
}

static void startTimer()
{
    timedOut = FALSE;
    alarm(params.timeout);
}

static void stopTimer()
{
    alarm(0);
    timedOut = FALSE;
}

////////////////////////////////////////////////
// FRAME HELPERS
////////////////////////////////////////////////
static int isIFrame(unsigned char c)
{
    return c == C_I(0) || c == C_I(1);
}

static int sendSupervisionFrame(unsigned char a, unsigned char c)
{
    unsigned char frame[5] = {FLAG, a, c, a ^ c, FLAG};
    return writeBytesSerialPort(frame, 5) == 5 ? 0 : -1;
}

// Writes byte into out, escaped if it is FLAG or ESC. Returns the bytes written.
static int stuffByte(unsigned char byte, unsigned char *out)
{
    if (byte == FLAG || byte == ESC)
    {
        out[0] = ESC;
        out[1] = byte ^ 0x20;
        return 2;
    }
    out[0] = byte;
    return 1;
}

// Reads the next frame with address expectedA. The control field goes to *c and
// the destuffed data field of I frames goes to frameData.
static FrameResult readFrame(unsigned char expectedA, unsigned char *c)
{
    State state = START;
    unsigned char byte;
    unsigned char bcc2 = 0; // XOR of the data bytes and BCC2: 0 when there is no error
    int size = 0;

    while (TRUE)
    {
        if (timedOut)
            return FRAME_TIMEOUT;

        int res = readByteSerialPort(&byte);
        if (res < 0 && errno != EINTR)
            return FRAME_ERROR;
        if (res <= 0)
            continue; // no byte, or read() interrupted by the alarm

        switch (state)
        {
        case START:
            if (byte == FLAG)
                state = FLAG_RCV;
            break;

        case FLAG_RCV:
            if (byte == expectedA)
                state = A_RCV;
            else if (byte != FLAG)
                state = START;
            break;

        case A_RCV:
            if (byte == FLAG)
                state = FLAG_RCV;
            else
            {
                *c = byte;
                state = C_RCV;
            }
            break;

        case C_RCV:
            if (byte == (expectedA ^ *c))
            {
                state = BCC_OK;
                size = 0;
                bcc2 = 0;
            }
            else if (byte == FLAG)
                state = FLAG_RCV;
            else
                state = START;
            break;

        case BCC_OK:
        case ESC_RCV:
            if (byte == FLAG)
            {
                if (!isIFrame(*c))
                {
                    if (size == 0)
                        return FRAME_OK;
                    state = FLAG_RCV; // S and U frames have no data field: discard
                }
                else if (state == ESC_RCV || size < 2 || bcc2 != 0)
                    return FRAME_BAD_DATA;
                else
                {
                    frameDataSize = size - 1;
                    return FRAME_OK;
                }
            }
            else if (state == BCC_OK && byte == ESC)
                state = ESC_RCV;
            else if (size > MAX_PAYLOAD_SIZE)
                state = START; // longer than any valid frame: discard
            else
            {
                if (state == ESC_RCV)
                {
                    byte ^= 0x20;
                    state = BCC_OK;
                }
                frameData[size++] = byte;
                bcc2 ^= byte;
            }
            break;
        }
    }
}

// Reads frames until one with address a and control c arrives (or timeout/error).
static FrameResult waitFrame(unsigned char a, unsigned char c)
{
    FrameResult res;
    unsigned char received = 0;
    do
        res = readFrame(a, &received);
    while (res == FRAME_BAD_DATA || (res == FRAME_OK && received != c));
    return res;
}

// Sends a supervision command and waits for the expected answer,
// retransmitting the command after each timeout.
static int sendCommand(unsigned char a, unsigned char c, unsigned char answerA, unsigned char answerC)
{
    int timeouts = 0;
    while (TRUE)
    {
        if (sendSupervisionFrame(a, c) < 0)
            return -1;
        startTimer();
        FrameResult res = waitFrame(answerA, answerC);
        stopTimer();

        if (res == FRAME_OK)
            return 0;
        if (res == FRAME_ERROR)
            return -1;
        if (++timeouts > params.nRetransmissions)
        {
            printf("Timeout: no answer after %d retransmissions\n", params.nRetransmissions);
            return -1;
        }
        printf("Timeout: retransmission %d/%d\n", timeouts, params.nRetransmissions);
    }
}

// Saves the parameters, opens the serial port and installs the alarm handler.
static int openLink(LinkLayer llParameters)
{
    params = llParameters;
    if (openSerialPort(params.serialPort, params.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }
    printf("Serial port %s opened\n", params.serialPort);

    // No SA_RESTART: the alarm interrupts a blocking read() (errno = EINTR)
    struct sigaction act = {0};
    act.sa_handler = &alarmHandler;
    if (sigaction(SIGALRM, &act, NULL) == -1)
    {
        perror("sigaction");
        closeSerialPort();
        return -1;
    }
    return 0;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llOpenTx(LinkLayer llParameters)
{
    if (openLink(llParameters) < 0)
        return -1;

    if (sendCommand(A_TX_CMD, C_SET, A_TX_CMD, C_UA) < 0)
    {
        closeSerialPort();
        return -1;
    }
    printf("Connection established\n");
    return 0;
}

int llOpenRx(LinkLayer llParameters)
{
    if (openLink(llParameters) < 0)
        return -1;

    if (waitFrame(A_TX_CMD, C_SET) != FRAME_OK || sendSupervisionFrame(A_TX_CMD, C_UA) < 0)
    {
        closeSerialPort();
        return -1;
    }
    printf("Connection established\n");
    return 0;
}

////////////////////////////////////////////////
// LLSEND
////////////////////////////////////////////////
int llSend(const unsigned char *buf, int bufSize)
{
    if (buf == NULL || bufSize <= 0 || bufSize > MAX_PAYLOAD_SIZE)
        return -1;

    // F A C BCC1 | data + BCC2, each byte stuffed into 1 or 2 bytes | F
    unsigned char frame[4 + 2 * (MAX_PAYLOAD_SIZE + 1) + 1];
    unsigned char c = C_I(ns);
    unsigned char bcc2 = 0;
    int size = 0;

    frame[size++] = FLAG;
    frame[size++] = A_TX_CMD;
    frame[size++] = c;
    frame[size++] = A_TX_CMD ^ c;
    for (int i = 0; i < bufSize; i++)
    {
        bcc2 ^= buf[i];
        size += stuffByte(buf[i], &frame[size]);
    }
    size += stuffByte(bcc2, &frame[size]);
    frame[size++] = FLAG;

    int timeouts = 0;
    while (TRUE)
    {
        if (writeBytesSerialPort(frame, size) != size)
            return -1;
        startTimer();

        FrameResult res;
        unsigned char answer = 0;
        do
            res = readFrame(A_TX_CMD, &answer);
        while (res == FRAME_BAD_DATA ||
               (res == FRAME_OK && answer != C_RR(ns ^ 1) && answer != C_REJ(ns)));
        stopTimer();

        if (res == FRAME_ERROR)
            return -1;
        if (res == FRAME_OK && answer == C_RR(ns ^ 1))
        {
            ns ^= 1;
            return bufSize;
        }
        if (res == FRAME_OK)
            printf("REJ%d received: retransmitting I%d\n", ns, ns);
        else if (++timeouts > params.nRetransmissions)
        {
            printf("Timeout: I%d not acknowledged after %d retransmissions\n", ns, params.nRetransmissions);
            return -1;
        }
        else
            printf("Timeout: retransmitting I%d (%d/%d)\n", ns, timeouts, params.nRetransmissions);
    }
}

////////////////////////////////////////////////
// LLRECEIVE
////////////////////////////////////////////////
int llReceive(unsigned char *packet)
{
    while (TRUE)
    {
        unsigned char c = 0;
        FrameResult res = readFrame(A_TX_CMD, &c);
        if (res != FRAME_OK && res != FRAME_BAD_DATA)
            return -1;

        if (c == C_SET)
        {
            printf("SET received again: resending UA\n");
            sendSupervisionFrame(A_TX_CMD, C_UA);
        }
        else if (isIFrame(c))
        {
            int frameNs = c >> 7;
            if (frameNs != nr)
            {
                printf("Duplicate I%d discarded: resending RR%d\n", frameNs, nr);
                sendSupervisionFrame(A_TX_CMD, C_RR(nr));
            }
            else if (res == FRAME_BAD_DATA)
            {
                printf("Error in the data of I%d: sending REJ%d\n", frameNs, nr);
                sendSupervisionFrame(A_TX_CMD, C_REJ(nr));
            }
            else
            {
                memcpy(packet, frameData, frameDataSize);
                nr ^= 1;
                sendSupervisionFrame(A_TX_CMD, C_RR(nr));
                return frameDataSize;
            }
        }
    }
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