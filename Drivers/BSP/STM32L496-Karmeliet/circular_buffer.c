#include <string.h>
#include <stdint.h>

#include "circular_buffer.h"

#define MIN(a, b)  (((a) < (b)) ? (a) : (b))
#define MAX(a, b)  (((a) > (b)) ? (a) : (b))


enum ERROR_CIRCULAR_BUFFER circularBuffer_init(struct T_CIRCULAR_BUFFER* circularBuffer)
{
    if (circularBuffer == NULL)
    {
        return CIRCULARBUFFER_BADPARAMETERS;
    }

    memset(circularBuffer, 0, sizeof(struct T_CIRCULAR_BUFFER));

    return CIRCULARBUFFER_SUCCESS;
}


enum ERROR_CIRCULAR_BUFFER  circularBuffer_Push(struct T_CIRCULAR_BUFFER* const circularBuffer, const uint8_t* const buf, size_t Len)
{
    enum ERROR_CIRCULAR_BUFFER returnValue = CIRCULARBUFFER_SUCCESS;

    // Read index is Read only one time : Push and Pull can be called at the same time as on cortex M*
    // 32 bits accesses are atomics
    const uint32_t sizeRemainingBuffer = sizeof(circularBuffer->UserRxBufferFS) - (circularBuffer->UserRxBufferWrite - circularBuffer->UserRxBufferRead);

    if (buf == NULL || circularBuffer == NULL)
    {
        return CIRCULARBUFFER_BADPARAMETERS;
    }

    if (Len > sizeRemainingBuffer)
    {
        return CIRCULARBUFFER_MEMORY;
    }

    // check if there are any actions to perform
    if (Len)
    {
        const uint32_t indexStartWrite = circularBuffer->UserRxBufferWrite % sizeof(circularBuffer->UserRxBufferFS);
        const uint32_t virtualIndexEndIncludingOverflow = sizeof(circularBuffer->UserRxBufferFS) - indexStartWrite;
        const uint32_t indexEndExcludingOverflow = MIN(sizeRemainingBuffer, virtualIndexEndIncludingOverflow);
        const uint32_t sizeToCopy = MIN(Len, indexEndExcludingOverflow);

        memcpy(circularBuffer->UserRxBufferFS + indexStartWrite, buf, sizeToCopy);

        circularBuffer->UserRxBufferWrite += sizeToCopy;
        Len -= sizeToCopy;

        returnValue = circularBuffer_Push(circularBuffer, buf + sizeToCopy, Len);
    }

    return returnValue;
}

size_t circularBuffer_Pull(struct T_CIRCULAR_BUFFER* const circularBuffer, uint8_t* buf, size_t Len)
{
    uint32_t returnValue = 0;

    if (buf == NULL)
    {
        return 0;
    }

    // Write index is Read only one time : Push and Pull can be called at the same time as on cortex M*
    // 32 bits accesses are atomics

    const uint32_t sizeTotalAvailableData =circularBuffer->UserRxBufferWrite - circularBuffer->UserRxBufferRead;

    if (Len && sizeTotalAvailableData)
    {
        const uint32_t indexStartRead = circularBuffer->UserRxBufferRead % sizeof(circularBuffer->UserRxBufferFS);
        const uint32_t virtualIndexEndIncludingOverflow = sizeof(circularBuffer->UserRxBufferFS) - indexStartRead;
        const uint32_t indexEndExcludingOverflow = MIN(sizeTotalAvailableData, virtualIndexEndIncludingOverflow);
        const uint32_t sizeToCopy = MIN(Len, indexEndExcludingOverflow);

        memcpy(buf, circularBuffer->UserRxBufferFS + indexStartRead, sizeToCopy);

        circularBuffer->UserRxBufferRead += sizeToCopy;
        Len -= sizeToCopy;
        returnValue = sizeToCopy;

        returnValue += circularBuffer_Pull(circularBuffer, buf + sizeToCopy, Len);
    }

    return returnValue;
}

#if CIRCULARBUFFER_TESTENTRY != 0

#define SELF_TEST_ASSERT(x)     if(!(x)) __asm("bkpt #0")
void circularbuffer_selfTest()
{
    static struct T_CIRCULAR_BUFFER circbuffer = { 0 } ;
    uint8_t buffer[8];
    uint8_t bufferRead[8];

    // config test
    SELF_TEST_ASSERT(0 == (sizeof(circbuffer.UserRxBufferFS) % sizeof(buffer)));
    SELF_TEST_ASSERT(sizeof(bufferRead) == sizeof(buffer));

    // Test 0 : init
    circularBuffer_init(&circbuffer);
    SELF_TEST_ASSERT(circbuffer.UserRxBufferRead == 0);
    SELF_TEST_ASSERT(circbuffer.UserRxBufferWrite == 0);

    // Test 1 : read on empty buffer
    SELF_TEST_ASSERT(0 == circularBuffer_Pull(&circbuffer, buffer, sizeof(buffer)));
    SELF_TEST_ASSERT(circbuffer.UserRxBufferRead == 0);
    SELF_TEST_ASSERT(circbuffer.UserRxBufferWrite == 0);

    // Test 2a : push null buffer
    SELF_TEST_ASSERT(CIRCULARBUFFER_BADPARAMETERS == circularBuffer_Push(&circbuffer, NULL, sizeof(buffer)));
    SELF_TEST_ASSERT(circbuffer.UserRxBufferRead == 0);
    SELF_TEST_ASSERT(circbuffer.UserRxBufferWrite == 0);

    // Test 2a : pull null buffer
    SELF_TEST_ASSERT(0 == circularBuffer_Pull(&circbuffer, NULL, sizeof(buffer)));
    SELF_TEST_ASSERT(circbuffer.UserRxBufferRead == 0);
    SELF_TEST_ASSERT(circbuffer.UserRxBufferWrite == 0);

    // test 3 : push and read 1 byte
    buffer[0] = 0x42;
    SELF_TEST_ASSERT(0 == circularBuffer_Push(&circbuffer, buffer, 1));
    SELF_TEST_ASSERT(circbuffer.UserRxBufferRead == 0);
    SELF_TEST_ASSERT(circbuffer.UserRxBufferWrite == 1);

    SELF_TEST_ASSERT(1 == circularBuffer_Pull(&circbuffer, buffer, sizeof(buffer)));
    SELF_TEST_ASSERT(circbuffer.UserRxBufferRead == 1);
    SELF_TEST_ASSERT(circbuffer.UserRxBufferWrite == 1);

    // Test 4 : read on empty buffer that already contains data
    SELF_TEST_ASSERT(0 == circularBuffer_Pull(&circbuffer, buffer, sizeof(buffer)));
    SELF_TEST_ASSERT(circbuffer.UserRxBufferRead == 1);
    SELF_TEST_ASSERT(circbuffer.UserRxBufferWrite == 1);

    // Test 5: add too much data on empty buffer
    SELF_TEST_ASSERT(CIRCULARBUFFER_MEMORY == circularBuffer_Push(&circbuffer, buffer, sizeof(circbuffer.UserRxBufferFS) + 1));
    SELF_TEST_ASSERT(circbuffer.UserRxBufferRead == 1);
    SELF_TEST_ASSERT(circbuffer.UserRxBufferWrite == 1);

    // Test 6: complete buffer
    int i ;
    for(i = 0; i < sizeof(buffer); ++i)
    {
        buffer[i] = i;
    }
    for(i = 0; i < sizeof(circbuffer.UserRxBufferFS) / sizeof(buffer); ++i)
    {
        SELF_TEST_ASSERT(CIRCULARBUFFER_SUCCESS == circularBuffer_Push(&circbuffer, buffer, sizeof(buffer)));
        SELF_TEST_ASSERT(circbuffer.UserRxBufferRead == 1);
        SELF_TEST_ASSERT(circbuffer.UserRxBufferWrite == 1 + (i + 1) * sizeof(buffer));
    }

    SELF_TEST_ASSERT(CIRCULARBUFFER_MEMORY == circularBuffer_Push(&circbuffer, buffer, 1));
    SELF_TEST_ASSERT(circbuffer.UserRxBufferRead == 1);
    SELF_TEST_ASSERT(circbuffer.UserRxBufferWrite == sizeof(circbuffer.UserRxBufferFS) + 1);

    // Test 7: read complete buffer
    for(i = 0; i < sizeof(circbuffer.UserRxBufferFS) / sizeof(bufferRead); ++i)
    {
        memset(bufferRead, 0, sizeof(bufferRead));
        SELF_TEST_ASSERT(sizeof(bufferRead) == circularBuffer_Pull(&circbuffer, bufferRead, sizeof(bufferRead)));
        SELF_TEST_ASSERT(circbuffer.UserRxBufferRead == 1 + (i + 1) * sizeof(bufferRead));
        SELF_TEST_ASSERT(circbuffer.UserRxBufferWrite == sizeof(circbuffer.UserRxBufferFS) + 1);
        SELF_TEST_ASSERT(0 == memcmp(buffer, bufferRead, sizeof(bufferRead)));
    }
}
#endif
