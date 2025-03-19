#include <stdint.h>

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CIRCULAR_BUFFER_H
#define __CIRCULAR_BUFFER_H

/** Defines **/
#define CIRCULAR_BUFFER_SIZE    4096

#define CIRCULARBUFFER_TESTENTRY 0


/** Types **/
struct T_CIRCULAR_BUFFER
{
    uint8_t UserRxBufferFS[CIRCULAR_BUFFER_SIZE];
    uint16_t UserRxBufferRead;
    uint16_t UserRxBufferWrite;
};

enum ERROR_CIRCULAR_BUFFER
{
    CIRCULARBUFFER_SUCCESS,
    CIRCULARBUFFER_BADPARAMETERS,
    CIRCULARBUFFER_MEMORY,
};

/** Exported Functions **/

/**
  * @brief  Reset a circular buffer
  * @param  circularBuffer: a pointer on a fully allocated T_CIRCULAR_BUFFER

  * @retval ERROR_CIRCULAR_BUFFER indicating the success of the operation
  */
enum ERROR_CIRCULAR_BUFFER circularBuffer_init(struct T_CIRCULAR_BUFFER* const circularBuffer);

/**
  * @brief  Write data to a circular buffer (not blocking).
  * @param  circularBuffer: a pointer on a fully allocated T_CIRCULAR_BUFFER (previously initialized with circularBuffer_init)
  * @param  buf: buffer of data to be written in circular buffer
  * @param  Len: Len of the buffer
  *
  * @retval ERROR_CIRCULAR_BUFFER indicating the success of the operation. In case there are not enough room, nothing is done
  */
enum ERROR_CIRCULAR_BUFFER circularBuffer_Push(struct T_CIRCULAR_BUFFER* const circularBuffer, const uint8_t* const buf, size_t Len);

/**
  * @brief  Read data from a circular buffer (not blocking).
  * @param  circularBuffer: a pointer on a fully allocated T_CIRCULAR_BUFFER (previously initialized with circularBuffer_init)
  * @param  buf: buffer that will receive the data read
  * @param  Len: Len of the buffer
  *
  * @retval Number of bytes copied to buf
  */
size_t circularBuffer_Pull(struct T_CIRCULAR_BUFFER* circularBuffer, uint8_t* buf, size_t Len);

#if CIRCULARBUFFER_TESTENTRY != 0
void circularbuffer_selfTest();
#endif


#endif
