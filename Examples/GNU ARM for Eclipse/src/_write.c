//
// This file is part of the µOS++ III distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// Do not include on semihosting and when freestanding
#if !defined(OS_USE_SEMIHOSTING) && !(__STDC_HOSTED__ == 0)

// ----------------------------------------------------------------------------

#include <errno.h>
#include "diag/Trace.h"
#include <stm32f1xx_hal.h>
#include "main.h"

// ----------------------------------------------------------------------------

// When using retargetted configurations, the standard write() system call,
// after a long way inside newlib, finally calls this implementation function.

// Based on the file descriptor, it can send arrays of characters to
// different physical devices.

// Currently only the output and error file descriptors are tested,
// and the characters are forwarded to the trace device, mainly
// for demonstration purposes. Adjust it for your specific needs.

// For freestanding applications this file is not used and can be safely
// ignored.

extern UART_HandleTypeDef huart3;

ssize_t
_write (int fd, const char* buf, size_t nbyte);

ssize_t
_write (int fd __attribute__((unused)), const char* buf __attribute__((unused)),
        size_t nbyte __attribute__((unused)))
{
#if defined(TRACE)
  // STDOUT and STDERR are routed to the trace device
  if (fd == 1 || fd == 2)
    {
      return trace_write (buf, nbyte);
    }
#endif // TRACE

  if (fd==UART_BT){
    if(HAL_UART_Transmit_DMA(&huart3, buf, nbyte)==HAL_OK){
     return nbyte;
    }
  }
#ifdef DEBUG
  errno = ENOSYS;
  return -1;
#endif //DEBUG


}

// ----------------------------------------------------------------------------

#endif // !defined(OS_USE_SEMIHOSTING) && !(__STDC_HOSTED__ == 0)
