/* coroutine.h
 *
 * Coroutine mechanics implemented on top of standard ANSI C.
 * See http://www.chiark.greenend.org.uk/~sgtatham/coroutines.html
 *
 * Copyright 1995,2000 Simon Tatham — MIT licence (see below).
 *
 * $Id: coroutine.h 6386 2005-10-12 09:13:42Z simon $
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * SIMON TATHAM BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef COROUTINE_H
#define COROUTINE_H

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

/* --------------------------------------------------------------------------
 * `scr' macros — static (non-reentrant) coroutines
 * -------------------------------------------------------------------------- */
#define scrBegin         static int scrLine = 0; switch(scrLine) { case 0:;
#define scrFinish(z)     scrLine = 0; } return (z)
#define scrFinishV       scrLine = 0; } return

#define scrReturn(z) \
        do { \
            scrLine = __LINE__; \
            return (z); case __LINE__:; \
        } while (0)
#define scrReturnV \
        do { \
            scrLine = __LINE__; \
            return; case __LINE__:; \
        } while (0)

/* --------------------------------------------------------------------------
 * `ccr' macros — reentrant coroutines with an explicit context struct
 * -------------------------------------------------------------------------- */
#define ccrContParam     void **ccrParam

#define ccrBeginContext  struct ccrContextTag { int ccrLine
#define ccrEndContext(x) } *x = *ccrParam

#define ccrBegin(x)      if (!x) { x = *ccrParam = malloc(sizeof(*x)); x->ccrLine = 0; } \
                         if (x) switch(x->ccrLine) { case 0:;
#define ccrFinish(z)     } free(*ccrParam); *ccrParam = 0; return (z)
#define ccrFinishV       } free(*ccrParam); *ccrParam = 0; return

#define ccrReturn(z) \
        do { \
            ((struct ccrContextTag *)*ccrParam)->ccrLine = __LINE__; \
            return (z); case __LINE__:; \
        } while (0)
#define ccrReturnV \
        do { \
            ((struct ccrContextTag *)*ccrParam)->ccrLine = __LINE__; \
            return; case __LINE__:; \
        } while (0)

#define ccrStop(z)    do { free(*ccrParam); *ccrParam = 0; return (z); } while (0)
#define ccrStopV      do { free(*ccrParam); *ccrParam = 0; return; } while (0)

#define ccrContext    void *
#define ccrAbort(ctx) do { free(ctx); ctx = 0; } while (0)

/* Convenience types */
typedef int8_t ccrResult_t;
#define CCR_BUSY        0
#define CCR_DONE_OK     1
#define CCR_DONE_ERROR  (-1)

typedef int8_t scrResult_t;
#define SCR_IDLE  0
#define SCR_BUSY  (!SCR_IDLE)

#endif /* COROUTINE_H */
