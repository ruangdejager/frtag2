/*
 * math_func.c
 *
 * Assorted math helper functions.
 */

#include "math_func.h"

/*
 * Basic linear conversion: y = (x * mNum / mDen) + c
 */
int16_t MATH_FUNC_i16ConvLin(uint16_t u16X, int16_t i16MNum, int16_t i16MDen, int16_t i16C)
{
    int32_t i32Res;
    i32Res  = (int32_t)u16X * (int32_t)i16MNum;
    i32Res /= (int32_t)i16MDen;
    i32Res += i16C;
    return (int16_t)i32Res;
}
