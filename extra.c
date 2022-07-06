#include <stdio.h>
#include <math.h>
#include "extra.h"

float max(float num1, float num2)
{
    return (num1 > num2 ) ? num1 : num2;
}

/**
 * Find minimum between two numbers.
 */
float min(float num1, float num2) 
{
    return (num1 > num2 ) ? num2 : num1;
}
