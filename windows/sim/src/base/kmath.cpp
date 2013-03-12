#include <stdlib.h>
#include "kmath.h"
#include <math.h>


float clamp(float val, float minVal, float maxVal)
{
	return fmax(minVal,fmin(val,maxVal));
}
