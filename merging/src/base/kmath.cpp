#include <stdlib.h>
#include "kmath.h"


float clamp(float val, float minVal, float maxVal)
{
	return __max(minVal,__min(val,maxVal));
}
