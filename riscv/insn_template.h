// See LICENSE for license details.

#define __STDINT_LIMITS
#include <stdint.h>
#include "mmu.h"
#include "mulhi.h"
#include "softfloat.h"
#include "tracer.h"
#include "platform.h" // softfloat isNaNF32UI, etc.
#include "internals.h" // ditto
#include <assert.h>
