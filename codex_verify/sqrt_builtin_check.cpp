#include <math.h>
float test_builtin(float x) {
    return __builtin_sqrtf(x);
}
float test_std(float x) {
    return sqrtf(x);
}
