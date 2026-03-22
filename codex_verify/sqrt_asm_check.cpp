float test_asm(float x) {
    float result;
    __asm__ volatile ("vsqrt.f32 %0, %1" : "=t" (result) : "t" (x));
    return result;
}
