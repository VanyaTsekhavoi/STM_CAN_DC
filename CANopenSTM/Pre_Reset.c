__attribute__((noreturn)) void jump_to_Reset_Handler(void) __attribute__((section(".branch")));

void jump_to_Reset_Handler(void)
{
    asm("B.W Reset_Handler");
    //    Reset_Handler();
}

