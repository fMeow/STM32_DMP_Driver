# STM32CubeMX Example
This code runs on STM32F103C8T6.

When re-generate projecs, remember to rename main.cpp to main.c, otherwise a new main.c will be generated along side main.cpp. This is a annoying STM32CubeMX feature that by now it can only generate C projects. 

After that, fix some header errors accroding to your compiler settings.

Attention, when trace is no implemented, just rename all trace_printf and trace_puts to your output functions.
