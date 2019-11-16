#pragma once
#include "../PCH/pch.hpp"


void my_exit_handler(sig_t s);
void *exit_handler(void *);

void *VisualizerFreeze_handler(void *);

void *LcmPublish_handler(void *);
