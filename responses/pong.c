#include <stdio.h>

#include "pong.h"
#include "uart.h"

void pong() {
    uart.send("pong");
}
