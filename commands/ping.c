#include <stdio.h>

#include "ping.h"

void on_ping() {
    uart_send("pong");
}
