// Copyright (c) 2025 Vipin M
// Licensed under the MIT License. See LICENSE file in the project root for full license information.

#ifndef SERIAL_H
#define SERIAL_H

#include "Utils.h"

// Serial function prototypes
void serial_init();
void scan_serial();
void send_response(const char resp[]);
void print_legal_moves();

#endif // SERIAL_H
