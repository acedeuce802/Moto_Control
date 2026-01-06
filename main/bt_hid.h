#pragma once
#include <stdbool.h>
#include <stdint.h>

void bt_hid_init(void);
void bt_hid_enter_pairing_mode(void);
void bt_hid_factory_reset(void);     // clears bonds + reboots
bool bt_hid_is_connected(void);

void bt_hid_send_consumer(uint16_t bits);