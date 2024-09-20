#ifndef __INTERFACE_H
#define __INTERFACE_H

#include <main.h>
#include <string.h>

#include <odometry.h>
#include <controller.h>
#include <radio.h>
#include <wt901c.h>

extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

void usb_transmit(Odometry*, Wt901c*, Radio* rc); //Transmision USB
void usb_receive(Controller*, uint8_t*, uint32_t*); //Callback de recepcion USB, llamar en CDC_Receive_FS
float pid_compute(Controller*, Odometry*, Radio*);

#endif
