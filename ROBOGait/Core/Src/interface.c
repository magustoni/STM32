#include <interface.h>

//Transmision USB
void usb_transmit(Odometry* o, Wt901c* w, Radio* rc)
{
	float tx_buf[17];

	//Mensaje saliente:
	tx_buf[0] = o->px;
	tx_buf[1] = o->py;
	tx_buf[2] = o->yaw;

	tx_buf[3] = o->v; //v.x
	tx_buf[4] = o->w; //w.z

	tx_buf[5] = o->cam;

	tx_buf[6] = w->a[0]; //a.x
	tx_buf[7] = w->a[1]; //a.y
	tx_buf[8] = w->a[2]; //a.z

	tx_buf[9] = w->w[0]; //w.x
	tx_buf[10] = w->w[1]; //w.y
	tx_buf[11] = w->w[2]; //w.z

	tx_buf[12] = w->q[1]; //q.x
	tx_buf[13] = w->q[2]; //q.y
	tx_buf[14] = w->q[3]; //q.z
	tx_buf[15] = w->q[0]; //q.w

	tx_buf[16] = rc->mode; //modo funcionamiento

	CDC_Transmit_FS((uint8_t*)tx_buf, sizeof(tx_buf));
}

//Callback de recepcion USB, llamar en CDC_Receive_FS
void usb_receive(Controller* c, uint8_t* Buf, uint32_t* Len)
{
	//Mensaje entrante: v_in, w_in, cam_in
	uint8_t len = (uint8_t)*Len;
	float rx_buf[len];
	memcpy(rx_buf, Buf, len);

	c->v_in = rx_buf[0]; //Velocidad lineal solicitada
	c->w_in = rx_buf[1]; //Velocidad angular solicitada
	c->cam_in = rx_buf[2]; //Angulo camara solicitado
}

float pid_compute(Controller* c, Odometry* o, Radio* rc)
{
	if(rc->mode)
	{
		c->error = c->error_i = c->error_d = 0;
		c->timer = HAL_GetTick();
	}
	else
	{
		c->error_d = (c->v_in - o->v - c->error) / (HAL_GetTick() - c->timer) * 1000;
		c->error = c->v_in - o->v;
		c->error_i += c->error * (HAL_GetTick() - c->timer) / 1000;

		c->timer = HAL_GetTick();
		return Kp * c->error + Ki * c->error_i + Kd * c->error_d;
	}

	return 0;
}
