#include <odometry.h>

//Inicializador de "objeto" odometria
HAL_StatusTypeDef odometry_init(Odometry* odom, TIM_HandleTypeDef* htim, ADC_HandleTypeDef* hadc)
{
	//Asignacion de handlers
	odom->encoder = htim; //Temporizador encoder
	odom->servos = hadc; //ADC realimentaciones

	//Inicializar odometria a 0
	odom->px = 0;
	odom->py = 0;
	odom->yaw = 0;
	for(int i = 0; i < 5; i++) odom->v_aux[i] = 0;
	odom->index = 0;

	//Iniciar lectura encoder
	HAL_TIM_Encoder_Start_IT(odom->encoder, TIM_CHANNEL_ALL); //Encoder - PA5+PB3

	//Inicio conversion ADC por DMA
	HAL_ADC_Start_DMA(odom->servos, odom->adc_buf, 2); //Realimentacion servos - PA1 y PA2

	//Inicializacion temporizador
	odom->timer = HAL_GetTick();

	return HAL_OK;
}

//Callback encoder, llamar en HAL_TIM_IC_CaptureCallback SOLO si htim==odom->encoder
void odometry_callback(Odometry* odom)
{
	//Lectura de la cuenta y transformacion a (m) recorridos
	odom->encoder_count = (int16_t)__HAL_TIM_GET_COUNTER(odom->encoder) * (2 * PI * RADIO) / (REDUCCION * 4);
}

//Actualizacion de la odometria, incluir en bucle principal
void odometry_update(Odometry* odom, float yaw)
{
	if(HAL_GetTick() - odom->timer > TIMER)
	{
		//Calculo velocidad por ventana movil de 5 elementos (m/s)
		odom->v_aux[odom->index++] = (odom->encoder_count - odom->encoder_aux) * 1000 / (HAL_GetTick() - odom->timer);
		if (odom->index > 4) odom->index = 0;
		odom->v = (odom->v_aux[0] + odom->v_aux[1] + odom->v_aux[2] + odom->v_aux[3] + odom->v_aux[4]) / 5;

		//Realimentacion velocidad angular: potenciometro -> ADC -> radio giro -> velocidad angular(rad/s)
		odom->w = (3.699 - 2.214 * odom->adc_buf[0] * 3.3 / 4096) * odom->v;

		//Actualizacion odometria
		odom->yaw = yaw;
		odom->px += (odom->encoder_count - odom->encoder_aux) * cos(yaw);
		odom->py += (odom->encoder_count - odom->encoder_aux) * sin(yaw);
		odom->encoder_aux = odom->encoder_count;

		//Realimentacion angulo camara: potenciometro -> ADC -> proporcion -> angulo camara(rad)
		odom->cam = (107 - 0.0534 * odom->adc_buf[1]) * 3.14159 / 180;

		//Actualizar timer
		odom->timer = HAL_GetTick();
	}
}
