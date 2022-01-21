#include "em_device.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "string.h"
#include "InitDevice.h"
#include "em_timer.h"
#include "stdio.h"
#include "math.h"
#include "em_rtc.h"

#define HABILITAR_DEBUG 1
const char public_key[] = "PUT YOUR KEY";
const char private_key[] = "PUT YOUR KEY";
const char apn_server[] = "PUT YOUR APN SERVER";
const char apn_user[] = "PUT YOUR APN USER";
const char apn_password[] = "PUT YOUR APN PASSWORD";

struct GGA_Data{
	float tiempo;
	float latitud;
	float longitud;
	int lock;
	int sats;
	float dilusion_h;
	float altitud;
};

void USART_Tx_String(USART_TypeDef *puerto, const char *string){
	for(int i=0; i<strlen(string); i++){
		USART_Tx(puerto, (uint8_t)string[i]);
	}
}

//TODO: Agregar control de MOSFET antes de encender el modulo
unsigned int MC60_On(){
	unsigned int MC60_status = GPIO_PinInGet(gpioPortB, 8);
	GPIO_PinOutClear(gpioPortA, 0);
	Delay_Ms(100);
	if(!MC60_status){
		Send_Debug_Msg("Prev Status: OFF");
		GPIO_PinOutSet(gpioPortB, 7);
		Delay_Ms(1000);
		GPIO_PinOutClear(gpioPortB, 7);
		Delay_Ms(2000);
		MC60_status = GPIO_PinInGet(gpioPortB, 8);
		if(MC60_status){
			Send_Debug_Msg("New Status: ON");
			return 1; //Se encendio correctamente
		}else{
			Send_Debug_Msg("New Status: OFF");
			return 2; //Se intento encender, pero no fue posible
		}
	}
	Send_Debug_Msg("Ninguna accion ejecutada.");
	return 0;//Ya estaba encendido
}

//TODO: Agregar control de MOSFET para asegurar el apagado
unsigned int MC60_Off(){
	unsigned int MC60_status = GPIO_PinInGet(gpioPortB, 8);
	if(MC60_status){
		Send_Debug_Msg("Prev Status: ON");
		GPIO_PinOutSet(gpioPortB, 7);
		Delay_Ms(1000);
		GPIO_PinOutClear(gpioPortB, 7);
		Delay_Ms(2000);
		MC60_status = GPIO_PinInGet(gpioPortB, 8);
		if(!MC60_status){
			Send_Debug_Msg("New Status: OFF");
			GPIO_PinOutSet(gpioPortA, 0);
			return 1; //Se apago correctamente
		}else{
			Send_Debug_Msg("New Status: ON");
			GPIO_PinOutSet(gpioPortA, 0);//Forzar apagado con MOSFET
			return 2; //Se intento apagar, pero no fue posible
		}
	}
	GPIO_PinOutSet(gpioPortA, 0);
	return 0;//Ya estaba apagado
}

//Delay desde 1ms hasta 2700ms
void Delay_Ms(uint16_t msDelay) {
  uint16_t cycle_delay = msDelay * 24;
  TIMER_CounterSet(TIMER0, 0);
  TIMER0->CMD = TIMER_CMD_START;
  while(TIMER0->CNT < cycle_delay){
  }
  TIMER0->CMD = TIMER_CMD_STOP;
}

//Extrae latitud, longitud y tiempo de una cadena GGA
struct GGA_Data Get_GGA_Data(const char *texto){
	struct GGA_Data data;
	float tiempo, latitud, longitud, dilusion_h, altitud;
	char ns, ew;
	int lock, sats;
	char *buffer = strchr(texto, '$');
	char coords[256];
	if(sscanf(buffer, "$GNGGA,%f,%f,%c,%f,%c,%d,%d,%f,%f", &tiempo, &latitud, &ns, &longitud, &ew, &lock, &sats, &dilusion_h, &altitud) >= 1) {
		if(!lock){
			latitud = 0.0;
			longitud = 0.0;
		}else{
			if(ns == 'S') {latitud  *= -1.0;}
            if(ew == 'W') {longitud *= -1.0;}
            float degrees = trunc(latitud / 100.0f);
            float minutes = latitud - (degrees * 100.0f);
            latitud = degrees + minutes / 60.0f;
            degrees = trunc(longitud / 100.0f);
            minutes = longitud - (degrees * 100.0f);
            longitud = degrees + minutes / 60.0f;
		}
	}
	data.tiempo = tiempo;
	data.latitud = latitud;
	data.longitud = longitud;
	data.lock = lock;
	data.sats = sats;
	data.dilusion_h = dilusion_h;
	data.altitud = altitud;

	return data;
}

//TODO: Agregar timeout para no bloquear el programa
//TODO: Mejorar lectura del buffer para comandos GNSS
const char * Rx_At_Response(bool isGNSS){
	char line[256];
	char ch;
	int i = 0;
	uint8_t crCount = 0;
if(isGNSS){
	while(ch != '$'){
		ch = USART_Rx(USART1);
	}
	while(ch != '*'){
		line[i] = ch;
		ch = USART_Rx(USART1);
		i++;
	}
}else{
	for(i = 0; i < 256; i++){
		ch = USART_Rx(USART1);
		line[i] = ch;

		if(ch==10 || ch==13){
			line[i] = 32;
		}
		if(ch == 13){
			crCount++;
		}
		if(crCount > 1){
			line[i] = '\0';
			break;
		}
	}
}
	return line;
}

//Puerto serial conectado al modulo MC60: USART1
void Send_At_Cmd(const char *cmd){
	USART_Tx_String(USART1, cmd);
	USART_Tx(USART1, 13);
}

//Puerto serial de debug: USART0
void Send_Debug_Msg(const char *msg){
	if(HABILITAR_DEBUG){
		USART_Tx_String(USART0, "DEBUG: ");
		USART_Tx_String(USART0, msg);
		USART_Tx(USART0, 10);
	}
}


void RTC_IRQHandler(void){
	RTC_IntClear(RTC_IFC_COMP0);
	Send_Debug_Msg("Interrupcion RTC");
	GPIO_PinOutSet(gpioPortF, 2);
}

void Ps_Hold_Asserted(unsigned int value){
	if(value){
		GPIO_PinOutSet(gpioPortB, 11);
	}else{
		GPIO_PinOutClear(gpioPortB, 11);
	}
}

int main(void)
{
  CHIP_Init();
  enter_DefaultMode_from_RESET();
  Ps_Hold_Asserted(1);
  const char *buffer;
  char buffer2[256];
  RTC_IntClear(RTC_IFC_COMP0);
  RTC_IntEnable(RTC_IEN_COMP0);
  NVIC_EnableIRQ(RTC_IRQn);
  RTC_CompareSet(0, 120);//Segundos para obtener coordenadas

  while (1) {
	  MC60_On();
	  Send_Debug_Msg("Encender modulo MC60");
	  Delay_Ms(2700);
	  Send_At_Cmd("ATE0");//Deshabilitar echo
	  Send_Debug_Msg("Deshabilitar echo");
	  Delay_Ms(300);
	  Send_At_Cmd("AT+QGNSSC=1");//Alimentar parte GNSS
	  Send_Debug_Msg("Alimentar parte GNSS");
	  Delay_Ms(2700);
	  struct GGA_Data dataGGA;
	  struct GGA_Data lastDataGGA;
	  dataGGA.lock = 0;
	  int retry = 0;
	  while((dataGGA.lock != 1 && dataGGA.lock != 2) && retry < 60){
		  Send_At_Cmd("AT+QGNSSRD=\"NMEA/GGA\"");
		  buffer = Rx_At_Response(true);
		  dataGGA = Get_GGA_Data(buffer);//Solucionar caracteres basura cuando no hay Fix (lock)
		  sprintf(buffer2, "%.3f,%f,%f,%d,%d,%.2f,%.2f,%d", dataGGA.tiempo, dataGGA.latitud, dataGGA.longitud, dataGGA.lock, dataGGA.sats, dataGGA.dilusion_h, dataGGA.altitud, RTC_CounterGet());
		  Send_Debug_Msg(buffer2);
		  if(dataGGA.lock == 1 || dataGGA.lock == 2){
			  lastDataGGA = dataGGA;
		  }
		  retry++;
		  RTC_CounterSet(0);
		  Delay_Ms(1000);
  	  }
	  Send_At_Cmd("AT+QGNSSC=0");
	  Send_Debug_Msg("Apagar modulo MC60");
	  Send_Debug_Msg("Activar modo sleep (EM2)...");
	  Delay_Ms(1000);
	  MC60_Off();
	  GPIO_PinOutClear(gpioPortF, 2);
	  EMU_EnterEM2(true);
  }
}
