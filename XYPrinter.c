//Filipe do Ó Cavalcanti | Maio 2019
//contXrole da mesa com motor de passo
//Driver X; Conectam-se aos pinos PE0 PE1 PE2 PE3
//Driver Y; PWM0 PF2 | PF1 troca sentido | PF3 Enable
//Sequência de start up coloca a mesa toda para um lado até que
//o usuário ordene a parada, via UART, comando P
//Frequência para o motor de passo X: 75 Hz 3300 passos
//Frequência para o motor de passo Y: 300 Hz 420 passos

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/gpio.h"
#include "inc/tm4c1294ncpdt.h"
#include "driverlib/timer.h"

uint32_t g_ui32SysClock;
int rec[5] = {0,999,999,999,999},run=0;
int setPointX=0, contX=0, dirX=1;
int setPointY=0, contY=0, dirY=2;

#include "config.h"

void UART0Config(); //PA0 PA1
void GPIOMesaConfig(); //X: PE0...PE3 | Y: PL0...PL3
void TIMER0Config(); //PD0
void startUp();
void moveX(int dirX);
void moveY(int dirY);
void PWM0Config();
unsigned concatenate(unsigned x, unsigned y);

//---------- Interrupções
void Timer0IntHandler(void){
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    moveX(dirX);

}
void PWM1IntHandler(void){
    GPIO_PORTF_AHB_ICR_R |= 1<<2;
    moveY(dirY);
}


//-------------------

int main(void)
{
    g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                             SYSCTL_OSC_MAIN |
                                             SYSCTL_USE_PLL |
                                             SYSCTL_CFG_VCO_480), 120000000);
    UART0Config();
    PWM0Config();
    GPIOMesaConfig();
    startUp();
    TIMER0Config();

    while(1){
        if(setPointX<contX)
            dirX = 0;
        else if(setPointX>contX)
            dirX = 1;
        else
            dirX = 2;

        if(setPointY<contY){
            GPIO_PORTF_AHB_ICR_R |= (1<<2);
            dirY = 0;
        }
        else if(setPointY>contY){
            GPIO_PORTF_AHB_ICR_R |= (1<<2);
            dirY = 1;
        }
        else{
            dirY = 2;
        }
        if(contY>500)
            contY = 500;
        else if(contY<0)
            contY = 0;

    }
}

//--------------------

void moveX(int dirX){
    //0 esquerda | 1 direita
    //PE
    if(dirX == 0){
        if (GPIO_PORTE_AHB_DATA_R == 0x1)
            GPIO_PORTE_AHB_DATA_R = 0x8;
        else
            GPIO_PORTE_AHB_DATA_R = GPIO_PORTE_AHB_DATA_R >> 1;
        contX--;
    }
    else if(dirX == 1){
        if (GPIO_PORTE_AHB_DATA_R == 0x8)
            GPIO_PORTE_AHB_DATA_R = 0x1;
        else
            GPIO_PORTE_AHB_DATA_R = GPIO_PORTE_AHB_DATA_R << 1;
        contX++;
    }
}

void moveY(int dirY){
    //0 esquerda | 1 direita
    //PL
    if(dirY == 0){
        GPIO_PORTF_AHB_DATA_R |= 1<<1;
        GPIO_PORTF_AHB_DATA_R &= ~(1<<3);
        contY = contY -1;
    }
    else if(dirY == 1){
        GPIO_PORTF_AHB_DATA_R &= ~(1<<1);
        GPIO_PORTF_AHB_DATA_R &= ~(1<<3);
        contY = contY + 1;
    }
    else{
        GPIO_PORTF_AHB_DATA_R |= 1<<3;
    }

}
