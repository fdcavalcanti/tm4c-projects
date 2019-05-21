//Controle para o braço robótico
//PID
//Alimentação 3,3 V
//Inserir ângulo do braço via serial: Sval
//Frente/Ré: PN5/PN4
//Enable: PP4
//ADC In: PK0 (feedback) POT: 90º 2,131 V / 0º: 1,043 V
//TIMER0: interrupt cada 100 ms
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
//#include <string.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "inc/tm4c1294ncpdt.h"

uint32_t g_ui32SysClock;
#define Vmin 0.96
#define Vmax 1.94
int rec[3] = {0,999,999};
int setPoint=90, ADCVal, ang, uC=50;
int Kp = 10; Ki = 2, Kd = 4;
int erro = 0, erro_ant = 0, integral = 0;
float Vin, dt = 0.1;

unsigned concatenate(unsigned x, unsigned y);
void UART0Config();
void HBridgeConfig();
void ADC0Config();
void TIMER0Config();
void controlador();

void
UARTIntHandler(void)
{
    uint32_t ui32Status;
    ui32Status = UART0_MIS_R;
    UART0_ICR_R = ui32Status;

    while(ROM_UARTCharsAvail(UART0_BASE))
    {
        char dataIn = UART0_DR_R;
        ROM_UARTCharPutNonBlocking(UART0_BASE, dataIn);
        if (dataIn == 'S')
            rec[0] = 1;
        else if (rec[0]==1 & rec[1] == 999)
            rec[1] = dataIn-48;
        else if (rec[0]==1 & rec[1]!=999){
            rec[2] = dataIn-48;
            setPoint = concatenate(rec[1], rec[2]);
            rec[0]=0; rec[1]=999; rec[2]=999;
            ROM_UARTCharPutNonBlocking(UART0_BASE, '\n');
            ROM_UARTCharPutNonBlocking(UART0_BASE, 13);
        }

        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, GPIO_PIN_0);
        SysCtlDelay(g_ui32SysClock / (1000 * 3));
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0);
    }
}

void
UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
    while(ui32Count--)
    {
        ROM_UARTCharPutNonBlocking(UART0_BASE, *pui8Buffer++);
    }
}

void ADCInter1(){
    ADC0_ISC_R |= (1<<3);
    ADCVal = ADC0_SSFIFO3_R;     //Salva o valor obtido em X
}

void TIMER0Inter1(){
    TIMER0_ICR_R |= 1<<0;
    controlador();
}

int
main(void)
{
    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                             SYSCTL_OSC_MAIN |
                                             SYSCTL_USE_PLL |
                                             SYSCTL_CFG_VCO_480), 120000000);


    UART0Config(); //UART0 PA0 PA1 | LED2 PN0
    HBridgeConfig(); //PWM PF2(6) PF3(6) GEN1
    ADC0Config(); //AD PK0 | AIN 16
    TIMER0Config();//TIMER0 PD0(3) PWM0

    //UARTSend((uint8_t *)"\033[2JPronto! ", 13);
    UARTSend("Pronto! ", 9);

    while(1)
    {
        if((ADC0_ACTSS_R & (1<<16))==0)
            ADC0_PSSI_R |= (1<<3);      //Ativa a amostragem
    }
}
void controlador(void){
    float deriv;
    Vin = ADCVal*3.3/4095.0;
    ang = (Vin-Vmin)*90/(Vmax-Vmin);

    erro = setPoint - ang;
    integral = integral + erro*dt;
    if (integral>=150)
        integral = 150;
    else if (integral<=-150)
        integral = -150;
    deriv = (erro-erro_ant)/dt;

    uC = erro*Kp + integral*Ki + deriv*Kd + 50;
    erro_ant = erro;
    //uC = kp*erro + 50;
    if(uC > 100)
        uC = 100;
    else if(uC < 0)
        uC = 0;

    PWM0_1_CMPA_R = 3000-30*uC-1;

}

void TIMER0Config(void){
    SYSCTL_RCGCGPIO_R |= 1<<3;
    SYSCTL_RCGCTIMER_R |= 1<<0;
    GPIO_PORTD_AHB_AFSEL_R |= 1<<0;
    GPIO_PORTD_AHB_PCTL_R = 0x3;
    GPIO_PORTD_AHB_DEN_R |= 1<<0;

    //ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    TIMER0_CTL_R = 0x0;
    TIMER0_CFG_R = 0x0; //32 bit timer
    TIMER0_TAMR_R = 0x2;
    TIMER0_TAILR_R = 0xB71B00;

    NVIC_EN0_R |= 1<<19;
    TIMER0_IMR_R |= 1<<0;
    TIMER0_CTL_R |= 1<<0;

}

void ADC0Config(void){
    SYSCTL_RCGCADC_R |= 0x1;     //Habilita módulo 0 do ADC
    SYSCTL_RCGCGPIO_R |= 0x200;  //GPIO porta K
    GPIO_PORTK_AFSEL_R = 0x1;   //Pino 0 como entrada alternativa no GPIO
    GPIO_PORTK_DEN_R = 0x0;     //Limpa o pino do registrador gpio enable
    GPIO_PORTK_AMSEL_R = 0x1;   //Desliga a isolação do pino 0, ativando função analógica

    ADC0_ACTSS_R = 0x0;         //Desativa o sample sequencer pra configurar
    ADC0_EMUX_R = 0x0;          //Seleciona o evento para iniciar o sequencer
    ADC0_SSEMUX3_R = 0x1;       //Seleciona o segundo mux
    ADC0_SSMUX3_R = 0x0;        //Pino 15 do segundo mux
    ADC0_SSCTL3_R = 0x6;        //Ativa o end sequence (só faz uma leitura) USAR TAMBÉM PARA INTERRUPÇÃO
    ADC0_IM_R |= (1<<3);
    ADC0_ACTSS_R = 0x8;         //Aciona a SS3
    NVIC_EN0_R |= (1<<17);

}

void HBridgeConfig(void){
    //PWM PF2(6) PF3(6) PWM1 GEN1
    //DEAD BAND ATIVO 100 ticks
    SYSCTL_RCGCPWM_R = 0x1;
    SYSCTL_RCGCGPIO_R |= 1<<5;
    GPIO_PORTF_AHB_AFSEL_R |= 1<<2;
    GPIO_PORTF_AHB_AFSEL_R |= 1<<3;
    GPIO_PORTF_AHB_DEN_R |= 1<<2;
    GPIO_PORTF_AHB_DEN_R |= 1<<3;
    GPIO_PORTF_AHB_PCTL_R = 0x6600;

    PWM0_CC_R = 0x100;
    PWM0_CTL_R = 0x0;

    PWM0_1_GENA_R = 0x8C;
    PWM0_1_GENB_R = 0x80C;

    PWM0_1_LOAD_R = 0x1770/2;
    PWM0_1_CMPA_R = 0xBB8/2;
    PWM0_1_CMPB_R = 0xBB8/2;
    PWM0_1_DBCTL_R = 0x1;
    PWM0_1_DBRISE_R = 0x64;
    PWM0_1_DBFALL_R = 0x64;
    PWM0_1_CTL_R = 0x1;
    PWM0_ENABLE_R = 0xC;
}

void UART0Config(void){

    SYSCTL_RCGCUART_R = 0x1;
    SYSCTL_RCGCGPIO_R |= 0x1;
    GPIO_PORTA_AHB_DEN_R = 0x3;
    GPIO_PORTA_AHB_AFSEL_R |= 0x3;
    GPIO_PORTA_AHB_PCTL_R |= 0x11;
    UART0_CTL_R &= ~(0x1);
    UART0_IBRD_R = 0x41;
    UART0_FBRD_R = 0x7;
    UART0_LCRH_R = 0x70;
    UART0_CC_R = 0x0;
    UART0_CTL_R |= 0x1;

    NVIC_EN0_R = (1<<5);
    UART0_IM_R = 0x50; //INT_RX e INT_RT
    //LEDs da UART PN0
    SYSCTL_RCGCGPIO_R |= 1<<12;
    GPIO_PORTN_DIR_R = 0x1;
    GPIO_PORTN_DEN_R = 0x1;
}

unsigned concatenate(unsigned x, unsigned y) {
    unsigned pow = 10;
    while(y >= pow)
        pow *= 10;
    return x * pow + y;
}
