int ax;

void
UARTIntHandler(void)
{
    uint32_t ui32Status;
    ui32Status = UART0_MIS_R;
    UART0_ICR_R = ui32Status;

    while(UARTCharsAvail(UART0_BASE))
    {
        int num1, num2;
        char dataIn = UART0_DR_R;
        UARTCharPutNonBlocking(UART0_BASE, dataIn);
        if (dataIn == 'X'){
            rec[0] = 1;
            ax = 0;
        }
        else if(dataIn == 'Y'){
            rec[0] = 1;
            ax = 1;
        }
        else if (rec[0]==1 & rec[1] == 999)
            rec[1] = dataIn-48;
        else if(rec[0]==1 & rec[1]!=999 & rec[2]==999)
            rec[2] = dataIn-48;
        else if(rec[0]==1 & rec[2]!=999 & rec[3]==999)
            rec[3] = dataIn-48;
        else if(rec[0]==1 & rec[3]!=999 & rec[4]==999){
            rec[4] = dataIn-48;
            num1 = concatenate(rec[1],rec[2]);
            num2 = concatenate(rec[3],rec[4]);
            if(ax==0)
                setPointX = concatenate(num1,num2);
            else if(ax == 1)
                setPointY = concatenate(num1,num2);
            rec[0]=0; rec[1]=999; rec[2]=999; rec[3]=999; rec[4]=999;
            UARTCharPutNonBlocking(UART0_BASE, '\n');
            UARTCharPutNonBlocking(UART0_BASE, 13);
        }
        else if (dataIn == 'P')
            run++;

        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, GPIO_PIN_0);
        SysCtlDelay(g_ui32SysClock / (1000 * 3));
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0);
    }
}

void startUp(){
    dirX = 1;
    do{
        SysCtlDelay(g_ui32SysClock / (250*3));
        moveX(dirX);
    }while(run==0);
    contX = 0;

    do{
       moveY(0);
    }while(run==1);
    contY = 0;

 }

void GPIOMesaConfig(){
    //Eixo X
    //PE0 PE1 PE2 PE3
    SYSCTL_RCGCGPIO_R  |= 1<<4;
    GPIO_PORTE_AHB_DIR_R |= 0xF;
    GPIO_PORTE_AHB_DEN_R |= 0xF;
    GPIO_PORTE_AHB_DATA_R = 0x1;

    //Eixo Y
    //PWM 0
    //Controle direção PF1
    SYSCTL_RCGCGPIO_R  |= 1<<5;

    GPIO_PORTF_AHB_DIR_R |= 1<<1;
    GPIO_PORTF_AHB_DEN_R |= 1<<1;
    //Enable PF3
    GPIO_PORTF_AHB_DIR_R |= 1<<3;
    GPIO_PORTF_AHB_DEN_R |= 1<<3;
    GPIO_PORTF_AHB_DATA_R |= 1<<3;

}


void UART0Config(void){
    //PA0 PA1
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
    //LEDs da UART: PN0
    SYSCTL_RCGCGPIO_R |= 1<<12;
    GPIO_PORTN_DIR_R = 0x1;
    GPIO_PORTN_DEN_R = 0x1;
}

void TIMER0Config(){
    //GPIO D
    SYSCTL_RCGCGPIO_R |= 1<<3;
    SYSCTL_RCGCTIMER_R |= 1<<0;

    TIMER0_CTL_R = 0x0;
    TIMER0_CFG_R = 0x0;
    TIMER0_TAMR_R = 0x2;
    TIMER0_TAILR_R = 0x186A00/4;
    NVIC_EN0_R |= 1<<19;
    TIMER0_IMR_R |= 1<<0;
    TIMER0_CTL_R |= 1<<0;
}

void PWM0Config(){
    //PWM PF2(6) PF3(6) PWM1 GEN1
    //DEAD BAND ATIVO 100 ticks
    SYSCTL_RCGCPWM_R = 0x1;
    SYSCTL_RCGCGPIO_R |= 1<<5;
    GPIO_PORTF_AHB_AFSEL_R |= 1<<2;
    GPIO_PORTF_AHB_DEN_R |= 1<<2;
    GPIO_PORTF_AHB_PCTL_R = 0x600;

    PWM0_CC_R = 0x105; //Clock PWM = sysClk/64
    PWM0_CTL_R = 0x0;

    PWM0_1_GENA_R = 0x8C;

    //PWM0_1_LOAD_R = 0x186A; //~300 Hz
    PWM0_1_LOAD_R = 0x199A; //~300 Hz
    PWM0_1_CMPA_R = PWM0_1_LOAD_R/2;
    PWM0_1_CTL_R = 0x1;

    GPIO_PORTF_AHB_IM_R |= (1<<2);
    GPIO_PORTF_AHB_IBE_R &= ~(1>>2);
    GPIO_PORTF_AHB_IS_R &= ~(1<<2);
    GPIO_PORTF_AHB_IEV_R &= ~(1>>2);
    GPIO_PORTF_AHB_ICR_R &= ~(1<<2);

    NVIC_EN0_R |= 1<<30;

    PWM0_ENABLE_R = 0xC;
}

unsigned concatenate(unsigned x, unsigned y) {
    unsigned pow = 10;
    while(y >= pow)
        pow *= 10;
    return x * pow + y;
}
