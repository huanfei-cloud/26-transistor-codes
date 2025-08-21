先初始化舵机，再初始化定时器以及PWM通道，最后可以设定舵机角度以及速度。

在cube中配置时，将TIM定时器对应的通道调到PWM generation CHx模式

prescaler为定时器挂载的总线频率*10**（-6）-1，如总线APB1频率168Mhz，则prescaler=167

由电机的额定频率设定重载值counter period，counter period = 总线频率/额定频率/(prescaler+1)

pulse为设定的初始比较寄存器CCR值，一般为2000或小于counter period值即可