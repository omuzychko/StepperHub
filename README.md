# StepperHub

If you need something to run up to 3 steppers via UART interface (or USB-to-UART), with maximum possible speeds, simulataneously and for cheap - you got to the right place!

##Hardware

  - [Arduino CNC Shield with A4988 or DRV8825 driver boards](http://www.ebay.com/itm/CNC-kit-2-1X-Shield-4X-A4988-Drivers-for-Arduino-UNO-R3-ATmega328P-CH340G-/201552649446?hash=item2eed795ce6:g:E~UAAOSwGYVW~XXA)
  - [STM32 Nucleo-F446RE](http://www.digikey.com/product-detail/en/stmicroelectronics/NUCLEO-F446RE/497-15882-ND/5347712)

Arduino Shield with stepper drivers (but without Arduino itself) you should be able to get for for around 10$.
Nucleo 64 boards priced in a range of 10..15$ (depennding on exact model and place you get it).
So you get efficient UART (or USB-to-UART) 3-axis stepper controller for 20$. 
  
##Build/Dev Tools

  - [Cube MX](http://www.st.com/stm32cube)
  - [Keil uVision](http://www.keil.com/uvision/)

##Preface

I have been looking for UART stepper controller to build an PAN-TILT-ROLL motorized stage for my another project. 
If balanced - you can avoid using gearboxes (putting up to 5 lb on a single 5mm NEMA17 axis works just fine, just balance the mass arround the axis). But you will get another problem - inertia of huge masses. You can't put stepper into fast speed immediately, it just will not go (it will jerk skipping the steps). If you set huge enough timing between steps to get that mass into rotation - it will be rotating extremly slow unless you accelerate stepping overtime. 

So in my search of cheap stepper controller which can do the job I've found [Arduino CNC Shield V3](http://http://blog.protoneer.co.nz/arduino-cnc-shield/). But Arduino's AVR controller is extremly weak. If you need to run acceleration profiles with high speeds for multiple motors at the same time - you better get something more powerfull.

I just love TI ARM controllers (TM4C123 series). With full speed 16/32 or 32/64-bit timers they amazingly fit for this task (these controller can run up to 18 timers in 32-bit mode). But I am too lazy to prototype/DIY the board with TI controller to host Arduino CNC Shield. Luckily there are arduino-pinout [STM32 Nucleo 64](http://ww.st.com/stm32nucleo) dev borads availiable, with a very juicy STM32F4 controllers. STM32F4 series have just a few 32-bit itmers, but you may run 16-bit mode with 16-bit prescaller timers - less precision but does the job. So I've got NUCLEO-F446RE and after couple days of playing with Cube MX and uVision I've got quite nice results.

##[Here](https://www.youtube.com/watch?v=D3u7s1SLicY) is a demo video of eBay NEMA17 running at 3400rpm in 16x microstepping mode

##Implementation description

Each A4988 driver board needs two I/O signals to operate:

  - Stepping pulse;
  - Direction;

3 timers (TIM1, TIM2 and TIM3) configured to run in PWM mode using timer output channels (so the stepping pulse pin signalled automatically by TIM pheripheral).

  - X axis - TIM1, PWM stepping pulses at PA10, Direction GPIO out at PB4
  - Y axis - TIM2, PWM stepping pulses at PB3,  Direction GPIO out at PB10
  - Z axis - TIM3, PWM stepping pulses at PB5,  Direction GPIO out at PA8

The TIM_UPDATE interrupt handler is also enabled for each timer. It is used to count PWM steps pulses. These iterrupts configured with highest priority to others. So we don't miss the steps count and can easily run all three motros at 400kHz. Step pulse pin however is not flipped in interrupt handler programatically (as been said - this happens through PWM mode). PWM guarantees uniform pulsing, while interrupt handler routine is always a bit delayed and the delay duration varies every time (not much, tens to hundreds of nanoseconds, but at high speed this is critical).

There is one more timer configured - TIM14. 
It runs in a regular mode, simply excuting its TIM_UPDATE interrupt routine every 50 microseconds. This is a stepper controller timer, it checks the current speed of each connected mottor, estimates the time left to reach the destination (target step number) and comperas it with the time required to reduce the speed to the minimum (starting/stopping step time). And changes the speed accodringly (accellerating/decelerating the motor, or just keeping it at maximum allowed speed).

##UART portocol (TBD)

<implementation in progress>
...

##WARNING - Generating code with CubeMX project will give you an error message. Now worries - its OK!

I've have overclocked CPU to 200MHz to get simpler calculations of timing parameters, so easier to debug. Its just about 10% over spec. Playing with overclocking I was able to run this poor CPU at 280 MHz, getting close to 50MHz flipping GPIO PIN in a loop.

