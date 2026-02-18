# Timers

The following information is for STM32 timers. You will find similar concepts with different
microcontrollers; however, the terminology and specifics may differ.

## Configuring a Timer

There are three main parts that can control the behavior of a timer:

1. Prescaler (PSC)
2. Counter Period (also Auto-Reload Register, or ARR)
3. Count & Compare Register (CCR)

Let's break down each one:

### Prescaler (PSC)
This value determines the frequency of the timer. The timer frequency is calculated by the
following equation: `Clock Frequency / (PSC+1)`.

For example, if our clock frequency is 72 MHz and we set PSC = 71, our timer will tick at a
frequency of 1 MHz.

### Counter Period (Auto-Reload Register, or ARR)
This defines the number of ticks in one period. The period is calculated by the following equation:
`(1 / Timer Frequency) * (ARR + 1)`.

For example, if our timer frequency is 1 MHz, then the timer will tick every 1000 ns. So, if we
set the ARR = 4999, our period will be 0.005 seconds (`1000 ns * (4999 + 1) = 0.005 s`).

### Count & Compare Register (CCR)
This register holds a specific value that the timer tick counter is compared against. When the
counter reaches the value in the CCR, an event is triggered, such as an interrupt or output signal
change, which is commonly used for tasks like PWM generation or input capture.

## Use Cases

### Periodic Timer

### Input-Capture

### Output-Capture

### PWM

#### What is PWM?
PWM (pulse width modulation) is a digital signal that is set to high and low for a set amount of
time to represent a percentage.

#### Why do we use PWM?
Over a wire, we can only send a value of 0 (low voltage) or 1 (high voltage). However, let's say
that I want to control the percent brightness of an LED. How can I send this percentage (a value
between 0 and 1) to the LED with just a wire that can only send 0 or 1? This is where PWM comes
in. To send a value of 20% (0.20) using PWM, we first set a specified **period**. Then, for the
first 20% of the period, we would set the wire to 1. For the rest of the period, we set the wire to
0. This gives us a **duty cycle** of 20%.

#### Configuring a STM32 PWM timer
For PWM, we will have to modify the PSC, ARR, and CCR registers. The PSC and ARR registers can
be modified in the `.ioc` in order to set a constant period for the PWM signal. Then, in our code
we can modify the CCR register in order to set different PWM signals. For normal PWM generation,
the PWM signal will be set high in the beginning of the period. Then, when the timer counter goes
above the value of CCR, the PWM signal will be set low.

If you are still confused, here's a helpful [slide deck](https://docs.google.com/presentation/d/1eK4ROr9wMi3IOqEUcBABVkFSWsVM56jEgm_zoR2-wio/edit?usp=drive_link)
that goes more in depth. 




