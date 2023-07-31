# STM32F429I-Discovery-and-HCSR-04-Ultrasonic-Ranging-Sensor-Built-in-Self-Test
In this project, we design a simple built-in test (BIT) with three types: P-BIT (Power-on BIT), C-BIT (Continous BIT), and I-BIT (Initiated BIT). 

P-BIT (Power-on BIT): Applied whenever the system is powered on. It checks whether there is failure in the system or not.

I-BIT (Initiated BIT): Applied whenever there is a manual call to the BIT (built-in test). This call may be through user button etc.

C-BIT (Continous BIT): Applied in regular periods to check there is a failure in the system. The call the BIT functionality happens every 5 min, 10 min, etc.

STM32CubeIDE was utilized to code this project.

As one can see, the LCD Display on the card is used to print out the failure messages, distance to the target, and the needle pointer.

If you have any questions about the code, please contact either "mustafa.baraz@ug.bilkent.edu.tr" or "b.guzell@outlook.com.tr".
