# Emerging-Sys-Arch-Tech

#Summarize the project and what problem it was solving. In this Project, we create a prototype design for a thermostat device. CC3220S microcontroller will be interfacing with an on-board TMP006 temperature sensor via the I2C bus. Temperature readings will show the microcontroller to device whether to turn on or also shut down the heater/cooler to help maintain room temperature. Results go to print on PC screen via UART protocol.

 #What did you do particularly well? What I thought I did well on is the scheduler. I was able to work though the Timer0 and configured to generate periodic interrupts for every 100msec. Checking for the switches pressing is required to be done at 200msec, to check temperature measurements and control heating action at 500msec and printing results on the computer screen at 1000msec.

 #Where could you improve? I think I need to improve on code and comments. In the beginning I was running the programs and got a lot of errors. I was missing variables. For comments I need to write more details comments what talking about my code. So if someone reads it they understand it better.


 #What tools and/or resources are you adding to your support network? Being able to write programs and project that I can use for any IoT devices. Also zybooks we use in class is a great resourse to use.

 #What skills from this project will be particularly transferable to other projects and/or course work? Yes I think I make this project maintainable, readable, and adaptable. I work on these projects for hours. I took notes on what work and what didn’t work. I carefully thought about each problem.
