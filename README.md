This test program is tuned for the following setup:
- SparkFlex Controller
- Neo Vortex motor
- Rev-11-2853 Spark Flex Data Port Breakout Cable
- 3:1 gearbox as load.

Example output

This demonstrates running Teleop, then Auto multiple full cycles of 3 steps forward then 1 step back to the start. 
[Flex Vortex Abs Pos testing 2.txt](https://github.com/user-attachments/files/18634235/Flex.Vortex.Abs.Pos.testing.2.txt)

This next one demonstrates power cycle, one partial auto cycle, power cycle, one partial auto cycle.  Issue is that the 
absolute mode does not seem to nbe working, note how the Current Position shifts after the power cycle.

[Flex Vortex Abs Pos testing 3.txt](https://github.com/user-attachments/files/18634505/Flex.Vortex.Abs.Pos.testing.3.txt)
