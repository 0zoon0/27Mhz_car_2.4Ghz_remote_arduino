# 27Mhz RC car to 2.4Ghz conversion
This project is about converting a 27Mhz RC car to a 2.4Ghz remote controller using Arduino Pro Mini (168p, 3.3v, 8Mhz) and NRF24 communication module.
The nice thing about this particular RC car is that it had an Arduino like board soldered on the main board. This board read the input from the antenna, decode it, and control the motors. It has four motors, three of them required two pins each, and one motor with just one input (as it rotates one direction). A high voltage (3.3v) input on a pin would make motor spin one direction, and a high voltage on the other pin would spin it in the opposite direction.
So the conversion is pretty easy - remove the logic board and hook up the arduino instead. 
The car's board with removed logic board and Arduino with the NRF24 and new remote are presented in here:
![Screenshot](https://raw.githubusercontent.com/0zoon0/27Mhz_car_2.4Ghz_remote_arduino/master/images/1.jpg)
The schematics of the connections is:
![Screenshot](https://raw.githubusercontent.com/0zoon0/27Mhz_car_2.4Ghz_remote_arduino/master/images/2.jpg)