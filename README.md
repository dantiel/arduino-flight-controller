# Arduino Ornithopter Flight Controller


## Features:

* PPM-to-PWM converter
* Flightcontroller with 3 stabilization modes (balance/stabilize/manual)
* Glide control system for ornithopter

## Part list:

1x Arduino Pro Mini  
1x Hall Sensor  
1x 10K Ω Resistor  
1x  MPU-6050  
(1x BEC e.g. 5V)  
few connector pins and wires  
solder and soldering iron  
solderable prototype board  


## Circuit
...


## Required Libraries:

Copy these libraries into `~/Documents/Arduino/libraries` if necesary.

* DigitalServo (_default_, it is the same as default Arduino Servo  library but with different default pulse value. Included in this repository)
* Wire  (_default_)
* [PPMReader](https://github.com/Nikkilae/PPM-reader)


## Setting Modes

### Enter Calibration Mode:
1. Hold model in equilibrium position unmoving
2. Move left stick to left bottom, right stick to right bottom:
	```
	|    _____        _____    |
	|   /     \      /     \   |
	|  |   .   |    |   .   |  |
	|  |  /    |    |    \  |  |
	|   \˚____/      \____˚/   |
	|                          |
	```
3. Continue to still hold model unmoving for about 2 seconds
	
	
### Enter Glide Throttle Setting Mode:

1. Hold model in air ensuring that wings can move freely
2. Move left stick to right bottom, right stick to left bottom
	```
    |    _____        _____    |
    |   /     \      /     \   |
    |  |   .   |    |   .   |  |
    |  |    \  |    |  /    |  |
    |   \____˚/      \˚____/   |
    |                          |
	```
3. Let go of right stick and move throttle quickly to desired position,
	which is set after two exact seconds
	

## TODO:

* save calibration and glide setting  in  EEPROM
* add circuit diagram
...


## Additional Notes / Tribute:

* Glide Controller inspired by GLDAB.  For information about original glide device and ordering one see http://ovirc.free.fr/GLDAB_Historique.php . Lighter version of GLDAB and further explanations can be found at http://kakutaclinic.life.coocan.jp/GLDABE.htm  .
* See a test demonstration of this glide controller: https://youtu.be/4Quc2F3qK30