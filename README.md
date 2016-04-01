# self-balancing-car
The self-balancing car includes a MCU Arduino Leonardo, two step motors , two motor drivers A4899, a IMU sensor MPU-6050, a bluetooth
module HC-06, and other neccessary parts.
The balancing control is based on two PI control loops, I find a good introduction as the link http://www.instructables.com/id/Self-Balancing-Robot/; For sensor fusion part(gyroscope + accelerometer to get the leaning angle) is performed by a Kalman filter, you may follow the next link in order to get more knowledge http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/.
I will post all the code including the arduino control code and app file for bluetooth control(which unfortunately only works on Andriod OS). For bluetooth control, you can install the balancecar_gui.apk file on your android phone.
