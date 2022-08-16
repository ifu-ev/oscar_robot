# oscar Launch 

## First time use - configuration to use sick LIDAR:
Insert IP of LIDAR to your launch file:

```nano ~/oscar_ws/src/oscar_base/sick_scan/launch/sick_tim_5xx.launch```

Change the default value of arg name="hostname" to your LIDAR IP address

Set static IP address for ethernet on your computer in the same addressspace as the LIDAR. 
For example: LIDAR -> 192.168.0.10, Computer -> Adress: 192.168.0.102, Netmask: 255.255.0.0, Gateway:192.168.0.102

## First time use - configuration to use Roomba: 

USB Permission: 

```sudo usermod -a -G dialout $USER```
 
Restart 

## Install

Install ds4drv to connect to Ps4 controllers:```sudo pip install ds4drv```


## Launch
Disconnect the Ps4 controllers from any cable. 
Run ds4dr to connect to Ps4 controllers: ```sudo ds4drv```
Press the "share" button and the small black button with the Playstation symbol simultanously on the controller until you see white lights. 
Wait until the terminal informs you, that the controller is connected. 

Launch robot with Ps4 controllers:

```roslaunch oscar_launch joy_teleop_roomba_sick.launch```



