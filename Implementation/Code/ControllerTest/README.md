# Rollbot Demo

## General Description

Rollbot is a single actuator robot that can move on the ground. It is driven by a single motor inside the robot that spins a pendulum mass, and you can control its motion by accelerating and decelerating the motor. For more details please check the paper:

[Rollbot: a Spherical Robot Driven by a Single Actuator (arxiv.org)](https://arxiv.org/abs/2404.05120)

In this demo you will be controlling motor's driving speed in an attempt to move the robot here and there (slamming into purge towers…) using a Xbox one controller. 

## System Description

To run this demo, you will need:

- A Rollbot of course!
- **ControllerTest** code running on the raspberry pi zero 2w controller.
- Rollbot connected to a Xbox one controller, preferably using Bluetooth.

For how to set up Bluetooth connection to Xbox one controller and how to compile and run the code, please refer to the additional information section.

## What To Bring Checklist

- [ ] Rollbot
- [ ] Stuff for Rollbot to run into (purge tower or something)
- [ ] Hex screw driver for opening up Rollbot
- [ ] Li-Po battery x2
- [ ] Li-Po battery charger
- [ ] Tape to attach the Li-Po batteries to Rollbot
- [ ] Xbox one Bluetooth Joystick
- [ ] Backup AA battery x2 for joystick

And if you really want to make sure it's not going to go wrong or if you want to adjust things on the go:

- [ ] Keyboard and adaptor
- [ ] Monitor, connector and adaptor (HDMI)
- [ ] Computer
- [ ] Phone for local network

## Commands

Once the Rollbot and controller are up and running, you should be able to control the Rollbot using buttons and triggers. There are three states for Rollbot:

1. **Disengaged**, where motor power is off and unless you really want to engage it, it won't accidentally engage. It should always be in this state on your hands !!!
2. **Engaged**, where the motor power is on but the motor is not spinning. Keep in this state before you hand the controller to the kids.
3. **Running**, where the motor power is on and the motor is spinning. 

Here is how you switch between the states and control the Rollbot:

1. **Disengaged -> engaged**. Press X and B at the same time while not pressing A and Y, hold for 1s.
2. **Engaged-> running**. Press left and right menu buttons (the two buttons between the left joystick and the ABXY pad, right below the xbox symbol) at the same time.
3. **Running or Engaged -> Disengaged**. Press LB and RB buttons (the two buttons above the left joystick and the ABXY pad) at the same time.
4. **Control**. By default, the control input is the RT button (the right analog trigger above the right joystick and RB). the harder you push the button, the faster the motor will rotate. Note that this is the analog signal, so you can have intermediate speeds if you wish to.
5. **Change control input**. By default, the control input is the RT button, but you can change it to other buttons or joysticks by pressing down A and left control pad's right upper, right lower, left upper or left lower controls at the same time. These four directions corresponds to the RT, right joystick's Y axis, LT, left joystick's Y axis respectively. For triggers, the harder you push down the faster the motor driving speed is; for joysticks, the higher Y axis is, the faster.
6.  **Quit program**. Hold A and Y but not B and X for 10s to quit the program. **Note that you should always quit this way or else you risk running multiple instances of pigpio library which might cause issues.**

## Additional Information

1. To build the ControllerTest and run the program, you will need cmake and make on the raspberry pi

   ```shell
   sudo apt update && sudo apt upgrade
   sudo apt install make
   sudo apt install cmake
   ```

   To build the code, run 

   ```shell
   cd xxx/ControllerTest
   cmake ./
   make ./ControllerTest
   ```

   And to run the code, run

   ```shell
   sudo ./ControllerTest
   ```

   If you want to make it auto start at the boot so you don't need a computer, in the source code `main.cpp` change the code to

   ```c++
   #define WAIT_FOR_CONNECTION 1
   ```

   and add the code to start on boot menu by

   1. Open the terminal and type the following command to open the rc.local file

      ```shell
      sudo nano /etc/rc.local
      ```

   2. In the `rc.local` file, enter the following line of code before the "exit 0" line, where xxx is the path to the ControllerTest folder (you can find that out using `ls -lrt -d -1 "$PWD"/{*,.*}`)

      ```
      sudo systemctl start bluetooth &
      sudo xxxxx/ControllerTest/ControllerTest &
      ```

   3. Hit **CTRL + S** to save the file.

2. To enable Bluetooth-based Xbox one joystick support on a fresh raspberry pi

   ```shell
   sudo apt install bluetooth pi-bluetooth bluez blueman
   bluetoothctl	
   ```

   Then put the Xbox one in pairing mode by long clicking the pairing button, and in bluetoothctl

   ```shell
   agent on
   scan on
   ```

   Then after you find the controller you want to connect to, remember its MAC address.

   ```shell
   pair [XX:XX:XX:XX:XX:XX]
   trust [XX:XX:XX:XX:XX:XX]
   ```

   Then you should be able to auto connect to this controller every time.
