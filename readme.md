# Fanuc Allin Wrap

## At the very beginning
Trust me, the configuration process of all the fanuc with ros thing is very truly freaking traumatic.

Just leave alone that "dictionary like" 説明書(Introduction Handouts), and follow this readme carefully. You might be OK with this whole thing. Might, I mean, about 70%.

## Section Zero
0. There is a plate on the upper surface of the Fanuc Control Box which was fixed with four screws. Remove that plate.
1. Find a net cable. And connect your Ubuntu PC with the Fanuc Control Box. Be advised! Please mark the port you used on that Control Box. (**There are about two or three port on the inner PCB of the Control Box**) The later teach pendant configuration requires the information of the port you are using.


## Section One
0. 
1. Turn on the Fanuc Control Box



## Section XXX
1. Get the source code and build it.
```
mkdir -p /fanuc_ws/src
cd fanuc_ws/src
git clone https://github.com/wangarcher/fanuc_lrmate200id7l_wrap.git
cd ..
catkin_make
source devel/setup.bash
```


## Finale
0. Turn on the Fanuc Control Box and your Ubuntu PC, then connect them together with the corresponding internet interface setting.
1. Switch the Control Box to T1.
2. On the teach pandent, press SELECT->TYPE->TP Prog->ENTER. Then you might notice there's a **ROS** program on the screen.
3. Before start the program, press the Save Switch(hold), press the SHIFT on the pannel(hold), and press the RESET on the pannel.
4. After the FAULT light is out, keep pressing the Save Switch and SHIFT bottom, then use the cursor on to the program **ROS**, then press ENTER for one time, then press the FWD one time. 
5. On the terminal which the fanuc workspace has already been sourced, enter
```
roslaunch fanuc_bringup fanuc_lrmate200id7l_bringup_real.launch 
```
6. Enjoy your playing.