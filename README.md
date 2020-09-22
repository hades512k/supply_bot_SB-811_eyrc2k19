# supply_bot_SB-811_eyrc2k19
This repository contains the implementation of eyrc 2k19-20 supply bot theme winning team.


The Arena has a white circle which acts as a path for the line following robot

Coins of colors red and green will be placed at distinct locations of the arena, to which the robot has to reach the propel the coins forward towards the central white region to score points.

The python program running on a computer acts as a command control service which captures image of the arena with the help of an overhead camera and guids the robot to move with the help of Xbees.

The embedded C program running on the AtMega328P microcontroller of the robot will recieve the commands from the server and performs appropriate actions.
