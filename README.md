# iRobotCreate2
This repository is the Arduino library and Arduino sketch, Circuit Diagram for the iRobot Create® 2 that runs on Arduino UNO, MEGA 2560.  

## Description
##### iRobotCreate2  
This iRobotCreate2 is the Arduino library for the iRobot Create® 2 that runs on Arduino UNO and Arduino MEGA 2560, Arduino Nano 3.1. It's necessary to add the PNP transistor "2N4403(x1)" and the Rectifler "1N4001(x2)" for this system. We are currently developing that.

<img class="photo" src="https://github.com/5ide6urnslab/iRobotCreate2/blob/master/resource/iRobotCreate2.jpg" width="480px" />

<br><br>
##### iRobotArm  
This iRobotArm is the Arduino library for the Makeblock Robotic Arm Add-on Pack that runs on Arduino UNO. This library can add a Robot Arm to iRobot Create® 2. We designed the new system to control the Makeblock Robotic Arm Add-on Pack in arduino. It's necessary to add the motor driver "Sparkfun TB6612FNG Breakout" and the DC/DC converter "MCW03-05S12". We are currently developing that.

<img class="photo" src="https://github.com/5ide6urnslab/iRobotCreate2/blob/master/resource/iRobotArm.jpg" width="480px" />


## Installation
(1) To use the above library, first you need to download and install  
https://www.arduino.cc/en/Guide/HomePage

(2) To get a copy of the repository you can download the source from  
https://github.com/5ide6urnslab/iRobotCreate2

(3) The addon should sit in **/Arduino/libraries/** .

## Running the Example Project
An example project is provided in the **/example/** folder.
<br>
##### Create 2 DJ Turntable I
This example "iRobotCreate2WithROS" is the phonograph using iRobot Create® 2. The iRobot Create® 2 is rotated and plays the music of LP Record(12 inch/33rpm). Please see below for the production manual "DJ Turntable".

http://www.irobot.com/About-iRobot/STEM/Create-2/Projects.aspx

<a href="http://www.youtube.com/watch?feature=player_embedded&v=TV7yp2ephXI
" target="_blank"><img src="http://img.youtube.com/vi/TV7yp2ephXI/0.jpg" 
alt="IMAGE ALT TEXT HERE" width="320" height="240" border="10" /></a>

<br>
##### Create 2 DJ Turntable II
This example "MakerFaireTokyo2016" is the Analog Record Player using iRobot Create® 2 and Tonearm. We have created the DJ Turntable System using the most advanced robot in the future. The iRobot Create® 2 is used for enjoying DJ play as musical instrument. This work staged at Maker Faire Tokyo 2016.

http://makezine.jp/event/makers2016/dum6_sen5e/  
http://dum6sen5e.com/Create-2-DJ-Turntable-II

<a href="http://www.youtube.com/watch?feature=player_embedded&v=A3ykXC0qWKs
" target="_blank"><img src="http://img.youtube.com/vi/A3ykXC0qWKs/0.jpg" 
alt="IMAGE ALT TEXT HERE" width="320" height="240" border="10" /></a>

<br>
##### RobotSimuLink
This example "RobotSimuLink" control the motor of iRobot Create® 2 and get sensors data by openframeworks v080. You must add ofxGui and ofxXmlSettings, ofxCsv as addon.  
<img class="photo" src="https://github.com/5ide6urnslab/iRobotCreate2/blob/master/resource/RobotSimuLink.png" width="340px" />

<br>
##### Create 2 DJ Turntable III
This example is the Analog Record Player using multiple iRobot Create® 2.This work is going to stage at Ogaki Mini Maker Faire 2016.

## Reference
(1) iRobot Open Interface Specifition
http://www.irobotweb.com/~/media/MainSite/PDFs/About/STEM/Create/iRobot_Roomba_600_Open_Interface_Spec_0908.pdf?la=en

(2) uClibc++ for Arduino  
https://github.com/maniacbug/StandardCplusplus

(3) openframeworks  
http://openframeworks.cc

## Copyright
© 2015 All Rights Reserved. iRobot, Roomba and Create are registered trademarks of iRobot Corporation.  
   http://www.irobot.com/About-iRobot/ST...

© 2015 All Rights Reserved. ROS is a trademark of the ROS.org.  
   http://www.ros.org
   
© 2015 All Rights Reserved. Makeblock is a trademark of Shenzhen Maker Works Technology Co., Ltd.  
   http://www.makeblock.cc

© 2015 All Rights Reserved. Sparkfun is a trademark of Sparkfun Electronics.  
   https://www.sparkfun.com

## License
##### About this manual.
<a rel="license" href="http://creativecommons.org/licenses/by-sa/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by-sa/4.0/88x31.png" /></a><br />5ide6urns lab is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by-sa/4.0/">Creative Commons Attribution-ShareAlike 4.0 International License</a>.

##### About this software. 
Released under the MIT license. http://opensource.org/licenses/mit-license.php

## Credit
#####Create 2 DJ Turntable I:

Electronics:   Show Kawabata(5ide6urns lab) http://dum6sen5e.com  
Electronics:   Shingo Masui(5ide6urns lab)  
Photographer:  Kenichi Iwasaki(5ide6urns lab)

#####Create 2 DJ Turntable II:

Electronics:   Show Kawabata(5ide6urns lab) http://dum6sen5e.com  
Electronics:   Shingo Masui(5ide6urns lab)  
Photographer:  Kenichi Iwasaki(5ide6urns lab)
