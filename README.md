STEP 1

go to https://172.16.0.10/desk/
click tab SETTINGS/END EFFECTOR/
click the pen next to Mechanical Data 
click UPLOAD
select src2/OpenSai/balanceBot/endeffector-config-balancebot.json
go back to DESK
click on the N/A tab
click Activate FCI (if not already activated)
unlock the joints



STEP 2

open the terminal
run the robot driver with:
cd ~/OpenSai/drivers/FrankaPanda/redis_driver
and then run sh launch_driver.sh
open a second window and run the Force/Torque sensor driver with:
cd ~/OpenSai/drivers/ATIGamma_redis_driver/build
and then run ./ATIGamma_redis_driver 192.168.1.1



STEP 3

open a third window in the terminal and run the controller + visualizer with:
cd ~/OpenSai/balanceBot
and run: 
sh scripts/launch.sh 

if there is a redis error, open a new terminal window and run redis-server



STEP 4

Have fun



STEP 5

To stop the controller,
close the python tab and CTRL+C the controller window
Then to turn off the rest:
CTRL+C the robot driver and sensor driver
