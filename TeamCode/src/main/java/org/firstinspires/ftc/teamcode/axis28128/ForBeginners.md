# Guide what's happening in this file
### Intake logic
* Very simple, uses two buttons and that is it. No need to modify
### Spindexer logic
* Works perfectly, no need to modify.
* Manual mode is just to correct the sensor over/undershooting.
* If they actually gear it correctly a lot more could be possible.
### Driving logic
* All is actually handled by PedroPathing for driving. Field-centric.
* See Tuning / Constants for more info
### Goal tracking
* Done with trigonometry using the coordinates system provided
by PedroPathing
* Tested only on Red Goal.
### Shooting
* Very simple PF controller on our flywheel.
* See better values please
* A good idea would also be to check Brogan M Pratt's video on PIDF tuning a flywheel.
### Autos
* Would be good to implement, but also requires that our spindexer doesn't block balls, 
and is correctly geared, also that there is a camera on the robot.
* For camera logic, see Internal Samples on FtcRobotController.
* For creating Autos, check Brogan M Pratt's video on that, for a sketch.

# Working with tuning, code modifying and debugging, etc.
### USB Tethering for hotspot/school Wi-Fi while laptop is connected via ADB USB to REV Control HUB
* See guide on connecting via Wi-Fi.
* Just upload code when modifying.
* To change values during runtime, access the panel at:
`192.168.43.1:8001`. From there you can change the static declared values to see which are the best.
* Values changed are not permanent. Make sure to change them later in code and then upload new code with new values so that they stay there forever.
* For autos/visualising the robot on the field, make sure to check out `visualizer.pedropathing.com`.
* telemetryM objects will show up on your Panels dashboard, while classic telemetry objects will show up on the Driver Hub only.

### Last notes
* Java is not that far off C++ for syntax. Some errors may be bullshit so feel free to ask claude/deepseek/minimax for help.
* If you have any question, be free to dm me via iMessage as I don't have stable internet the whole day, thanks.
* Have fun maintaining this code.