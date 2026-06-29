\# Autonomous Adjustable Hospital Bed



&#x20; ## Objective

&#x20; The current project consists of complete motor control and comparing IMUs for proper angle achievement. This must be changed in such a way that the motors it will be controlled using a webserver hosted from the esp32. It must also be controlled by the UDP input received through the Wi-Fi and the relevant outputs must be displayed in the Nextion display, which is an existing features. Do not depend on the sensors at all.



&#x20; ## Steps

&#x20; 	1. Remove all the dependencies to the IMU sensors, thus the multiplexer.

&#x09;2. Add controls using a web server with buttons to control left, right, top and bottom parts to both directions.

&#x09;3. Retain the control using Nextion GUI and through UDP



&#x20; ## Requirement

&#x20; 	- Controls using web server, Nextion GUI and the UDP commands

&#x09;- Do not depend on IMUs for anything, remove any dependencies on IMUs and multiplexers (TCA)



&#x20;  ## Expected Output

&#x09;Control of three motors, in both directions separately using web server, Nextion display and through UDP.

