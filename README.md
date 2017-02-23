# Project Ediosn Drone

This project aims to build a mobile phone controlled IoT drone prototype operated by Intel Edison.

Note: The final prototype suffers from PID tuning issues due to hardware limitations. The solution requires extra dedicated hardware, although since we already spend over $600 on this project, we cannot afford to update the prototype.

- We wrote extensive documentation to cover the past 5 weeks (Documentation.pdf)

- Videos Links:
	Presentation:
		https://youtu.be/A1RdrONbuTM
	Test 04:
		https://youtu.be/4uC_Rd7Gfuc
	Test 03:
		https://youtu.be/9BUDFJfgEbQ
	Test 02:
		https://youtu.be/QIXCuklesOs
	Test 01:
		https://youtu.be/ADkz-hN5q1U	


- Full Sketch is found at: [Code/Drone_Control/Drone_Control.ino]
- Used libraries are found at: [Code/libraries]

- Shortcomings:
	We were close to completing the project, although we did not quiet make it in time.
	The missing part is tuning (which is what I was referring to as balancing during the presentation) the constants of the PID algorithm.
	We have 6 constants and the range can be between 0 and 3 for the P, 0 and 2 for the I and 0 and 1 for the P in increments of 0.1 for the P, 0.2 for the I and 0.05 for the D.
	Those constants go into the PID algorithm to determine the adjustments necessary for the Quadcopter to self-balance.
	We also need two sets of those, one for the Pitch and Roll and the other one for the Yaw.
	The only way to tune this is through trial and error and we did plenty of tuning up until the presentation date, however, we did not get the right constants in time to hover it during the presentation.
	We also did not have time to include the Yaw PID and constants.
