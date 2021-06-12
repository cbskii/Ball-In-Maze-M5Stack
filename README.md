# M5Stack Ball In Maze Game
A little "ball in maze" style game for the M5Stack using the ESP IDF.

The primary goal was to make a game on the M5Stack that uses the built in MPU9250 IMU to move a ball through a maze shown on the display. A secondary goal was to rely only on the ESP IDF for the ESP32 and write some of the drivers from scratch rather than relying on the M5Stack Arduino libraries.

The idea is based off of this style of game:
![image](https://user-images.githubusercontent.com/16770076/121775928-46c7ec80-cb58-11eb-947b-48d80508b907.png)

And the result ended up looking like this:

There are still several issues to be fixed and improvements to be made, but future support may be limited.

## Building
The project is setup to build as a standard ESP IDF based project. More information for getting started can be found here: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/.

At time of writing the flow looks something like this:
1. Follow ESP IDF set up instructions
2. Use get_idf command/export needed environment variables
3. Build with `idf.py build` or build, flash, and monitor with `idf.py flash monitor`. Exit serial monitor with `Ctrl-]`.
