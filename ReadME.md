# Project 4: Calibration and Augmented Reality

## Overview
Project 4 involves augmented reality in which we can display virtual objects on an existing background. In this project, we detected corners of a chessboard (or a patterned circle grid), selected calibration images, calibrated the camera, displayed 3D axes and finally displayed a virtual object.

## Time Travel days
Used for this project: 0
 
## Development Environment
Operating System: Windows 10 64 bit \
IDE: Microsoft Visual Studio 2022/Visual Studio 2019  \
OpenCV version: 4.5.1

## Project Structure
Ensure the following files are in your directory. 

```
│   artwork.jpg
│   CalibrationFunction.hpp
│   CalibrationFunctions.cpp
│   csv_util.cpp
│   csv_util.h
│   Main.cpp
│   Main.h
│   Project 4 Report.pdf
│   ReadME.md
```

## How to run
1. Since IDE used is Visual Studio, there is no makefile.
2. Make sure the files above are present in the directory.
3. Import folder as an existing project into Visual Studio.
4. Configure project to include opencv directory.
5. Main.cpp has main function.
6. TThe CalibrationFunctions.hpp has all the calibration function declarations. The CalibrationFunctions.cpp has all the filter function definitions.
7. Alternatively, you can write your own Makefile.
8. If you are using a makefile, you will need to add CalibrationFunctions.o to the list of files for the program rule.

## How to use Program
Run Main for the tasks & extensions.

### Running videoObjectDetection.cpp
1. On running Main.cpp a live feed will start. The camera needs to be pointed towards the chessboard after which we can perform each task on a keypress
3. Press `1` to run Task 1 (extracting corners).
4. Press `s` to run Task 2 (saving calibration images).
5. Press `c` to run Task 3 (saving intrinsic parameters to csv file and display reprojection error).
6. Press `p` to run Task 4 (displays camera matrix, distortion coefficient, rotation matrix and translation matrix of the calibration images).
7. Press `a` to run Task 5 (displays 3D axes)
10. Press `v` to run Task 6 (displays virtual object).
11. Press `7` to run Task 7 (uses Harris corner detection).
12. Press `q` or close the main window to exit the program.
13. Enter the same key to toggle operation on or off.

## Extensions
### 1. Get your AR system working with a target other than the checkerboard
The camera needs to be pointed towards the asymmetric circles pattern after which we can perform each task on a keypress.

### 2. Do something to the target to make it not look like a target any more
1. Press `t` to enable masking of the pattern with the artwork.
2. Enter the same key to toggle operation on or off.

### 3. Get your system working with multiple targets or objects in the scene
See report for details.

### 4. Save video to file
1. Press `b` to start saving video to file and press `b` again to stop recording. (Extension 4)

### 5. Save Image to file
1. Press `n` to save image to file. (Extension 4)
2. A preview of the saved image will popup in a new window.
3. Press any key to close this preview and resume regular program execution.

## Limitations
1. If the video recording is started in color mode it will only capture color filter effects in the video. Similarly, if the video is started in a grayscale effect the whole video will show up as greyscale.
