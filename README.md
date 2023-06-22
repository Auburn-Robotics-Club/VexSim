
# Vex V5 Simulator [WIP]

A simulator designed for students using Vexcode Pro for Vex V5 robotics systems. Designed to be an easy drop code and run simulator to encourage students to explore their coding abilites. Still a heavy work and progress

## Acknowledgements

 - [Carson - AUBIE1 Lead Developer](https://github.com/pydev101)
 - [SFML - Graphics](https://www.sfml-dev.org/)
 - [SelbaWard - Graphics](https://github.com/Hapaxia/SelbaWard)
 - [Minalien - Controller](https://www.codeproject.com/Articles/26949/Xbox-360-Controller-Input-in-C-with-XInput)

## Installation

Requires Visual Studio with Desktop C++ Developer tools

Clone the Github and open the project in Visual Studio

Make sure you are in Debug x64 configuration

Start without debugging, it is expected to fail

Copy DLL files from "{ProjectDirectory}/SFML-2.5.1/DLLs/" to "x64/Debug/"

Start without debugging again and the field should appear

## Usage

Open simulator.h under "Header Files"

void pre_sim_setup() is run before the main loop starts and should be used to initlize everything.

void simulation(int t) is called in a loop after each phycics update. "t" repersents the time in msec since the simulation started.

## Features

Tank Drive Robot configurations
Controller Support

Vex Motor

Vex Encoders

Vex Inertial Sensor

Graphing (Recommend you read BaseGUI.h under Simulator)

-2 Graphs that overlay the field

-Scatter - paints a dot on the field

-Quiver - draws a vector on the field

-Plot - draws a line between each plotted point

-clearPlotBuffer - Clears the plot buffer starting new line segments

-clear - Clears graph
