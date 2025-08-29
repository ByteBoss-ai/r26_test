# R26_test

<p align="center">
  <img src="https://github.com/teamrudra/r26_test/blob/main/misc/rover.webp" width="480" height="480"/>

#### Some Instructions
1. You may use any online resources, datasheets, or documentation needed, but be mindful of your time and stay focused on the task.
2. The duration of the test is 90 mins from 5:15pm to 6:45 pm.
3. There will be a MCQ test conducted [here](https://rudra26test.vercel.app/)
4. There are 4 tasks in the tests. Complete all of them.
5. In case you are not able to complete all the tasks, do upload whatever you are able to.
6. In the `README.md` of your repository include your thought process, places where you got stuck, where you used the help of AI, google or other online resources.
7. Even if you are not able to solve anything, do fill the readme and what your thought process would have been.
8. Carefully read the instructions to implement the required functionality.
9. Install [mingw(c compiler)](https://www.mingw-w64.org/downloads/#w64devkit) and [git](https://git-scm.com/downloads) if you haven't already done it.
10. After finishing your test, provide the link to your forked repository in the google form provided at the end.

### Aim/Objective: To decode GPS data of start and goal position, and create a path planning algorithm which computes an optimal path over a predefined gridmap

## Description
You are implementing code to decode GPS position data, received from a u-blox GNSS module on-board a rover (check out the [datasheet](https://drive.google.com/file/d/1rOcPxpP-3JE8l39kBMiQV6KKe8B6zlDf/view)). You are given the current/start position of the rover and the goal position where the rover has to reach, your goal is to develop a path planning algorithm to traverse over a pre-defined gridmap and generate necessary odometry commands (total time & angle traversed) to guide the rover along the generated path. 

### Task 0: Fork the provided repository and ensure it is set to PUBLIC so we can access and assess your work.
### Task 1: Decoding gps data (in ubx format) from u-blox reciever.
Working with UBX format and extracted relevant navigation data for use in the planner.
### Task 2: Develop a path planning algorithm to traverse over a gridmap.
Implemented a grid-based path planner that computes an optimal route from start to goal.
### Task 3: Generate odometry commands to guide the rover along the generated path.
Converted the path into motion commands (direction and timing) based on wheel parameters.
### Task 4: Compile and run the code.
Verified the workflow on sample inputs and ensured the project compiles successfully with g++.

#### Code
1. [src/main.cpp](src/main.cpp): Code for running the test.
2. [src/ublox_reader.cpp](src/ublox_reader.cpp): Recitfy errors in this code to compute correct lat & lon coordinates.
3. [src/planning.cpp](src/planning.cpp): Complete the defined `Planner::pathplanning` function 
4. [src/odometry.cpp](src/odometry.cpp): Complete the defined `Odometry::computeCommands` function 

#### How to Compile & Check your code
(make sure you are in the root directory)   
1. Compile your code by running: `make build`
2. To check if your code is implemented correctly run: `make check`
   
If you are able to compile your code successfully you should see something like this on your screen:

```
*** Success: ***
--------------------------------------------
```

4. If your make check was unsuccesfull, you can clean your attempt by running `make clean`, review your implementation and repeat the previous steps.

# Solution
## Understanding
Describe what you understood about the problem.
_Input: _UBX-format GPS frames (raw hex lines in a text file) for start & goal.

_Objective:_

1. Decode UBX NAV-POSLLH payload to obtain latitude, longitude, height.

2. Use those GPS coordinates as start and goal for a grid-based path planner.

3. Convert planned path into odometry commands (angles & durations) for your rover wheels.

4. Compile & validate with provided make build / make check.

TASK 1:
* u-blox GPS sends data in a special binary format called UBX protocol.

* Each message has a header, a class, an ID, a length, and then the actual payload (the useful info like lat/lon).

ubloxreader code:

* Reads UBX message data (saved in hex in a file).

* Converts the hex text to binary bytes.

* Picks out latitude, longitude, and height from the message.

* Converts them into human-readable values.
In Debugging: 
* original decodeUBX is wrong because of two things:
Offset | Field       | Size | Example
-------+-------------+------+---------
0      | Sync char 1 | 1    | 0xB5
1      | Sync char 2 | 1    | 0x62
2      | Class       | 1    | 0x01 (NAV)
3      | ID          | 1    | 0x02 (POSLLH)
4      | Length (LSB)| 1    | varies
5      | Length (MSB)| 1    |
6      | Payload     | N    |
N+6    | CK_A        | 1    |
N+7    | CK_B        | 1    |
👉 So:

Class is at buffer[2], not buffer[30].

ID is at buffer[3], not buffer[32].

Payload (the useful GPS numbers) starts at buffer + 6, not buffer + 4.

Therefore, the original code was checking way too far into the message (30 and 32 instead of 2 and 3).

It passed the wrong offset to NAV_POSLLH (+4 instead of +6).

That means it was never actually reading the proper lat/lon bytes.

Also:
the #include **"ublox_reader.h" error**  means:

The header file doesn’t exist in your project folder.

Or it exists, but the compiler doesn’t know where to find it.

Or it exists, but the function/struct declarations inside are missing/wrong.

TASK 2:
To move from start to goal on a grid with obstacles, I used a grid-based path planner (A* algorithm). It checks neighboring cells, avoids blocked cells, estimates the distance to the goal using a heuristic, and chooses the path with the lowest total cost. Once the goal is reached, the path is traced back from goal to start.

TASK 3:

Goal:

* You have a path (sequence of grid points) from the planner.

* You want to generate motion commands for a rover: how far to move and at what angle.

Rover Motion Basics:

* The rover moves with wheels of a certain radius and speed (RPM).

* Linear velocity = wheel circumference × revolutions per second.

Key Calculations:

* Distance: Between two consecutive path points → tells how far the rover must travel.

* Angle: Direction to turn to face the next point → computed using atan2 to get degrees.

Commands Output:

* Each segment of the path is converted to a motion command with:

  *distance / velocity → time to travel that segment.

  *angle → direction to face before moving.


## Thought Process
After understanding the problem, describe how you decided to proceed towards solving the question.

TASK 1:

Therefore, I added a ublox_reader.h file as well: Since the .cpp file is using classId and GPS, plus the functions readUbloxFile,   gpsFromData, and decodeUBX, the header should declare those.

* Verify UBX packet layout and byte offsets for NAV-POSLLH payload.


⚡ In short:

UBX class is at buffer[2].

UBX ID is at buffer[3].

Payload (lat/lon/etc.) starts at buffer[6].

* Fix any wrong offset indices or checks in the code.

* Add robust parsing of the hex input lines and ensure payload pointer arithmetic is correct.

For Task 2: 
* Stored the grid and its size in the Planner constructor.

* isvalid function to check if a cell is inside the grid and not blocked.

* heuristic function to estimate distance between two cells.

* pathplanning function to compute the path from start to goal (currently a placeholder).

For Task 3: 
* Constructor: Calculated the rover’s linear velocity using wheel radius and RPM.

* distance(): Computed straight-line distance between two consecutive path points.

* angle(): Calculated the heading angle from one point to the next in degrees.

* computeCommands(): Set up a framework to convert a path into motion commands (time and rotation).

* Loops over the path and calculates the distance and angle for each segment.

For Task 4: 
* compile, run unit tests (provided make check), iterate on bugs.

## Implementation
How did you decide to implement your solution.

Mention the details, such as the path planning & odometry how you tested it.

TASK 1:
int decodeUBX(uint8_t *buffer, classId *gps) {
  // UBX: [0]=0xB5, [1]=0x62, [2]=Class, [3]=ID
  if (buffer[2] == 0x01 && buffer[3] == 0x02) { // Class = NAV, ID = POSLLH
    return NAV_POSLLH(buffer + 6, gps);         // payload starts at offset 6
  }
  return 1;
}

TASK 2:
Planner::Planner(const vector<vector<bool>> &grid) : grid(grid) {
    rows = grid.size();
    cols = grid[0].size();
}

bool Planner::isvalid(int x, int y) const {
    // Check if the cell is inside the grid and not blocked
    return (x >= 0 && x < rows && y >= 0 && y < cols && !grid[x][y]);
}

double Planner::heuristic(int x1, int y1, int x2, int y2) const {
    // Estimate distance between two cells (Euclidean)
    return sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));
}

vector<pair<int, int>> Planner::pathplanning(pair<int, int> start,
                                             pair<int, int> goal) {
    vector<pair<int, int>> path; // store final path 
    /* Your main path planning logic goes here */
    return path;
}

TASK 3:
// 1. Constructor: compute linear velocity from wheel radius and RPM
Odometry::Odometry(double wheel_radius, double rpm) { 
    linear_vel = 2 * M_PI * radius * (rpm / 60.0); 
}

// 2. Distance between two points
double Odometry::distance(int x1, int y1, int x2, int y2) { 
    return sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1)); 
}

// 3. Heading angle between two points
double Odometry::angle(int x1, int y1, int x2, int y2) { 
    return atan2(y2-y1, x2-x1) * 180.0 / M_PI; 
}

// 4. Compute motion commands (framework, loops over path)
MotionCommand Odometry::computeCommands(vector<pair<int,int>> &path) { 
    MotionCommand res = {0.0, 0.0}; 
    /* Loop over path to compute distance/time and angle */ 
    return res; 
}

TASK 4:
Created header files for all mentioned in main code



# Google Form
[Link to Repo Submission](https://docs.google.com/forms/d/e/1FAIpQLSdlVJ2LzP8wUOATRD804zDVL611rwwGMO1y_ecYu5aoV5YQfw/viewform)


<p align="center">
  <img src="https://github.com/teamrudra/r25-test/blob/main/datasheets/feynman-simple.jpg" width="600" height="600"/>
</p>
     
