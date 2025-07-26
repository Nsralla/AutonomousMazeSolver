#include "AccelStepper.h"

// === STACK IMPLEMENTATION FOR FLOOD FILL ===
#define STACK_SIZE 256 // Maximum size of the stack
// Store coordinate pairs rather than cell pointers
struct StackItem
{
  int x;
  int y;
};
struct Stack
{
  StackItem items[STACK_SIZE]; // Array to hold stack items
  int top;
};
// STACK OPERATIONS
void initStack(Stack *stack)
{
  stack->top = -1; // Initialize stack as empty
}
bool isEmpty(Stack *stack)
{
  return stack->top == -1; // Stack is empty if top is -1
}
bool isFull(Stack *stack)
{
  return stack->top == STACK_SIZE - 1; // Stack is full if top is at max size
}

void push(Stack *stack, int x, int y)
{
  if (!isFull(stack))
  {
    stack->top++;
    stack->items[stack->top].x = x;
    stack->items[stack->top].y = y;
  }
  else
  {
    Serial.println("Stack overflow");
  }
}

// Return coordinates by reference
bool pop(Stack *stack, int &x, int &y)
{
  if (!isEmpty(stack))
  {
    x = stack->items[stack->top].x;
    y = stack->items[stack->top].y;
    stack->top--;
    return true;
  }
  else
  {
    Serial.println("Stack underflow");
    return false;
  }
}

// === CELL ===
struct CELL
{
  bool wallN, wallS, wallE, wallW;
  int floodValue;
  bool visited;
};
const int mazeSize = 16; // 16 x 16 cell
CELL maze[mazeSize][mazeSize];

int robotX = 0; // current x position
int robotY = 0; // current y position

enum Direction
{
  NORTH,
  EAST,
  SOUTH,
  WEST
};
Direction robotHeading = NORTH;

// ======= MOTOR PINS====
#define LEFT_STEP_PIN 21;
#define LEFT_DIR_PIN 19;
#define RIGHT_STEP_PIN 25;
#define RIGHT_DIR_PIN 33;

#define LEFT_MS1 14
#define LEFT_MS2 4
#define LEFT_MS3 5

#define RIGHT_MS1 22
#define RIGHT_MS2 32
#define RIGHT_MS3 23
// ===ULTRA SONIC PINS ========
#define FRONT_TRIG_PIN 18
#define FRONT_ECHO_PIN 34

#define LEFT_TRIG_PIN 27
#define LEFT_ECHO_PIN 35

#define RIGHT_TRIG_PIN 26
#define RIGHT_ECHO_PIN 13

// ====MOTORS===========
AccelStepper leftMotor(AccelStepper::DRIVER, LEFT_STEP_PIN, LEFT_DIR_PIN);
AccelStepper rightMotor(AccelStepper::DRIVER, RIGHT_STEP_PIN, RIGHT_DIR_PIN);

// ==== MOTORS CONSTANTS ============
const int STEPS_PER_20_CM = 2270;
const int STEPS_PER_1_CM = 113.5;
const int STEP_PER_90_DEGREE = 400;

// === ULTRASONIC CONSTANS ===
#define WALL_THRESHOLD_CM 10;
bool frontWall = false;
bool leftWall = false;
bool rightWall = false;

//  === SETUP ==========================
void setup()
{
  Serial.begin(9600);
  initMotors();
  delay(2000);

  initUltrasonics();
  delay(1000);

  //   init all cells in the maze
  for (int i = 0; i < mazeSize; i++)
  {
    for (int j = 0; j < mazeSize; j++)
    {
      maze[i][j].wallN = false;
      maze[i][j].wallS = false;
      maze[i][j].wallE = false;
      maze[i][j].wallW = false;
      maze[i][j].visited = false;
    }
  }

  //  set outer wall of the maze
  for (int i = 0; i < mazeSize; i++)
  {
    maze[i][0].wallS = true;
    maze[i][mazeSize - 1].wallN = true;
  }

  for (int i = 0; i < mazeSize; i++)
  {
    maze[0][i].wallW = true;
    maze[mazeSize - 1][i].wallE = true;
  }

  //   init flood fill values
  initializeFloodValues();

  Serial.println("Maze initialized");
  Serial.println("Ready to solve the maze");
}
// ======================================
void loop()
{
  if (true)
  {
    solveMaze();
  }
}
//  ===== MOTOR FUNCTIONS ======
void initMotors()
{
  leftMotor.setMaxSpeed(1000); // 1000 steps per second
  rightMotor.setMaxSpeed(1000);

  leftMotor.setAcceleration(500); // 500 steps per sec^2
  rightMotor.setAcceleration(500);

  leftMotor.setCurrentPosition(0);
  rightMotor.setCurrentPosition(0);
}
void moveForwardOneCell()
{
  Serial.println("Moving forward 20 CM...");
  leftMotor.move(STEPS_PER_20_CM);
  rightMotor.move(STEPS_PER_20_CM);

  while (leftMotor.isRunning() || rightMotor.isRunning())
  {
    leftMotor.run();
    rightMotor.run();
  }
}
void rotateLeft90()
{
  Serial.println("Rotating Left 90 DEGRE ... ");
  leftMotor.move(-STEP_PER_90_DEGREE);
  rightMotor.move(STEP_PER_90_DEGREE);

  while (leftMotor.isRunning() || rightMotor.isRunning())
  {
    leftMotor.run();
    rightMotor.run();
  }
}
void rotateRight90()
{
  Serial.println("Rotating 90 DEGREE right....");
  leftMotor.move(STEP_PER_90_DEGREE);
  rightMotor.move(-STEP_PER_90_DEGREE);

  while (leftMotor.isRunning() || rightMotor.isRunning())
  {
    leftMotor.run();
    rightMotor.run();
  }
}
void rotate180()
{
  Serial.println("Rotating 180...");
  leftMotor.move(STEP_PER_90_DEGREE * 2);
  rightMotor.move(-STEP_PER_90_DEGREE * 2);

  while (leftMotor.isRunning() || rightMotor.isRunning())
  {
    leftMotor.run();
    rightMotor.run();
  }
}
// ==================================

// ==== ULTRA SONIC FUNCTIONS =============
void initUltrasonics()
{
  pinMode(LEFT_TRIG_PIN, OUTPUT);
  pinMode(FRONT_TRIG_PIN, OUTPUT);
  pinMode(RIGHT_TRIG_PIN, OUTPUT);

  pinMode(LEFT_ECHO_PIN, INPUT);
  pinMode(RIGHT_ECHO_PIN, INPUT);
  pinMode(FRONT_ECHO_PIN, INPUT);
}
long readDistanceCM(int trigPin, int echoPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000); // Timeout after 30ms
  long distance = duration * 0.034 / 2; // Convert to cm
  return distance;
}
void scanWalls()
{
  long frontWallDistance = readDistanceCM(FRONT_TRIG_PIN, FRONT_ECHO_PIN);
  long leftWallDistance = readDistanceCM(LEFT_TRIG_PIN, LEFT_ECHO_PIN);
  long rightWallDistance = readDistanceCM(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN);

  frontWall = (frontWallDistance <= WALL_THRESHOLD_CM);
  leftWall = (leftWallDistance <= WALL_THRESHOLD_CM);
  rightWall = (rightWallDistance <= WALL_THRESHOLD_CM);

  Serial.print("Front: ");
  Serial.print(frontWall ? "Wall" : "Open");
  Serial.print(" | Left: ");
  Serial.print(leftWall ? "Wall" : "Open");
  Serial.print(" | Right: ");
  Serial.println(rightWall ? "Wall" : "Open");
}
bool wallInFront()
{
  return frontWall;
}
bool wallOnLeft()
{
  return leftWall;
}
bool wallOnRight()
{
  return rightWall;
}
void updateMazeWithWalls()
{
  // First, detect walls using sensors
  scanWalls();

  // Get current position
  int x = robotX;
  int y = robotY;

  // Update walls based on robot's heading
  switch (robotHeading)
  {
  case NORTH:
    // Update current cell
    maze[x][y].wallN = frontWall;
    maze[x][y].wallE = rightWall;
    maze[x][y].wallW = leftWall;

    // Update adjacent cells (symmetrically)
    if (y < mazeSize - 1) // Northern neighbor exists
      maze[x][y + 1].wallS = frontWall;

    if (x < mazeSize - 1) // Eastern neighbor exists
      maze[x + 1][y].wallW = rightWall;

    if (x > 0) // Western neighbor exists
      maze[x - 1][y].wallE = leftWall;
    break;

  case EAST:
    // Update current cell
    maze[x][y].wallE = frontWall;
    maze[x][y].wallS = rightWall;
    maze[x][y].wallN = leftWall;

    // Update adjacent cells (symmetrically)
    if (x < mazeSize - 1) // Eastern neighbor exists
      maze[x + 1][y].wallW = frontWall;

    if (y > 0) // Southern neighbor exists
      maze[x][y - 1].wallN = rightWall;

    if (y < mazeSize - 1) // Northern neighbor exists
      maze[x][y + 1].wallS = leftWall;
    break;

  case SOUTH:
    // Update current cell
    maze[x][y].wallS = frontWall;
    maze[x][y].wallW = rightWall;
    maze[x][y].wallE = leftWall;

    // Update adjacent cells (symmetrically)
    if (y > 0) // Southern neighbor exists
      maze[x][y - 1].wallN = frontWall;

    if (x > 0) // Western neighbor exists
      maze[x - 1][y].wallE = rightWall;

    if (x < mazeSize - 1) // Eastern neighbor exists
      maze[x + 1][y].wallW = leftWall;
    break;

  case WEST:
    // Update current cell
    maze[x][y].wallW = frontWall;
    maze[x][y].wallN = rightWall;
    maze[x][y].wallS = leftWall;

    // Update adjacent cells (symmetrically)
    if (x > 0) // Western neighbor exists
      maze[x - 1][y].wallE = frontWall;

    if (y < mazeSize - 1) // Northern neighbor exists
      maze[x][y + 1].wallS = rightWall;

    if (y > 0) // Southern neighbor exists
      maze[x][y - 1].wallN = leftWall;
    break;
  }

  // Mark the current cell as visited
  maze[x][y].visited = true;
}
// ===================================

// ==== FLOOD FILL ALGORITHM ====
void initializeFloodValues()
{
  //
  int goalX1 = mazeSize / 2 - 1;
  int goalY1 = mazeSize / 2 - 1;
  int goalX2 = mazeSize / 2;
  int goalY2 = mazeSize / 2;

  // Set all flood values based on Manhattan distance to goal
  for (int x = 0; x < mazeSize; x++)
  {
    for (int y = 0; y < mazeSize; y++)
    {
      // Calculate minimum Manhattan distance to any goal cell
      int d1 = abs(x - goalX1) + abs(y - goalY1);
      int d2 = abs(x - goalX2) + abs(y - goalY1);
      int d3 = abs(x - goalX1) + abs(y - goalY2);
      int d4 = abs(x - goalX2) + abs(y - goalY2);

      // Take the minimum distance
      maze[x][y].floodValue = min(min(d1, d2), min(d3, d4));
    }
  }
}
// get the smallest flood value among neighbors
int getSmallestNeighborFloodValue(int x, int y)
{
  int smallestValue = 255;

  //    check north
  if (!maze[x][y].wallN && y + 1 < mazeSize)
  {
    smallestValue = min(smallestValue, maze[x][y + 1].floodValue);
  }

  //   check south
  if (!maze[x][y].wallS && y - 1 >= 0)
  {
    smallestValue = min(smallestValue, maze[x][y - 1].floodValue);
  }
  //   check east
  if (!maze[x][y].wallE && x + 1 < mazeSize)
  {
    smallestValue = min(smallestValue, maze[x + 1][y].floodValue);
  }
  //   check west
  if (!maze[x][y].wallW && x - 1 >= 0)
  {
    smallestValue = min(smallestValue, maze[x - 1][y].floodValue);
  }
  return smallestValue;
}
// Check if the cell's flood value is correct (1 + min of accessible neighbors)
bool checkFloodValue(int x, int y)
{
  int smallestNeighbor = getSmallestNeighborFloodValue(x, y);
  return maze[x][y].floodValue == smallestNeighbor + 1;
}
// Update flood value of a cell to be 1 + minimum of accessible neighbors
void updateFloodValue(int x, int y)
{
  maze[x][y].floodValue = getSmallestNeighborFloodValue(x, y) + 1;
}
// Push accessible neighboring cells to the stack
void pushOpenNeighbors(int x, int y, STACK *stack)
{
  // Check North
  if (!maze[x][y].wallN && y + 1 < mazeSize)
  {
    push(stack, &maze[x][y + 1]);
  }

  // Check South
  if (!maze[x][y].wallS && y - 1 >= 0)
  {
    push(stack, &maze[x][y - 1]);
  }

  // Check East
  if (!maze[x][y].wallE && x + 1 < mazeSize)
  {
    push(stack, &maze[x + 1][y]);
  }

  // Check West
  if (!maze[x][y].wallW && x - 1 >= 0)
  {
    push(stack, &maze[x - 1][y]);
  }
}

// MAIN FLOOD FILL ALGORITHM
// Modified flood fill algorithm to use coordinates directly
void floodFill(int startX, int startY, bool isReverseMode = false)
{
  Stack stack;
  initStack(&stack);

  // Start with the current cell
  push(&stack, startX, startY);

  while (!isEmpty(&stack))
  {
    int currentX, currentY;
    if (!pop(&stack, currentX, currentY))
    {
      continue; // Skip if stack was empty
    }

    // Skip goal cells based on mode
    if (!isReverseMode)
    {
      // When going to the goal, don't update goal cells
      if ((currentX == mazeSize / 2 || currentX == mazeSize / 2 - 1) &&
          (currentY == mazeSize / 2 || currentY == mazeSize / 2 - 1))
      {
        continue;
      }
    }
    else
    {
      // When returning to start, don't update start cell
      if (currentX == 0 && currentY == 0)
      {
        continue;
      }
    }

    // Check if the cell's flood value needs to be updated
    if (!checkFloodValue(currentX, currentY))
    {
      // Update the flood value
      updateFloodValue(currentX, currentY);

      // Add accessible neighbors to the stack
      // Check North
      if (!maze[currentX][currentY].wallN && currentY + 1 < mazeSize)
      {
        push(&stack, currentX, currentY + 1);
      }

      // Check South
      if (!maze[currentX][currentY].wallS && currentY - 1 >= 0)
      {
        push(&stack, currentX, currentY - 1);
      }

      // Check East
      if (!maze[currentX][currentY].wallE && currentX + 1 < mazeSize)
      {
        push(&stack, currentX + 1, currentY);
      }

      // Check West
      if (!maze[currentX][currentY].wallW && currentX - 1 >= 0)
      {
        push(&stack, currentX - 1, currentY);
      }
    }
  }
}

// Get the direction with the smallest flood value
Direction getOptimalDirection()
{
  int x = robotX;
  int y = robotY;
  int smallestValue = 255;
  Direction optimalDirection = robotHeading;

  // Check North
  if (!maze[x][y].wallN && y + 1 < mazeSize && maze[x][y + 1].floodValue < smallestValue)
  {
    smallestValue = maze[x][y + 1].floodValue;
    optimalDirection = NORTH;
  }

  // Check East
  if (!maze[x][y].wallE && x + 1 < mazeSize && maze[x + 1][y].floodValue < smallestValue)
  {
    smallestValue = maze[x + 1][y].floodValue;
    optimalDirection = EAST;
  }

  // Check South
  if (!maze[x][y].wallS && y - 1 >= 0 && maze[x][y - 1].floodValue < smallestValue)
  {
    smallestValue = maze[x][y - 1].floodValue;
    optimalDirection = SOUTH;
  }

  // Check West
  if (!maze[x][y].wallW && x - 1 >= 0 && maze[x - 1][y].floodValue < smallestValue)
  {
    smallestValue = maze[x - 1][y].floodValue;
    optimalDirection = WEST;
  }

  return optimalDirection;
}

// Move to the next cell with smallest flood value
void moveToNextCell()
{
  // Get optimal direction
  Direction optimalDirection = getOptimalDirection();

  // Turn robot to face optimal direction
  while (robotHeading != optimalDirection)
  {
    if ((robotHeading + 1) % 4 == optimalDirection)
    {
      rotateRight90();
      robotHeading = optimalDirection;
    }
    else if ((robotHeading + 3) % 4 == optimalDirection)
    {
      rotateLeft90();
      robotHeading = optimalDirection;
    }
    else
    {
      rotate180();
      robotHeading = (Direction)((robotHeading + 2) % 4);
    }
  }

  // Move forward
  moveForwardOneCell();

  // Update robot position
  switch (robotHeading)
  {
  case NORTH:
    robotY++;
    break;
  case EAST:
    robotX++;
    break;
  case SOUTH:
    robotY--;
    break;
  case WEST:
    robotX--;
    break;
  }
}

// === MAIN MAZE SOLVING ALGORITHM ===
void solveMaze()
{
  // Initialize flood values
  initializeFloodValues();

  // DEFINE GOAL COORDINATES
  int goalX1 = mazeSize / 2 - 1;
  int goalY1 = mazeSize / 2 - 1;
  int goalX2 = mazeSize / 2;
  int goalY2 = mazeSize / 2;

  bool goalReached = false;

  while (!goalReached)
  {
    // SCAN WALLS AROUND CURRENT POSITION
    scanWalls();

    // UPDATE MAZE WITH DITICTED WALLS
    updateMazeWithWalls();

    // RUN FLOOD FILL LGORITHM TO UPDATE FLOOD VALUES
    floodFill(robotX, robotY);

    // Move to the next cell with smallest flood value
    moveToNextCell();

    // Check if we've reached the goal
    if ((robotX == goalX1 || robotX == goalX2) &&
        (robotY == goalY1 || robotY == goalY2))
    {
      goalReached = true;
      Serial.println("Goal reached!");
    }
  }
}
