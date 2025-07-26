#include <stdio.h>
#include <stdbool.h>
#include "API.h"

// === STACK IMPLEMENTATION FOR FLOOD FILL ===
#define STACK_SIZE 256
#include <stdlib.h> // for abs()

typedef struct {
    int x;
    int y;
} StackItem;

typedef struct {
    StackItem items[STACK_SIZE];
    int top;
} Stack;

void initStack(Stack* stack) {
    stack->top = -1;
}

bool isEmpty(Stack* stack) {
    return stack->top == -1;
}

bool isFull(Stack* stack) {
    return stack->top == STACK_SIZE - 1;
}

void push(Stack* stack, int x, int y) {
    if (!isFull(stack)) {
        stack->top++;
        stack->items[stack->top].x = x;
        stack->items[stack->top].y = y;
    } else {
        fprintf(stderr, "Stack overflow\n");
    }
}

bool pop(Stack* stack, int* x, int* y) {
    if (!isEmpty(stack)) {
        *x = stack->items[stack->top].x;
        *y = stack->items[stack->top].y;
        stack->top--;
        return true;
    } else {
        fprintf(stderr, "Stack underflow\n");
        return false;
    }
}

// === CELL STRUCT ===
typedef struct {
    bool wallN, wallS, wallE, wallW;
    int floodValue;
    bool visited;
} CELL;

#define MAZE_SIZE 16
CELL maze[MAZE_SIZE][MAZE_SIZE];

// === ROBOT POSITION ===
int robotX = 0;
int robotY = 0;

typedef enum { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 } Direction;
Direction robotHeading = NORTH;


// === FUNCTION DECLARATIONS ===
void initializeFloodValues();   // You'll send this later
void solveMaze();               // You'll send this too

int main() {
    // === Init all cells in the maze ===
    for (int i = 0; i < MAZE_SIZE; i++) {
        for (int j = 0; j < MAZE_SIZE; j++) {
            maze[i][j].wallN = false;
            maze[i][j].wallS = false;
            maze[i][j].wallE = false;
            maze[i][j].wallW = false;
            maze[i][j].visited = false;
        }
    }

    // === Set outer walls ===
    for (int i = 0; i < MAZE_SIZE; i++) {
        maze[i][0].wallS = true;
        maze[i][MAZE_SIZE - 1].wallN = true;
    }
    for (int i = 0; i < MAZE_SIZE; i++) {
        maze[0][i].wallW = true;
        maze[MAZE_SIZE - 1][i].wallE = true;
    }

    fprintf(stderr, "Maze initialized\n");

    // === Init flood fill values ===
    initializeFloodValues();

    fprintf(stderr, "Ready to solve the maze\n");

    // === Start solving ===
    solveMaze();

    return 0;
}

// === MOVEMENT FUNCTIONS FOR SIMULATOR ===
void moveForwardOneCell() {
    fprintf(stderr, "Moving forward 1 cell...\n");
    API_moveForward();
}

void rotateLeft90() {
    fprintf(stderr, "Rotating left 90 degrees...\n");
    API_turnLeft();
    // Update heading
    robotHeading = (robotHeading + 3) % 4;  // equivalent to -1 modulo 4
}

void rotateRight90() {
    fprintf(stderr, "Rotating right 90 degrees...\n");
    API_turnRight();
    robotHeading = (robotHeading + 1) % 4;
}

void rotate180() {
    fprintf(stderr, "Rotating 180 degrees...\n");
    API_turnLeft();
    API_turnLeft();
    robotHeading = (robotHeading + 2) % 4;
}

// === WALL SCANNING (SIMULATOR VERSION) ===
bool frontWall = false;
bool leftWall = false;
bool rightWall = false;

void scanWalls() {
    frontWall = API_wallFront();
    leftWall = API_wallLeft();
    rightWall = API_wallRight();

    fprintf(stderr, "Front: %s | Left: %s | Right: %s\n",
            frontWall ? "Wall" : "Open",
            leftWall ? "Wall" : "Open",
            rightWall ? "Wall" : "Open");
}

bool wallInFront() {
    return frontWall;
}

bool wallOnLeft() {
    return leftWall;
}

bool wallOnRight() {
    return rightWall;
}


void updateMazeWithWalls() {
    // Get wall info from simulator
    scanWalls();

    int x = robotX;
    int y = robotY;

    switch (robotHeading) {
        case NORTH:
            maze[x][y].wallN = frontWall;
            maze[x][y].wallE = rightWall;
            maze[x][y].wallW = leftWall;

            if (y < MAZE_SIZE - 1)
                maze[x][y + 1].wallS = frontWall;
            if (x < MAZE_SIZE - 1)
                maze[x + 1][y].wallW = rightWall;
            if (x > 0)
                maze[x - 1][y].wallE = leftWall;
            break;

        case EAST:
            maze[x][y].wallE = frontWall;
            maze[x][y].wallS = rightWall;
            maze[x][y].wallN = leftWall;

            if (x < MAZE_SIZE - 1)
                maze[x + 1][y].wallW = frontWall;
            if (y > 0)
                maze[x][y - 1].wallN = rightWall;
            if (y < MAZE_SIZE - 1)
                maze[x][y + 1].wallS = leftWall;
            break;

        case SOUTH:
            maze[x][y].wallS = frontWall;
            maze[x][y].wallW = rightWall;
            maze[x][y].wallE = leftWall;

            if (y > 0)
                maze[x][y - 1].wallN = frontWall;
            if (x > 0)
                maze[x - 1][y].wallE = rightWall;
            if (x < MAZE_SIZE - 1)
                maze[x + 1][y].wallW = leftWall;
            break;

        case WEST:
            maze[x][y].wallW = frontWall;
            maze[x][y].wallN = rightWall;
            maze[x][y].wallS = leftWall;

            if (x > 0)
                maze[x - 1][y].wallE = frontWall;
            if (y < MAZE_SIZE - 1)
                maze[x][y + 1].wallS = rightWall;
            if (y > 0)
                maze[x][y - 1].wallN = leftWall;
            break;
    }

    maze[x][y].visited = true;
}



void initializeFloodValues() { // use manhaten distance to the goal cells
    int goalX1 = MAZE_SIZE / 2 - 1;
    int goalY1 = MAZE_SIZE / 2 - 1;
    int goalX2 = MAZE_SIZE / 2;
    int goalY2 = MAZE_SIZE / 2;

// It calculates the Manhattan distance from every cell to the center.
// That just means:

// Distance = |current_x - center_x| + |current_y - center_y|
// But since the maze center is actually four cells (in the middle), the function:

// Calculates the distance to all 4 center cells

// Picks the smallest distance (the closest one)

    for (int x = 0; x < MAZE_SIZE; x++) {
        for (int y = 0; y < MAZE_SIZE; y++) {
            int d1 = abs(x - goalX1) + abs(y - goalY1);
            int d2 = abs(x - goalX2) + abs(y - goalY1);
            int d3 = abs(x - goalX1) + abs(y - goalY2);
            int d4 = abs(x - goalX2) + abs(y - goalY2);
            int min1 = (d1 < d2) ? d1 : d2;
            int min2 = (d3 < d4) ? d3 : d4;
            maze[x][y].floodValue = (min1 < min2) ? min1 : min2;
        }
    }
}

int getSmallestNeighborFloodValue(int x, int y) {
    int smallest = 255;

    if (!maze[x][y].wallN && y + 1 < MAZE_SIZE)
        if (maze[x][y + 1].floodValue < smallest)
            smallest = maze[x][y + 1].floodValue;

    if (!maze[x][y].wallS && y - 1 >= 0)
        if (maze[x][y - 1].floodValue < smallest)
            smallest = maze[x][y - 1].floodValue;

    if (!maze[x][y].wallE && x + 1 < MAZE_SIZE)
        if (maze[x + 1][y].floodValue < smallest)
            smallest = maze[x + 1][y].floodValue;

    if (!maze[x][y].wallW && x - 1 >= 0)
        if (maze[x - 1][y].floodValue < smallest)
            smallest = maze[x - 1][y].floodValue;

    return smallest;
}

bool checkFloodValue(int x, int y) {
    int smallest = getSmallestNeighborFloodValue(x, y);
    return maze[x][y].floodValue == smallest + 1;
}

void updateFloodValue(int x, int y) {
    maze[x][y].floodValue = getSmallestNeighborFloodValue(x, y) + 1;
}

// Corrected version using coordinates, not CELL*
void pushOpenNeighbors(int x, int y, Stack* stack) {
    if (!maze[x][y].wallN && y + 1 < MAZE_SIZE)
        push(stack, x, y + 1);
    if (!maze[x][y].wallS && y - 1 >= 0)
        push(stack, x, y - 1);
    if (!maze[x][y].wallE && x + 1 < MAZE_SIZE)
        push(stack, x + 1, y);
    if (!maze[x][y].wallW && x - 1 >= 0)
        push(stack, x - 1, y);
}


void floodFill(int startX, int startY, bool isReverseMode) {
//  It updates the flood values (distance numbers) in the maze after the robot discovers new walls.

//The robot says: “Now that I’ve seen some walls, I need to recalculate which cells are closer to the goal.”


    Stack stack;
    initStack(&stack);
    
    push(&stack, startX, startY);
    
    while (!isEmpty(&stack)) {
        int currentX, currentY;
        if (!pop(&stack, &currentX, &currentY)) {
            continue;
        }

        // Skip goal cells
        if (!isReverseMode) {
            if ((currentX == MAZE_SIZE / 2 || currentX == MAZE_SIZE / 2 - 1) &&
                (currentY == MAZE_SIZE / 2 || currentY == MAZE_SIZE / 2 - 1)) {
                continue;
            }
        } else {
            if (currentX == 0 && currentY == 0) {
                continue;
            }
        }

        if (!checkFloodValue(currentX, currentY)) {
            updateFloodValue(currentX, currentY);

            if (!maze[currentX][currentY].wallN && currentY + 1 < MAZE_SIZE)
                push(&stack, currentX, currentY + 1);
            if (!maze[currentX][currentY].wallS && currentY - 1 >= 0)
                push(&stack, currentX, currentY - 1);
            if (!maze[currentX][currentY].wallE && currentX + 1 < MAZE_SIZE)
                push(&stack, currentX + 1, currentY);
            if (!maze[currentX][currentY].wallW && currentX - 1 >= 0)
                push(&stack, currentX - 1, currentY);
        }
    }
}

Direction getOptimalDirection() {
    int x = robotX;
    int y = robotY;
    int smallest = 255;
    Direction best = robotHeading;

    if (!maze[x][y].wallN && y + 1 < MAZE_SIZE && maze[x][y + 1].floodValue < smallest) {
        smallest = maze[x][y + 1].floodValue;
        best = NORTH;
    }
    if (!maze[x][y].wallE && x + 1 < MAZE_SIZE && maze[x + 1][y].floodValue < smallest) {
        smallest = maze[x + 1][y].floodValue;
        best = EAST;
    }
    if (!maze[x][y].wallS && y - 1 >= 0 && maze[x][y - 1].floodValue < smallest) {
        smallest = maze[x][y - 1].floodValue;
        best = SOUTH;
    }
    if (!maze[x][y].wallW && x - 1 >= 0 && maze[x - 1][y].floodValue < smallest) {
        smallest = maze[x - 1][y].floodValue;
        best = WEST;
    }

    return best;
}


void moveToNextCell() {
    Direction optimalDirection = getOptimalDirection();

    // Rotate to face the optimal direction
    while (robotHeading != optimalDirection) {
        if ((robotHeading + 1) % 4 == optimalDirection) {
            rotateRight90();
        } else if ((robotHeading + 3) % 4 == optimalDirection) {
            rotateLeft90();
        } else {
            rotate180();
        }
    }

    // Move forward
    moveForwardOneCell();

    // Update position
    switch (robotHeading) {
        case NORTH: robotY++; break;
        case EAST:  robotX++; break;
        case SOUTH: robotY--; break;
        case WEST:  robotX--; break;
    }
}

void solveMaze() {
    initializeFloodValues();

    int goalX1 = MAZE_SIZE / 2 - 1;
    int goalY1 = MAZE_SIZE / 2 - 1;
    int goalX2 = MAZE_SIZE / 2;
    int goalY2 = MAZE_SIZE / 2;

    bool goalReached = false;

    while (!goalReached) {
        scanWalls();
        updateMazeWithWalls();
        floodFill(robotX, robotY, false);  // false = not reverse mode
        moveToNextCell();

        if ((robotX == goalX1 || robotX == goalX2) &&
            (robotY == goalY1 || robotY == goalY2)) {
            goalReached = true;
            fprintf(stderr, "Goal reached!\n");
        }
    }
}
