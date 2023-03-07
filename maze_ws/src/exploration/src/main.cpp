#include "ros/ros.h"
#include <iostream>
#include <vector>
#include <utility>
#include <stack>
// #include <cstdlib>
#include <algorithm>
#include <ctype.h>
#include <map>
#include <fstream>
#include <string>

using namespace std;

#include "ros/callback_queue.h"
#include "exploration/Trigger.h"

// Move Base
#include "move_base_msgs/MoveBaseAction.h"
// #include "move_base_msgs/MoveBaseGoal.h"
#include "actionlib/client/simple_action_client.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"

#include "Tile.h"
// #include "Movement.h"
#include "Map.h"
#include "Dijkstra.h"

// 0: north, 1: east, 2: south, 3: west
int rDirection = 0;

string curInstruction = "";
ros::CallbackQueue callQueue;

#define DEBUG true
#define useros false

struct ClientScope
{
    // move_base_msgs::MoveBaseFeedbackConstPtr &feedback;
    string feedback;
    uint8_t result;
    string textRes;
    // int status;
    uint8_t status;
    bool resultReceived;
    bool startedGoal;

    // Constructor
    ClientScope() : feedback(""), result(10), resultReceived(false) {}
};

ClientScope scope;

void doneCb(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResult::ConstPtr &result)
{
    ROS_INFO_STREAM("Finished in state " << state.toString());
    // ROS_INFO_STREAM("Answer: " << result->sequence.back());

    scope.result = state.state_;
    scope.textRes = state.getText();

    // scope.result = result->status.goal_id.id;
    scope.resultReceived = true;
    scope.startedGoal = false;
}

// void feedbackCB(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback)
void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback)
{
    // ROS_INFO_STREAM("Feedback x: " << feedback->base_position.pose.position.x);
    // ROS_INFO_STREAM("Feedback y: " << feedback->base_position.pose.position.y);
    // ROS_INFO_STREAM("Feedback z: " << feedback->base_position.pose.position.z);
    // ROS_INFO_STREAM("GoalID: " << feedback->status.goal_id.id);
    // ROS_INFO_STREAM("x: " << feedback->feedback.base_position.pose.position.x);
    // ROS_INFO_STREAM("y: " << feedback->feedback.base_position.pose.position.y);
    // ROS_INFO_STREAM("z: " << feedback->feedback.base_position.pose.position.z);
    // ROS_INFO_STREAM("Got Feedback of length " << feedback->sequence.size());

    scope.feedback = feedback->base_position.pose.position.x;

    // scope.feedback = feedback->.goal_id.id;
}

void activeCb()
{
    ROS_INFO("Goal just went active");
    scope.resultReceived = false;
    scope.startedGoal = true;
}

void statusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr &msg)
{
    // ROS_INFO_STREAM("Status: " << msg->goal_id.id);
    ROS_INFO("Status: %d", msg->status_list[0].status);
    if (scope.startedGoal)
        scope.status = msg->status_list[0].status;
    // ROS_INFO_STREAM("Status: " << msg->status_list[0].text);
    // ROS_INFO_STREAM("Status: " << msg->status_list[0].goal_id.id);
}

int getColor()
{
    return 0;
}

void recovery()
{
    ROS_INFO("Recovery");
}

void movementClientAsync(move_base_msgs::MoveBaseGoal goal)
{
    ROS_INFO("movementClientAsync");

    // tell the action client that we want to spin a thread by default
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

    // wait for the action server to come up
    while (!ac.waitForServer(ros::Duration(1.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
        ros::spinOnce();
    }

    ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

    scope.resultReceived = false;
    scope.status = actionlib_msgs::GoalStatus::PENDING;

    // ros::spinOnce();

    while (ros::ok() && !scope.resultReceived &&
           scope.status != actionlib_msgs::GoalStatus::PREEMPTED &&
           scope.status != actionlib_msgs::GoalStatus::SUCCEEDED &&
           scope.status != actionlib_msgs::GoalStatus::ABORTED &&
           scope.status != actionlib_msgs::GoalStatus::REJECTED &&
           scope.status != actionlib_msgs::GoalStatus::RECALLED)

    //(scope.status == actionlib_msgs::GoalStatus::ACTIVE || scope.status == actionlib_msgs::GoalStatus::PENDING || scope.status == actionlib_msgs::GoalStatus::RECALLED || scope.status == actionlib_msgs::GoalStatus::RECALLING))
    {
        ROS_INFO("in movement client async loop, Status: %d", scope.status);
        ROS_INFO("resultReceived: %d", scope.resultReceived);

        if (false)
        {
            ac.cancelAllGoals();
            recovery();

            break;
        }

        ros::spinOnce();
    }

    ROS_INFO("movementClientAsync result: %d", scope.status);
    return;
}

void sendGoal(geometry_msgs::Pose pose)
{
    ROS_INFO("sendGoal");

    move_base_msgs::MoveBaseGoal goal;

    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header.stamp = ros::Time::now();
    poseStamped.header.frame_id = "base_link";

    poseStamped.pose = pose;

    goal.target_pose = poseStamped;
    movementClientAsync(goal);
}

// void handleUnitMovements(exploration::Trigger::Request &req, exploration::Trigger::Response &res)
void handleUnitMovements(int movement)
{
    if (useros)
    {
        ROS_INFO_STREAM("handleUnitMovements: " << movement);

        geometry_msgs::Pose pose;

        if (movement == 0) // North
        {
            ROS_INFO("North");

            pose.position.x = 0.3;
            pose.position.y = 0;
            pose.position.z = 0;
            pose.orientation.x = 0;
            pose.orientation.y = 0;
            pose.orientation.z = 0;
            pose.orientation.w = 1;
        }
        else if (movement == 1) // Turn right
        {
            ROS_INFO("Turn right");

            pose.position.x = 0;
            pose.position.y = 0;
            pose.position.z = 0;
            pose.orientation = tf::createQuaternionMsgFromYaw(-M_PI / 2);
        }
        else if (movement == 2) // South
        {
            ROS_INFO("Backwards");

            pose.position.x = -0.3;
            pose.position.y = 0;
            pose.position.z = 0;
            pose.orientation.x = 0;
            pose.orientation.y = 0;
            pose.orientation.z = 0;
            pose.orientation.w = 1;
        }
        else if (movement == 3) // Turn left
        {
            ROS_INFO("Turn left");

            pose.position.x = 0;
            pose.position.y = 0;
            pose.position.z = 0;
            pose.orientation = tf::createQuaternionMsgFromYaw(M_PI / 2);
        }

        sendGoal(pose);
    }
}

/* bool sendInstruction(exploration::Trigger::Request &req, exploration::Trigger::Response &res)
{
    res.message = curInstruction;
    // ROS_INFO_STREAM("Sending instruction: " << curInstruction);
    // ROS_INFO(curInstruction.c_str());

    curInstruction = "";

    return true;
}

void setInstruction(string instruction, Map &mapa)
{
    // ROS_INFO_STREAM("setInstruction: " << instruction);
    curInstruction = instruction;

    if (DEBUG)
    {
        mapa.printMaze(rDirection);
    }

    // while (ros::ok() && callQueue.isEmpty())
    // {
    //     ros::Duration(0.2).sleep();
    //     ROS_INFO("Waiting for request");
    // }

    // if (callQueue.isEmpty())
    //     ROS_INFO("no service calls on queue");

    ros::Duration(1.5).sleep();

    // if (callQueue.isEmpty())
    //     ROS_INFO("no service calls on queue");

    // callQueue.callOne(ros::WallDuration(5.0));
    ros::spinOnce();
} */

void moveForward(int &rDirection, Map &mapa)
{
    // cout << "forward" << endl;
    ROS_INFO("forward");

    handleUnitMovements(0);

    if (DEBUG)
    {
        mapa.printMaze(rDirection);
        // cin.get();
        ros::Duration(0.1).sleep();
    }
}

void moveBackward(int &rDirection, Map &mapa)
{
    // cout << "forward" << endl;
    ROS_INFO("backward");

    handleUnitMovements(2);

    if (DEBUG)
    {
        mapa.printMaze(rDirection);
        // cin.get();
        ros::Duration(0.1).sleep();
    }
}

// Gira a la izquierda y modifica la direccion
void left(int &rDirection, Map &mapa)
{
    (rDirection == 0) ? rDirection = 3 : rDirection--;
    // cout << "left" << endl;
    ROS_INFO("left");

    handleUnitMovements(3);

    if (DEBUG)
    {
        mapa.printMaze(rDirection);
        // cin.get();
        ros::Duration(0.1).sleep();
    }
}

// Gira a la derecha y modifica la direccion
void right(int &rDirection, Map &mapa)
{
    (rDirection == 3) ? rDirection = 0 : rDirection++;
    // cout << "right" << endl;
    ROS_INFO("right");

    handleUnitMovements(2);

    if (DEBUG)
    {
        mapa.printMaze(rDirection);
        // cin.get();
        ros::Duration(0.1).sleep();
    }
}

// Gira a la direccion indicada
void rotateTo(int &rDirection, int newDirection, Map &mapa)
{
    // ROS_INFO_STREAM("rotateTo: " << newDirection);
    if (rDirection + 2 == newDirection or rDirection - 2 == newDirection)
    {
        left(rDirection, mapa);
        left(rDirection, mapa);
    }
    else if (rDirection + 1 == newDirection or rDirection - 3 == newDirection)
    {
        right(rDirection, mapa);
    }
    else if (rDirection - 1 == newDirection or rDirection + 3 == newDirection)
    {
        left(rDirection, mapa);
    }
}

// Mueve el robot hacia el norte
void moveNorth(int &yMaze, int &rDirection, Map &mapa)
{
    // cout << "moveNorth" << endl;

    if (rDirection == 2)
        moveBackward(rDirection, mapa);

    rotateTo(rDirection, 0, mapa);
    moveForward(rDirection, mapa);

    // yMaze--;
}

// Mueve el robot hacia el sur
void moveSouth(int &yMaze, int &rDirection, Map &mapa)
{
    // cout << "moveSouth" << endl;

    if (rDirection == 0)
        moveBackward(rDirection, mapa);

    rotateTo(rDirection, 2, mapa);
    moveForward(rDirection, mapa);

    // yMaze++;
}

// Mueve el robot hacia el este
void moveEast(int &xMaze, int &rDirection, Map &mapa)
{
    // cout << "moveEast" << endl;

    if (rDirection == 3)
        moveBackward(rDirection, mapa);

    rotateTo(rDirection, 1, mapa);
    moveForward(rDirection, mapa);

    // xMaze++;
}

// Mueve el robot hacia el oeste
void moveWest(int &xMaze, int &rDirection, Map &mapa)
{
    // cout << "moveWest" << endl;

    if (rDirection == 1)
        moveBackward(rDirection, mapa);

    rotateTo(rDirection, 3, mapa);
    moveForward(rDirection, mapa);

    // xMaze--;
}

// Calcula la posicion en la que estaria el robot al moverse a la direccion indicada por key
void calcPos(vector<int>& pos, string key, Map &mapa)
{
    int rampDir = mapa.rampDirection(key);

    if (key == "north")
    {
        pos[1]++;

        if (rampDir != -1)
        {
            if (rampDir == 0)
                pos[2]++;
            else if (rampDir == 2)
                pos[2]--;
        }
    }
    else if (key == "south")
    {
        pos[1]--;

        if (rampDir != -1)
        {
            if (rampDir == 0)
                pos[2]--;
            else if (rampDir == 2)
                pos[2]++;
        }
    }
    else if (key == "east")
    {
        pos[0]++;

        if (rampDir != -1)
        {
            if (rampDir == 1)
                pos[2]++;
            else if (rampDir == 3)
                pos[2]--;
        }
    }
    else if (key == "west")
    {
        pos[0]--;

        if (rampDir != -1)
        {
            if (rampDir == 1)
                pos[2]--;
            else if (rampDir == 3)
                pos[2]++;
        }
    }
}

// Mueve el robot hacia la direccion indicada por key y regresa el tile al que se movio
Tile *move(Tile *tile, string key, int &xMaze, int &yMaze, int &rDirection, Map &mapa)
{
    if (tile->adjacentTiles[key])
    {
        calcPos(mapa.pos, key, mapa);
        mapa.moveMaze(key);

        if (key == "north")
        {
            moveNorth(yMaze, rDirection, mapa);
        }
        else if (key == "east")
        {
            moveEast(xMaze, rDirection, mapa);
        }
        else if (key == "south")
        {
            moveSouth(yMaze, rDirection, mapa);
        }
        else if (key == "west")
        {
            moveWest(xMaze, rDirection, mapa);
        }

        return tile->adjacentTiles[key];
    }

    return nullptr;
}

string posvectorToString(vector<int> pos)
{
    return to_string(pos[0]) + "," + to_string(pos[1]) + "," + to_string(pos[2]);
}

// Regresa la distancia en cm a la pared en la direccion dada
int getDistance(string key, Map &mapa)
{
    // ROS_INFO_STREAM("getDistance: " << key);
    // if (key == "north")
    //     rotateTo(rDirection, 0);
    // else if (key == "east")
    //     rotateTo(rDirection, 1);
    // else if (key == "south")
    //     rotateTo(rDirection, 2);
    // else if (key == "west")
    //     rotateTo(rDirection, 3);

    if (mapa.getChar(key) == '#')
        return 5;
    else
        return 30;
}

bool isWall(string key, Map &mapa)
{
    // ROS_INFO_STREAM("isWall: " << key);
    if (getDistance(key, mapa) > 20)
        return false;
    else
        return true;
}

// Regresa true si el caracter en la posicion establecida por key es una rampa
bool isRamp(string key, Map &mapa)
{
    if (mapa.getChar(key) == 'r')
        return true;
    else
        return false;
}

// Sigue las instrucciones del stack y regresa el tile al que se movio
Tile *followPath(stack<string> &path, Tile *tile, Map &mapa)
{
    while (!path.empty())
    {
        tile = move(tile, path.top(), mapa.xMaze, mapa.yMaze, rDirection, mapa);
        cout << path.top() << "\t" << mapa.pos[0] << ", " << mapa.pos[1] << ", " << mapa.pos[2] << endl;
        path.pop();
    }

    return tile;
}

int checkVictims(Map &mapa)
{
    return isdigit(mapa.maze[mapa.yMaze][mapa.xMaze]) ? mapa.maze[mapa.yMaze][mapa.xMaze] : 0;
}

Tile *createTile(vector<int> pos, char c, string key, vector<int> &adjPos, Map &mapa)
{
    Tile *newTile = new Tile(pos, 1, 0);

    if (c == 'a') // casilla azul
    {
        newTile->weight = 100;
    }
    else if (c == 'b') // bumper
    {
        newTile->weight = 100;
    }
    else if (c == '/') // escaleras
    {
        newTile->weight = 100;
    }
    else if (c == 'r') // rampa
    {
        newTile->weight = 150;
        newTile->rampa = mapa.rampDirection(key);
    }
    else if (c == 'c') //checkpoint
    {
        
    }
    else if (checkVictims(mapa))
    {
        newTile->victim = checkVictims(mapa);
    }

    return newTile;
}

void printMap(Map &mapa)
{
    ROS_INFO("xrmaze: %d, yrmaze: %d, zrmaze: %d", mapa.xMaze, mapa.yMaze, mapa.zMaze);
    ROS_INFO("unvisited: ");
    for (auto i : mapa.unvisited)
    {
        ROS_INFO("pos: %d, %d, %d", i->pos[0], i->pos[1], i->pos[2]);
    }

    ROS_INFO("Tile Pos: %d, %d, %d", mapa.tile->pos[0], mapa.tile->pos[1], mapa.tile->pos[2]);

    ROS_INFO("recovpos: %d, %d, %d", mapa.recovpos[0], mapa.recovpos[1], mapa.recovpos[2]);
}

// Funcion principal, se encarga de llamar a las funciones necesarias para explorar el laberinto completamente y regresar al inicio
void explore(bool checkpoint, int argc, char **argv)
{
    ros::init(argc, argv, "mainAlgorithm");
    ros::NodeHandle n;

    Map mapa;
    int prueba;


    if (checkpoint)
    {
        ROS_INFO_STREAM("Checkpoint");

        // {
        std::ifstream ifs("./checkpoint.txt");
        boost::archive::text_iarchive ia(ifs);
        ia >> mapa;

        printMap(mapa);
        cin.get();
        //     ia >> mapa;
        // }
    }
    else
    {
        ROS_INFO_STREAM("No checkpoint");
    }

    // mapa.printMaze(rDirection);

    // ros::Duration(3).sleep();
    Tile *startTile = mapa.tile;
    Tile *newTile = nullptr;
    vector<string> keys = {"north", "east", "south", "west"};

    vector<int> newPos = mapa.pos;

    ROS_INFO("newPos created");

    char c;

    stack<string> path;

    int steps = 2;

    do
    {
        ROS_INFO("Enters do while loop");

        mapa.tile->visited = true;
        for (auto &&key : keys)
        {
            newPos = mapa.pos;

            if (mapa.tile->adjacentTiles[key] == nullptr && !mapa.tile->walls[key])
            {
                c = mapa.getChar(key);

                calcPos(newPos, key, mapa);

                if (mapa.tiles.count(posvectorToString(newPos)))
                {
                    newTile = mapa.tiles.at(posvectorToString(newPos)); // Ya existe una tile para esa casilla

                    mapa.tile->appendTile(newTile, key);
                }
                else // no existe tile
                {
                    if (isWall(key, mapa))
                    {
                        mapa.tile->walls[key] = true;
                    }
                    else if (c == 'n')
                    {
                        mapa.tile->walls[key] = true;
                    }
                    else
                    {
                        newTile = createTile(newPos, c, key, mapa.pos, mapa);
                        mapa.tile->appendTile(newTile, key);
                        mapa.unvisited.push_back(newTile);

                        mapa.tiles.insert({posvectorToString(newPos), newTile});
                        if (newTile->rampa != -1)
                        {
                            newPos[2] = mapa.pos[2];
                            mapa.tiles.insert({posvectorToString(newPos), newTile});
                        }

                        steps = 2;
                    }
                }
            }
        }

        {
            std::ofstream ofs("./checkpoint.txt");
            boost::archive::text_oarchive oa(ofs);
            oa << mapa;

            ROS_INFO("Checkpoint saved");
        }

        path = bestUnvisited(mapa.tile, mapa.unvisited, mapa.tiles, keys);
        mapa.tile = followPath(path, mapa.tile, mapa);

        if (DEBUG)
            mapa.printMaze(rDirection);
        // system("pause");
        // cin.get();

        if (mapa.unvisited.empty())
        {
            steps--;
        }

    } while (ros::ok() && steps > 0);

    mapa.unvisited.push_back(startTile);
    path = bestUnvisited(mapa.tile, mapa.unvisited, mapa.tiles, keys);
    mapa.tile = followPath(path, mapa.tile, mapa);

    mapa.printMaze(rDirection);
}

int main(int argc, char **argv)
{
    try
    {
        if (getColor() == 1) // Checkpoint
        {
            explore(true, argc, argv);
        }
        else
        {
            explore(true, argc, argv);
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }

    // ros::ServiceServer service = n.advertiseService("trigger", sendInstruction);
    // ROS_INFO("Ready to send instruction.");

    // curInstruction = "";

    /*     vector<Tile *> unvisited;
        Tile *tile = new Tile(vector<int>{0, 0, 0}, 1, 0);

        // InstructionServer server(argc, argv);

        printMaze(rDirection);
        // explore(tile, unvisited, server);

        Tile *startTile = tile;
        Tile *newTile = nullptr;
        vector<string> keys = {"north", "east", "south", "west"};

        vector<int> pos = {0, 0, 0};
        vector<int> newPos = pos;

        map<string, Tile *> tiles = {{posvectorToString(pos), startTile}};

        char c;

        stack<string> path;

        int steps = 2;

        do
        {
            tile->visited = true;
            for (auto &&key : keys)
            {
                newPos = pos;

                if (tile->adjacentTiles[key] == nullptr && !tile->walls[key])
                {
                    c = getChar(key);

                    calcPos(newPos, key);

                    if (tiles.count(posvectorToString(newPos)))
                    {
                        newTile = tiles.at(posvectorToString(newPos)); // Ya existe una tile para esa casilla

                        tile->appendTile(newTile, key);
                    }
                    else // no existe tile
                    {
                        if (isWall(key))
                        {
                            tile->walls[key] = true;
                        }
                        else if (c == 'n')
                        {
                            tile->walls[key] = true;
                        }
                        else
                        {
                            newTile = createTile(newPos, c, key, pos);
                            tile->appendTile(newTile, key);
                            unvisited.push_back(newTile);

                            tiles.insert({posvectorToString(newPos), newTile});
                            if (newTile->rampa != -1)
                            {
                                newPos[2] = pos[2];
                                tiles.insert({posvectorToString(newPos), newTile});
                            }

                            steps = 2;
                        }
                    }
                }
            }

            path = bestUnvisited(tile, unvisited, tiles, keys);
            tile = followPath(path, tile, pos);

            if (DEBUG)
                printMaze(rDirection);
            // system("pause");
            // cin.get();

            if (unvisited.empty())
            {
                steps--;
            }

        } while (ros::ok() && steps > 0);

        unvisited.push_back(startTile);
        path = bestUnvisited(tile, unvisited, tiles, keys);
        tile = followPath(path, tile, pos);

        printMaze(rDirection); */

    return 0;
}
