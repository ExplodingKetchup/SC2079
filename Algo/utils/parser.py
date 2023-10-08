import json
import numpy as np


def test():
    try:
        with open(
            r"C:\Users\pcset\Documents\GitHub\SC2079\Algo\utils\resultAndroid.json",  # change this according to your local path
            "r",
            encoding="utf-8",
        ) as file:
            data = json.load(file)

        # Transform the data
        transformedObject = {
            "obstacles": [
                f'{item["x"]},{item["y"]},{item["d"]},{item["id"]}' for item in data
            ]
        }

        # Save the transformed data to example2.json
        with open(
            r"C:\Users\pcset\Documents\GitHub\SC2079\Algo\utils\algoReadValue.json",  # change this according to your local path
            "w",
            encoding="utf-8",
        ) as outfile:
            json.dump(transformedObject, outfile, indent=2)

        print("Data has been transformed and saved to example2.json.")

    except FileNotFoundError as e:
        print(f"Error reading example1.json: {e}")

    except json.JSONDecodeError as e:
        print(f"Error parsing JSON from example1.json: {e}")

    except Exception as e:
        print(f"An error occurred: {e}")


test()


def ReadWriteConvert():
    test()
    file = r"C:\Users\pcset\Documents\GitHub\SC2079\Algo\utils\algoReadValue.json"  # change this according to your local path
    maze = []
    for i in range(22):
        inner = []
        for j in range(22):
            if i == 0 or i == 21:
                inner.append(1)
                continue
            if j == 0 or j == 21:
                inner.append(1)
                continue
            inner.append(0)
        maze.append(inner)
    obstacles = []
    with open(file) as json_file:
        data = json.load(json_file)
        obs = data[
            "obstacles"
        ]  # Assuming the JSON contains a key "obstacles" with a list of obstacles
        for ob in obs:
            obj = ob.split(",")
            obj[0] = int(obj[0])
            obj[1] = int(obj[1])
            obj[2] = obj[2].strip()
            obj[3] = int(obj[3])
            obstacles.append(obj)

    GOALLIST = []
    GOALLIST.append([2, 2, "E", 0])

    ObstacleList = []
    # Convert the list of obstacles to fit the tree
    for i in range(len(obstacles)):
        # print("obstacles=", obstacles)
        obstacles[i][0] = int(obstacles[i][0]) + 1
        obstacles[i][1] = int(obstacles[i][1]) + 1

        if obstacles[i][2] == "E":
            obstacles[i][2] = "S"

        elif obstacles[i][2] == "N":
            obstacles[i][2] = "E"

        elif obstacles[i][2] == "S":
            obstacles[i][2] = "W"

        elif obstacles[i][2] == "W":
            obstacles[i][2] = "N"

    # print("obstacles 2 =", obstacles)

    goalincrement = 3  # need to change

    for i in range(len(obstacles)):
        Direction = obstacles[i][2]
        Xcoords = obstacles[i][0]
        Ycoords = obstacles[i][1]
        obstacleid = obstacles[i][3]

        maze[Xcoords][Ycoords] = 1
        maze[Xcoords - 1][Ycoords + 1] = 0.7  # topleft
        maze[Xcoords][Ycoords + 1] = 0.7  # top
        maze[Xcoords + 1][Ycoords + 1] = 0.7  # top right
        maze[Xcoords + 1][Ycoords] = 0.7  # right
        maze[Xcoords + 1][Ycoords - 1] = 0.7  # bottom right
        maze[Xcoords][Ycoords - 1] = 0.7  # bottom
        maze[Xcoords - 1][Ycoords - 1] = 0.7  # bottomleft
        maze[Xcoords - 1][Ycoords] = 0.7  # left

        if Direction == "N":
            maze[Xcoords - goalincrement][Ycoords] = 0.5
            GOALLIST.append([Xcoords - goalincrement, Ycoords, "S", obstacleid])
            ObstacleList.append((Xcoords - 1, Ycoords - 1, "N"))

        elif Direction == "S":
            maze[Xcoords + goalincrement][Ycoords] = 0.5

            GOALLIST.append([Xcoords + goalincrement, Ycoords, "N", obstacleid])
            ObstacleList.append((Xcoords - 1, Ycoords - 1, "S"))

        elif Direction == "E":
            maze[Xcoords][Ycoords + goalincrement] = 0.5
            GOALLIST.append([Xcoords, Ycoords + goalincrement, "W", obstacleid])
            ObstacleList.append((Xcoords - 1, Ycoords - 1, "E"))

        elif Direction == "W":
            maze[Xcoords][Ycoords - goalincrement] = 0.5
            GOALLIST.append([Xcoords, Ycoords - goalincrement, "E", obstacleid])
            ObstacleList.append((Xcoords - 1, Ycoords - 1, "W"))

        else:
            ObstacleList.append([Xcoords - 1, Ycoords - 1, "NIL", obstacleid])

    # print("Obstaclelist=", ObstacleList)
    # print("Goallist=", GOALLIST)

    for i in range(22):
        maze[0][i] = 1
        maze[21][i] = 1
        maze[i][0] = 1
        maze[i][21] = 1

    maze = np.array(maze)
    print("GOALLIST 1=", GOALLIST)
    return maze, ObstacleList, GOALLIST


ReadWriteConvert()


def write_json(data, filename):
    with open(filename, "w") as f:
        json.dump(data, f, indent=4)


def fix_Commands(commands):
    print("Commands before fixing:\n", commands)
    cmds = []
    for i in commands:
        if i == "Camera":
            cmds.append("RPI|TOCAM")
        elif "AND|" in i:
            cmds.append(i)
        else:
            cmds.append("STM|" + i)

    # cmds.append("RPI_END|0")  # add stop word
    return cmds


# if __name__ == "__main__":
# json_filename = (
#     r"C:\Users\pcset\Documents\NTU\Y3S1\SC2079 - MDP\algorithm\algoresult.json"
# )
# obstacles = read_json(json_filename)
# maze = create_maze()
# maze, obstacle_list, goal_list = convert_obstacles(obstacles)
# convert_obstacles()
