We are going to implement **RRT** and **RRT*** algorithms. You will be able to run these algorithms to find a path in a map, and visualize the result.

Files included:

**RRT.py** is the file where RRT and RRT* is implemented.

**main.py** is the script that provides helper functions that load the map from an image and call the classes and functions from **RRT.py**. 

**WPI_map.jpg** is a binary WPI map image with school buildings. You could replace it with some other maps you prefer.

## Instruction

Before starting any coding, please run the code first:

`python main.py`

The **main.py** loads the map image **WPI_map.jpg** and calls classes and functions to run planning tasks.

Please keep in mind that, the coordinate system used here is **[row, col]**, which is different from [x, y] in Cartesian coordinates. In README and the code comment, when the word '**point**' is used, it refers to a simple list [row, col]. When the word '**node**' or '**vertex**' is used, it refers to the Node class in RRT. 