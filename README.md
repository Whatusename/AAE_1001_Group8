# AAE_1001_Group8
<!-- Background of Path Planning to Aviation Engineering -->
## Theory of Path Planning Algorithm
The Astar algorithm is an algorithm for finding a shortest traversal path from a start point to a goal among paths with multiple nodes in the graph plane. It belongs to heuristic search algorithms as it uses heuristics to compute the nodes in the graph, thus reducing the number of nodes actually computed and improving the search efficiency.

**Steps in the Astar algorithm:**
1. **Initialization:** Add the start node to the open list and set the target node as the goal.
2. **Loop:** When the open list is not empty, perform the following steps: First take the node with the smallest f(n) value from the open list. If this node is the target node, the algorithm completes and returns the path. If not move this node to the closed list and check its neighbor nodes.  
For each neighbor node, calculate its f(n) value: If the neighbor node is not in the closure list, calculate its cost from the origin to the neighbor node f(n) and the heuristically estimated cost from the neighbor node to the target node h(n), and then calculate the f(n) value. If this f(n) value is lower than the current recorded value of the neighboring node, or the neighboring node is not in the open list, update its parent node and f(n) and g(n) values and add the neighboring node to the open list.
3. **End:** If the open list is empty and the target node is not found, it means there is no reachable path.
--------------------
## Task 1
### Map setting
The A-star path planning is used to find the shortest path to the destination. But before the implementation, the obstacles, sensitive areas and the boundary need to be set.
#### 1.principle
In this project, there are nodes that during the searching procedure, the computer can’t put into open/close list. By adding these nodes in a sequence, we can add obstacles. The same methods can be used in boundary setting and sensitive area setting. 
#### 2. Method
To interpret the nodes into code, the coordinate system is used. The numerous nodes of a line can be translated into a equation using y=ax+b form. For instance, the obstacle in fig.1 can be transformed into y=-2x+10, then we use a loop to generate x(i) range from 0 to 9, which represents the Ox of the nodes. After that, the Oy can be calculated by the fomular, and the nodes can be added (Ox, Oy).  
  `for i in range(0, 10):  
        ox.append(i)  
        oy.append(-2 * i + 10)`  
For sensetive area, the loop needs to be apply to both Ox and Oy.(As the Fig.2 shows)
![tu1](https://github.com/Whatusename/AAE_1001_Group8/blob/main/%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE%202024-12-01%20220224.png)
>fig.1  
![tu2](https://github.com/Whatusename/AAE_1001_Group8/blob/pictures/%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE%202024-12-01%20112901.png)  
>fig.2
### Cost calculation
How to find the total cost of the aircraft and the most suitable one. Use the formula based on the given one and design the calculation subroutine, so we define the parameters: FC (fuel cost); T (time); FCR (fuel consumption rate); CT (time related cost); CC (fixed cost) and build up the formula C= (FC*T*FCR*T) + (T*CT) +CC. Then we input the data provided so that we can calculate the total cost.

Then we need to judge which aircraft is most suitable in different scenarios, so we first need to analyze whether it is suitable for this situation, we have design three modules for different scenarios, firstly get the number of flights: number of passenger/ aircraft capacity, to find out if it satisfies the requirements. If so, we carry on to the next stage. We have an index of 0 to 3 for each type of aircraft so that in cycle operation the code runs in all possible ways and results.  For different time cost we also have methods to identify, we give them unique index independent from aircraft type number.

After getting the results, we output the float type of calculated number and use min function to make comparison between them, output the most suitable answer. 

------------------------
## Task 2
### Introduction
This project involves the creation of a new cost area, termed "Jet Stream," aimed at reducing the operating costs of flight routes by optimizing the flight path to take advantage of prevailing jet streams. This approach proposes a 5% reduction in route costs by altering the flight path dynamically.
### Task Overview
Establish a new cost area to reduce route costs.
### Step-by-step Strategy: 
1. Creating the Jet Stream Cost Area: Define a specific area on the flight map where jet streams can be utilized to reduce fuel consumption and time.  
![1](https://github.com/Whatusename/AAE_1001_Group8/blob/pictures/%E5%9B%BE%E7%89%871.jpg)
2.	Cost Calculation Modification: Integrate this area into the route planning algorithm to adjust cost calculations, specifically reducing costs by 5% within this area.
3.	Visualization and Testing: Implement visual simulations and tests to validate the cost savings.  
![2](https://github.com/Whatusename/AAE_1001_Group8/blob/pictures/%E5%9B%BE%E7%89%872.jpg)
### Technical Details
Define the Jet Stream Area: The jet stream area is defined laterally across the map with a vertical span of 5 units.
Cost Adjustment Code:  
`if (self.jet_stream_x_start <= self.calc_grid_position(node.x, self.min_x) <= self.jet_stream_x_end and`  
`                   self.jet_stream_y[0] <= self.calc_grid_position(node.y, self.min_y) <= self.jet_stream_y[1]):`  
`                   node.cost *= 0.95`
### Scenario Analysis:
**Task 1** (without Jet Stream): Total cost and time were higher.  
**Task 2** (with Jet Stream): Reduction in both cost and time, demonstrating a 4.76% overall reduction in operational costs when the jet stream is applied.  
![3](https://github.com/Whatusename/AAE_1001_Group8/blob/pictures/%E5%9B%BE%E7%89%874.jpg)  
### Conclusion
The implementation of the jet stream area within the flight route planning algorithm successfully demonstrated cost and time savings, making it a viable option for reducing operational expenses in air travel. This proof-of-concept can be further expanded to include real-time data and more complex scenarios.

------------------------

## Task 3
To get our answer, we tried to separate the whole scenario into 2 situations. One is that our aircraft has 2 engines, and another is that it is equipped with 4 engines. If our aircraft is a twin-engine aircraft, the trip fuel should be 40 kilograms per minute while the fixed cost Cc should be 2000, if it is a 4-engine one, its trip fuel should be 80 kilograms per minute and Cc should be 2500.

Since the number of the flights should be no more than 12 and the passenger capacity should be no more than 450, the number of flights should be 7 to 12. If the number of flights is 7 to 10, the aircraft should have 4 engines, in this case the passenger capacity should be 300 to 450, if the number is 11 or 12, it should have 2, and the passenger capacity should be 250 to 299.

Then we tried to get our python code. During this step we translated our original formula into the form “A=(B*C*I*D+E*D+F) *G” to make sure that the computer can completely understand our meaning. In this step, we also used artificial intelligence properly and it helped a lot when we were getting the right code. After some calculation, we finally got the result. Our flight’s passenger capacity should be at 273 to 299, that means the flight number should be 11. Considering that we should try to get the most profit, the passenger capacity should be as much as possible. So, the final passenger capacity is 299, and the total cost is **56254**.

------------------------------

## Additional Task 1
### Goals
- Add one checkpoint for each cost intensive area with visualization
- Reach all checkpoints before arriving at the destination and plan new path
- Calculate the total time and total cost in different scenarios
### Steps of the code
1.	Start from the base of task 1 with defined map, first defines two checkpoints in a 2D space, each located in a different "cost intensive area". The checkpoints are represented by their x and y coordinates: Checkpoint 1: (20.0, 30.0), Checkpoint 2: (40.0, 5.0). Setting the color to green, and the mark size to 10. 
2.	Plan a path through checkpoints using the A* algorithm, the code breaks down the path into three segments: Start to Checkpoint 1, Checkpoint 1 to Checkpoint 2, Checkpoint 2 to Goa land combines the x and y coordinates of each segment to form the complete path, and calculates the total time taken. The [1:] indexing is used to remove the duplicate points at the checkpoint. Plotting a path using matplotlib and displays the plot.
3.	Calculate and compare the total costs of three aircraft models ("A321neo", "A330-900neo", "A350-900") under three different scenarios. For each scenario, calculate the total cost for each aircraft model by calling the “calculate_aircraft_cost” function with different parameters. If the calculated flight time exceeds a certain threshold, the aircraft is considered "not suitable" and its total cost is set to infinity. The code then finds and prints the aircraft model with the minimum total cost for each scenario. The code also handles any potential Value Error exceptions that may occur during the cost calculation and prints an error message if such an exception occurs.

-----------------------

## Additional Task 2 
### Overview
The code is an A* path planning, with a random generated map, including starting point, goal point, a 40*40 size price intensive area, and several obstacles. The code will finally draw a map with a path.  
![5](https://github.com/Whatusename/AAE_1001_Group8/blob/pictures/%E5%9B%BE%E7%89%875.jpg)
### Code
1. A* code based on task 1

    
3. Import random to generate random map and set number for the number of obstacle and map size to adjust the density of obstacles.
![6](https://github.com/Whatusename/AAE_1001_Group8/blob/pictures/%E5%9B%BE%E7%89%877.png)![7](https://github.com/Whatusename/AAE_1001_Group8/blob/pictures/%E5%9B%BE%E7%89%878.jpg)


5. Set occupied zone to ensure no obstacles near the start a
![8](https://github.com/Whatusename/AAE_1001_Group8/blob/pictures/%E5%9B%BE%E7%89%8710.jpg)

4. Find the optional paths and draw the path that costs the least money on the map.

5. Calculate the total cost and time consumed during the operation.
### Input
- Number of obstacles
- Shortest distance between start and goal position
- Cost of fuel
- Size of price intensive area
 
### Output
- Final path
- Cost

### Dependencies
- Python 3.x
- Libraries: math, random, matplotlib

-----------------------

## Reflection
>### **Liu Boyu 24100965d**
Through this project, I have learnt how to cooperate with my partners and do our best to deal with different challenges. Although we didn’t know each other before the project, we all have to spare no effort to make sure that the project would run smoothly. The project also taught me how to use artificial intelligence in a proper way, for example, we used AI to help us with our codes after we had our methods, and it contributed a lot. Moreover, this also reminded me that AI can be widely used in numerous fields. And it is of great importance to use it effectively in our study.

-------------------
>### **Zhou Lianyi 24114931d**
In the context of the AAE1001 study, I gained significant insights and expertise. I was able to engage in productive discussions and explorations with the same group of individuals in a satisfactory manner. Furthermore, I made notable advancements in my proficiency with the Python language. This not only enhanced my familiarity with its operational aspects but also facilitated more efficient and effective problem-solving. The aircraft is a fundamental component of the model, serving as a basis for further expansion. Given my proficiency in citing sources, I would rate my ability to cite one to three sources as moderate. In addressing additional tasks within the context of the basic task, I sought to identify potential avenues for optimization for each component. The additional task component of the basic task was expanded to a certain degree, which enabled me to identify potential optimization opportunities for each component. It was also crucial for me to be able to present the tasks I had practiced and completed in a more logical and structured manner. This approach helped me to simplify and rationalize complex problems, enhance my communication skills, and improve my ability to present information to a general.

------------------------
>### **Sun Yinlan 24116186d**
In this project, I’m lucky to be able to lead my group. I try my best to allocate the work and organize our project.

During the leadership experience, I learned how to communicate with different people, to understand what they need and connect them together to finish the project. I found that everyone in my group is willing to do extra work to release other’s stress. Overall, I think it’s a wonderful experience to lead such a team.

About the tasks, I am responsible for the map setting part of task1 and readme uploading. During the experience of solving task 1, I learned how to read the code, how to translate something that we may think is too easy to be explained into the language the computer can understand. It’s a bit difficult to achieve such a goal for me, even if it seems easy (just drawing some lines and rectangles). But after all, the journey towards the final answer is really enjoyable, during which I can find I am really learning something cool.

-------------------------
>### **Tang Lingjie 24116964d**
In the group project I tried to solve problems with code. Although it was not easy, everything started from scratch. As I had never dealt with code before, at first, I just wanted to stay away from it, but later thought that the purpose of the course was to let me understand and learn computer languages, so then I proactively took on additional task 1. First, I started with the principles of the A* algorithm, then I tried to understand the format. Finally, I began to input. The principle was simple, but the process was tortuous, sometimes the program would not run and sometimes the terminal would not give the desired results. In this process, AI is an indispensable assistant. I don't remember how many times I've used the phrase "please explain". It has patiently explained my questions, pointed out existing errors and suggested ways of optimizing. I am also very grateful for the help of my group members and the group leader. AI is sometimes limited, and only humans can solve them when comes to more complex problems. During my efforts to tame the code, they provided necessary assistance. In summary, it was a completely new experience. Although there were many areas that still need to be improved, this project really opened the door to programming, and I believe I will have a deeper connection with computer languages in the future.

----------------------------
>### **Yang Chenyu 24128676d**
During the group project AAE1001, I had learned how to skillfully use the AI to help modify and write code for a given tasks. Moreover, I get known that AI can help me do some thing even never touch before. Additionally, let AI explain errors in the code, learned some basic rule for the code writing. Moreover, the presentation help me become confidence and do not feel nervous face so many people.

-----------------------
>### **Wang Zhiru 24117786d**
I gained a lot from this group project, even though I wasn't directly involved in coding, I gained experience in teamwork and learned a lot about programming. Initially, my understanding of the A* algorithm was limited to its theoretical foundations. However, with the help of several students who have a foundation in programming, I have gained a deeper understanding of how it works in practice. Throughout the project I was responsible for understanding the principles of the algorithm and part of the group debriefing, during which I learned some basic programming skills, such as my group leader taught me how to input the obstacles from our group's map using functions. The completion of this project required effective communication and collaboration. I felt that our group had a great collaborative atmosphere where I worked closely with my teammates and where the group leader valued each member's input. Even if I was not involved in the programming, the other students would carefully explain to me how their tasks were accomplished. Throughout the course of the project, we encountered challenges such as making sure our project stayed on schedule and met program requirements. I learned how to address these challenges by engaging in problem-solving discussions and providing ideas for overcoming obstacles. All in all, I think this group project is a valuable learning experience, which enhances my ability to work in a team, communicate effectively and presentation skills, and moreover, I have learned a lot of professional knowledge.

------------------
>### **Chan For Mand 24122821d**
The combination of the A* algorithm and cost-area optimization successfully shows the practical application. This project not just enhanced coding abilities, yet it also showed the skills and knowleged in complicated scenario planning in Python.

-----------------
>### **Ou Yeyang 24132829d**
Through this group project, I have deepened my understanding of the code and know how to use different arguments and statements to achieve my goals. For example, I can use a bunch of parameters and calculations to define a formula, use functions to determine whether the model is suitable or not, and so on. Constantly applying these in the code deepened my proficiency. In addition, I learned to use AI technology to help me correct grammar errors and optimize my code, and I can send my code to the AI, ask for and get suggestions for changes as a way to make the program better. Since this project is based on the A* algorithm, I have a deeper understanding of this type of pathfinding algorithm and know its advantages over other algorithms.
