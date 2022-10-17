## Localization

- One of the most popular path finding algorithm, that is, “pure pursuit” that has been highly used in autonomous navigation because of its efficient computational capability and its simplicity
- Output a smooth path that is workable for a differential drive robot
- PID controller is simple to implement and provides robust navigation
- Disadvantage of RTK GPS is susceptible to uncertain localization due to blockage, signal weakness, and multipath bugs and thus it needs redundancy and continuous safeguard checking.
- Range sensors and cameras with sensing capability are greatly used in intra‐crop row navigation.
We'd be using the last one.

## Image processing technique for pest detection
- First‐ and second‐order derivatives are used for identifying green parts (crop image) which produce a thicker boundary of the cropped image.
- Region‐based segmenentation is applied on gray scale images for boundary identification.
- Second‐order derivatives give a double‐edged response at intensity step and step changes in color intensity 
Q what does derivative of an image would give? 
## Trajectory
- Travelling Salesman algorithm to enhance the trajectory planning algorithm to trace the efficient movement of flexible link for covering all pests on the pest map.dynamic programming approach.
- To search the shortest path, apply the graph theory and express a pest map as a set of nodes and branches. Each node (pest) has X and Y coordinates on the pest map and they are connected expressed as branches. Minimize the total cost for link movement to reach all pest on a single plant.

#Navigation Stack

![](https://github.com/akgcode/farmbot/blob/main/assets/Navigation_stack.png)
