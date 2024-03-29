# Advanced Shortest Paths Project

## 1) Bidirectional Dijkstra

Social networks are live on the connections between people, so friend
suggestions is one of the most important features of Facebook. One of
the most important inputs of the algorithm for friend suggestion is most
probably the current distance between you and the suggested person
in the graph of friends connections. Your task is to implement efficient
computation of this distance. The grader will test your algorithm against
different real-world networks, such as a part of internet, a network of
scientific citations or coauthorship, a social network of jazz musicians or
even a social network of dolphins.

<h2>Problem Description</h2>
<h3>Task</h3> Compute the distance between several pairs of nodes in the network.
<h3>Input Format</h3> The first line contains two integers 𝑛 and 𝑚 — the number of nodes and edges in the
network, respectively. The nodes are numbered from 1 to 𝑛. Each of the following 𝑚 lines contains
three integers 𝑢, 𝑣 and 𝑙 describing a directed edge (𝑢, 𝑣) of length 𝑙 from the node number 𝑢to the
node number 𝑣. (Note that some social networks are represented by directed graphs while some other
correspond naturally to undirected graphs. For example, Twitter is a directed graph (with a directed
edge (𝑢, 𝑣) meaning that 𝑢 follows 𝑣), while Facebook is an undirected graph (where an undirected
edge {𝑢, 𝑣} means that 𝑢 and 𝑣 are friends). In this problem, we work with directed graphs only for a
simple reason. It is easy to turn an undirected graph into a directed one: just replace each undirected
edge {𝑢, 𝑣} with a pair of directed edges (𝑢, 𝑣) and (𝑣, 𝑢).)
The next line contains an integer 𝑞 — the number of queries for computing the distance. Each of the
following 𝑞 lines contains two integers 𝑢 and 𝑣 — the numbers of the two nodes to compute the distance
from 𝑢 to 𝑣.
<h3>Constraints</h3> 1 ≤ 𝑛 ≤ 1 000 000; 1 ≤ 𝑚 ≤ 6 000 000; 1 ≤ 𝑢, 𝑣 ≤ 𝑛; 1 ≤ 𝑙 ≤ 1 000; 1 ≤ 𝑞 ≤ 1 000.
<h3>Output Format</h3> For each query, output one integer on a separate line. If there is no path from 𝑢 to 𝑣,
output −1. Otherwise, output the distance from 𝑢 to 𝑣.

<h3>Verdict</h3>
Good job! (Max time used: 6.06/25.00, max memory used: 745611264/2147483648.)

## 2) A* Algorithm

In this task you will be given a description of a real-world road network
with not just edges and their lengths, but also with the coordinates of the
nodes. Your task is still to find the distance between some pairs of nodes,
but you will need to use the additional information about coordinates to
speedup your search.

## Problem Description 

### Task.
Compute the distance between several pairs of nodes in the network.
### Input Format.
The first line contains two integers 𝑛 and 𝑚 — the number of nodes and edges in the
network, respectively. The nodes are numbered from 1 to 𝑛. Each of the following 𝑛 lines contains the
coordinates 𝑥 and 𝑦 of the corresponding node. Each of the following 𝑚 lines contains three integers
𝑢, 𝑣 and 𝑙 describing a directed edge (𝑢, 𝑣) of length 𝑙 from the node number 𝑢 to the node number 𝑣.
It is guaranteed that 𝑙 ≥
√︀(𝑥(𝑢) − 𝑥(𝑣))2 + (𝑦(𝑢) − 𝑦(𝑣))2 where (𝑥(𝑢), 𝑦(𝑢)) are the coordinates of 𝑢
and (𝑥(𝑣), 𝑦(𝑣)) are the coordinates of 𝑣. The next line contains an integer 𝑞 — the number of queries
for computing the distance. Each of the following 𝑞 lines contains two integers 𝑢 and 𝑣 — the numbers
of the two nodes to compute the distance from 𝑢 to 𝑣.

### Constraints. 
1 ≤ 𝑛 ≤ 110 000; 1 ≤ 𝑚 ≤ 250 000; −109 ≤ 𝑥, 𝑦 ≤ 109; 1 ≤ 𝑢, 𝑣 ≤ 𝑛; 0 ≤ 𝑙 ≤ 100 000;
1 ≤ 𝑞 ≤ 10 000. For Python2, Python3, Ruby and Javascript, 1 ≤ 𝑛 ≤ 11 000, 1 ≤ 𝑚 ≤ 30 000.

### Output Format.
For each query, output one integer. If there is no path from 𝑢 to 𝑣, output −1. Otherwise,
output the distance from 𝑢 to 𝑣.

### Verdict
Good job! (Max time used: 21.87/50.00, max memory used: 41476096/2147483648.)
