# Multi-Agent-Control
The purpose of this repository is to implement several Multi-Agent Controllers, with the primary goal of maintaining a framework for future use.

## Simple Linear Consensus Protocal
This is a very simple linear consensus controller. We can simplify it using graph laplacian as following:
\[
\begin{align*}
\dot{x}_i & = \sum_{j \in \mathcal{N}_i} (x_j - x_i) \\
\dot{x}_i & = -[...-a_{ij}-a_{ij}...]x-\|\mathcal{N}_i\|x_i \\
\dot{x}_i & = -Lx
\end{align*}
\]
This simple controller provides a user interface. One can use sliding bar to generate up to 10 agents, and select certain communication network topology accordingly.

![overview](figure/LCP_GUI.png)