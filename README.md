# Hybrid Trajectory Optimization for a Monkey Bar Robot Using DIRCOL

Implementation of a Hybrid Direct Collocation (DIRCOL) based Trajectory Optimization Method for a Brachiating Robot
[[paper]](https://github.com/ologandavid/MonkeyBarBot/blob/main/Documents/FinalReport.pdf)

**This repo supports:**
* DIRCOL Trajecotry Solving for a Two Link Robot using iPOPT Solver
* visualization using Julia Plot
* visualization using MeshCat Libraries

## Introduction
In this repository, we propose a trajectory planning technique that mimics a Brachiation robot swinging from bar to bar. Using a hybrid system direct collocation (DIRCOL) trajectory optimization, we successfully demonstrate the robot swinging up from a dead hang to catch the first bar and swing to the subsequent bars. This DIRCOL technique was tested on various mass distributions in the robot as well as different bar separation distances to understand the behavior with varying parameters. In addition, we show the importance of a free time setup on the cost function in producing consistent feasible trajectories using this DIRCOL technique.

## Dependencies
Julia v1.6.7, Juptyer Notebook

## Installation
1. Install Julia. Download v1.6.7 from the Julia website. From here you can follow these platform specific instructions to get started.
2. Clone this repo and put it wherever you want.
3. Start a Julia REPL in your terminal using julia, or the location to the Julia binaries. If this doesn't work, make sure you followed the platform specific instructions.
4. Install the IJulia using the Julia package manager. In the REPL, enter the package manager using ], then enter add IJulia to add it to your system path.
5. In the REPL (hit backspace to exit these package manager), enter using IJulia
6. Launch the notebook using notebook() or jupyterlab()

## Trajectory Generation
The majority of the necessary code is located

## Performance
![](https://github.com/ologandavid/MonkeyBarBot/blob/main/MonkeyBarBot/tmp.gif)
Visualized Solved Trajectory using Julia Plot


