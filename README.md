# Unsupervised Active Visual Search with Monte Carlo Planning Under Uncertain Detections

> [!IMPORTANT]
> Consider citing our paper:
> ```BibTeX
>@ARTICLE{taioli_24_UAVS,
>        author={Taioli, Francesco and Giuliari, Francesco and Wang, Yiming and Berra, Riccardo and Castellini, Alberto and Bue, Alessio Del and Farinelli, Alessandro and Cristani, Marco and Setti, Francesco},
>        journal={IEEE Transactions on Pattern Analysis and Machine Intelligence}, 
>        title={{Unsupervised Active Visual Search with Monte Carlo Planning under Uncertain Detections}}, 
>        year={2024},
>        volume={46},
>        number={12},
>        pages={11047-11058},
>        doi={10.1109/TPAMI.2024.3451994}
>   }
>   ```

## Project Structure
```
|-- Dockerfile
|-- Makefile
|-- README.md
|-- target
|   |-- pomp_be_pd # executable for the policy computation
|-- input_pomp_be_pd 
|   |-- ... folder containing files needed for the policy
|-- output.txt
|-- src  
|   |-- include
|   |   |-- ... header files
|   |-- *.cpp files
```

## Setup
This project was develop using ubuntu 18.04 (boost library version ```1.65.1```). Setup the project by running the following commands:

1. Setup docker image 
```bash 
    docker build -t pomp_be_pd .
    docker run -it --rm -v "$(pwd):/workspace" --security-opt seccomp=unconfined --name pomp_be_pd_container -w /workspace pomp_be_pd /bin/bash
```

2. (Optional) Inside the docker image, test the boost library version
```bash
    g++ -o boost_version test_boost_library.cpp -L/usr/lib/x86_64-linux-gnu
    ./boost_version # should display 'Boost version: 1.65.1'
```
3. Compile the project
```bash
    cmake --target clean -G "Unix Makefiles" . # generate Makefile
    make -j 8
```

4. Run the policy solver
```bash
    ./target/pomp_be_pd  --problem "activevisualsearch" --runs=1 --maxdoubles=10 --mindoubles=10 --use_gt=1 --penality=1 --accuracy=120 --target_id=3 --startpos=10
```

## Explanation
The **original (without pre-processing)** dataset can be found [here.](https://www.cs.unc.edu/~ammirato/active_vision_dataset_website/visualize_data.html). For speed purposes, we applied some pre-processing steps to the dataset, thus not every files describe above is used during the simulation phase.

The folder ```pomp_be_pd``` contains several files:
- ```ListView.txt```: each rows represent an image of the AVDB datasets.
- ```MatrixG.txt```: each rows index is associated to the same row in ```ListView.txt```.  Each column value has a purpose: 
    - Col 0: row index if we execute action: `rotate_ccw`
    - Col 1: row index if we execute action: `rotate_cw`
    - Col 2: row index if we execute action: `forward`
    - Col 3: row index if we execute action: `backward`
    - ~~Col 4: row index if we execute action: deprecated~~
    - ~~Col 5: row index if we execute action: deprecated~~
- ```MatrixL_dt```: each rows index is associated to the same row in ```ListView.txt```. Each column ```i``` can be 0/1, if object ```i``` is detected or not by the object detector.
- ```MatrixL_gt```: same as ```MatrixL_dt```, but contains ground truth annotatations.
- ```ObservMatrix.txt```: take a look at the image [*Visualize camera positions and directions*](https://www.cs.unc.edu/~ammirato/active_vision_dataset_website/visualize_data.html)  for an easier explanation.
In AVDB, a house (scene) is represented by a grid structure in a 2D plane (cell can thus be *inside* or *outside* the house).
Every red-dot represent the agent pose inside the house, and every blue lines represent different view-point of the agent.
The aim of  ```ObservMatrix.txt``` is the following: the scene, compose of a grid structure is represented by a matrix, which can contains values ```1``` or ```0```, respectively if that particual cell is in the FOV of the agent or not from  a particular pose.
As usual, each row index of ```ObservMatrix.txt``` is associated to ```ListView.txt```. Finally, this **Observability matrix** is then flattend to a single vector.
- ```particles_that_are_inside_home.txt```: the scene is represented by a 2D grid structure. Aim of this file is to track if a given cell is inside (```1```) or outside (```0```) the home.
- ```GaussObsvMatrix.txt```: the idea is similar to ```ObservMatrix.txt```. Here, we save for each pose of the agent, the  probabilities to have the target object in location `j` given the agent pose.
- `detector_stat.json`: contains the statistics of the object detector (precision and recall for each object)


## Acknowledgements
Code based on [Monte-Carlo Planning in Large POMDPs - NeurIPS](https://papers.nips.cc/paper_files/paper/2010/hash/edfbe1afcf9246bb0d40eb4d8027d90f-Abstract.html)