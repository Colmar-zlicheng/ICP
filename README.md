# ICP C++ implementation
Final project for ME3602-1 @ SJTU

Include:
- ICP C++ implementation: ICP.cpp
- ICP C++ parallel compute implementation with MPI: ICP_MPI.cpp

### ICP derivation
```
./ICP_deduce_SVD.pdf
./ICP_deduce_SVD.tex
```

### data
```
data
├── points_after.pcd
└── points_before.pcd
```

### run ICP
```shell
g++ -o icp.o ICP.cpp
```

### run ICP with MPI

#### install MPI
```shell    
sudo apt-get install mpich
```
#### run
```shell
mpicxx -g -Wall -o icp_mpi.o ICP_MPI.cpp
mpirun -n 10 ./icp_mpi.o
```
you may need to add '--allow-run-as-root' to run mpi:
```shell
mpirun --allow-run-as-root -n 10 ./icp_mpi.o
```