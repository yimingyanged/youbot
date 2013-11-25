To use the external memory containers, you need to install STXXL, 
and select the external memory option in the pKMeans CMake settings.
Also, make sure you have a .stxxl configuration file in this directory.

The exact-kmeans option is based on the MPIKmeans 1.1 library
The latest version is available at
http://mloss.org/software/view/48/
That version also provides Python and Mex interfaces.

The relevant citation is:

@misc{ elkan03using,
  author = "C. Elkan",
  title = "Using the triangle inequality to accelerate kMeans",
  text = "C. Elkan. Using the triangle inequality to accelerate kMeans. In Proceedings
    of the Twentieth International Conference on Machine Learning, 2003, pp.
    147-153.",
  year = "2003",
  url = "citeseer.ist.psu.edu/elkan03using.html" 
}


A note on convergence:
Sum Of Squared Error should decrease monotonically with iterations.
With the approximate version it can sometimes be seen to increase.
This is because of randomness in the nearest neighbour finding.
