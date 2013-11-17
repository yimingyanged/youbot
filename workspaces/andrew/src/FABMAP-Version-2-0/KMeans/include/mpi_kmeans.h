/*
This exact-kmeans option is based on the MPIKmeans 1.1 library
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
*/

#ifndef __MPI_KMEANS_MEX_H__
#define __MPI_KMEANS_MEX_H__

/* use the input type variable to generate code for integer input or double */
#define INPUT_TYPE 0

#ifdef INPUT_TYPE
#if INPUT_TYPE==0
#define PREC float
#endif
#if INPUT_TYPE==1
#define PREC unsigned int
#endif 
#if INPUT_TYPE==2
#define PREC float
#endif 
#endif

#ifndef BOUND_PREC
#define BOUND_PREC float
//#define BOUND_PREC double
#endif


#ifndef PREC_MAX
    #define PREC_MAX FLT_MAX
    //#define PREC_MAX DBL_MAX
#endif

#ifndef BOUND_PREC_MAX
#define BOUND_PREC_MAX FLT_MAX
//#define BOUND_PREC_MAX DBL_MAX
#endif

#define BOUND_EPS 1e-6

// comment for more speed but no sse output
//#define KMEANS_DEBUG

// if you do not want to have verbose messages printed
#define KMEANS_VERBOSE

#ifdef KMEANS_DEBUG
unsigned int saved_two=0,saved_three_one=0,saved_three_two=0,saved_three_three=0,saved_three_b=0;
#endif

extern "C"{
PREC kmeans(PREC *CXp,const PREC *X,unsigned int *c,unsigned int dim,unsigned int npts,unsigned int nclus,unsigned int maxiter, unsigned int nr_restarts, bool randomInitialCentres = true);
}
PREC compute_distance(const PREC *vec1, const PREC *vec2, const unsigned int dim);
unsigned int assign_point_to_cluster_ordinary(const PREC *px, const PREC *CX, unsigned int dim,unsigned int nclus);
void randperm(unsigned int *order, unsigned int npoints);

#endif


