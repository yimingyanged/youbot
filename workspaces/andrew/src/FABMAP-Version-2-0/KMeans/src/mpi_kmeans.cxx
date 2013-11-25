#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include <memory.h>
#include <math.h>
#include <assert.h>
#include "mpi_kmeans.h"



int comp_randperm (const void * a, const void * b)
{
	return ((int)( *(PREC*)a - *(PREC*)b ));
}


void randperm(unsigned int *order, unsigned int npoints)
{
	PREC *r;
	unsigned int i;
	r = (PREC*)malloc(2*npoints*sizeof(PREC));
	for (i=0; i<2*npoints; i++,i++)
	{
		r[i] = rand();
		r[i+1] = i/2;
	}
	qsort (r, npoints, 2*sizeof(PREC), comp_randperm);

	for (i=1; i<2*npoints; i++,i++)
		order[i/2] = (unsigned int)r[i];

	free(r);
}

#if INPUT_TYPE>0
double compute_distance(const PREC *vec1, const PREC *vec2, const unsigned int dim)
{
	unsigned int k;
	double d,df;
	d = 0.0;
	for ( k=0 ; k<dim ; k++ )
	{
		df = ((double)vec1[k]-(double)vec2[k]);
		d += df*df;
	}
	d = sqrt(d);

	return d;
}
double compute_distance(const PREC *vec1, const double *vec2, const unsigned int dim)
{
	unsigned int k;
	double d,df;
	d = 0.0;
	for ( k=0 ; k<dim ; k++ )
	{
		df = ((double)vec1[k]-vec2[k]);
		d += df*df;
	}
	d = sqrt(d);

	return d;
}
#endif

PREC compute_distance(const PREC *vec1, const PREC *vec2, const unsigned int dim)
{
	unsigned int k;
	PREC d,df;
	d = 0.0;
	for ( k=0 ; k<dim ; k++ )
	{
		df = (vec1[k]-vec2[k]);
		d += df*df;
	}
	d = sqrt(d);

	return d;
}

PREC compute_sserror(const PREC *CX, const PREC *X, const unsigned int *c,unsigned int dim, unsigned int npts)
{
	unsigned int i;
	PREC sse, d;
	const PREC *pcx;
	const PREC *px;
	sse = 0.0;
	for ( i=0,px=X ; i<npts ; i++,px+=dim)
	{
		pcx = CX+c[i]*dim;
		d = (PREC)(compute_distance(px,pcx,dim));
		sse += d*d;
	}
	return(sse);
}

void compute_cluster_distances(BOUND_PREC *dist, BOUND_PREC *s, const PREC *CX, unsigned int dim,unsigned int nclus, const bool *cluster_changed)
{
	unsigned int j,i,cnt;
	const PREC *pcxp, *pcx;

	for ( j=0 ; j<nclus ; j++ )
		s[j] = BOUND_PREC_MAX;

	for ( i=0,pcx=CX ; i<nclus-1 ; i++,pcx+=dim)
	{
		pcxp = CX + (i+1)*dim;
		cnt=i*nclus+i+1;
		for ( j=i+1 ; j<nclus; j++,cnt++,pcxp+=dim )
		{
			if (cluster_changed[i] || cluster_changed[j])
			{
				dist[cnt] = (BOUND_PREC)(0.5 * compute_distance(pcx,pcxp,dim));
				dist[j*nclus+i] = dist[cnt];

				if (dist[cnt] < s[i])
					s[i] = dist[cnt];

				if (dist[cnt] < s[j])
					s[j] = dist[cnt];
			}
		}
	}
}


void remove_point_from_cluster(unsigned int cluster_ind, PREC *CX, const PREC *px, unsigned int *nr_points, unsigned int dim)
{
	PREC *pcx;

	pcx = CX + cluster_ind*dim;

	unsigned int k;
	/* empty cluster after or before removal */
	if (nr_points[cluster_ind]<2)
	{
		for ( k=0 ; k<dim ; k++ )
			pcx[k] = 0.0;
		nr_points[cluster_ind]=0;
	}
	else
	{
		PREC nr_old,nr_new; 
		nr_old = (PREC)nr_points[cluster_ind];
		(nr_points[cluster_ind])--;
		nr_new = (PREC)nr_points[cluster_ind];

		for ( k=0 ; k<dim ; k++ )
			pcx[k] = (nr_old*pcx[k] - (PREC)px[k])/nr_new;
	}
}

void add_point_to_cluster(unsigned int cluster_ind, PREC *CX, const PREC *px, unsigned int *nr_points, unsigned int dim)
{
	unsigned int k;
	PREC nr_old, nr_new;
	PREC *pcx;

	pcx = CX + cluster_ind*dim;

	/* first point in cluster */
	if (nr_points[cluster_ind]==0)
	{		
		(nr_points[cluster_ind])++;
		for ( k=0 ; k<dim ; k++ )
			pcx[k] = (PREC)px[k];
	}
	else
	{
		nr_old = (PREC)(nr_points[cluster_ind]);
		(nr_points[cluster_ind])++;
		nr_new = (PREC)(nr_points[cluster_ind]);
		for ( k=0 ; k<dim ; k++ )
			pcx[k] = (nr_old*pcx[k]+(PREC)px[k])/nr_new;
	}
}

unsigned int init_point_to_cluster(unsigned int point_ind, const PREC *px, const PREC *CX, unsigned int dim,unsigned int nclus, PREC *mindist, BOUND_PREC *low_b, const BOUND_PREC *cl_dist)
{
	PREC mind, d;
	unsigned int assignment, j,bias;
	const PREC *pcx;
	bool use_low_b = true;

	if (low_b==NULL) use_low_b = false;
	bias = point_ind*nclus;
	
	pcx = CX;
	mind = compute_distance(px,pcx,dim);
	if (use_low_b) low_b[bias] = (BOUND_PREC)mind;
	assignment = 0;
	pcx+=dim;
	for ( j=1 ; j<nclus ; j++,pcx+=dim )
	{
		if (mind + BOUND_EPS <= cl_dist[assignment*nclus+j])
			continue;

		d = compute_distance(px,pcx,dim);
		if(use_low_b) low_b[j+bias] = (BOUND_PREC)d;

		if (d<mind)
		{
			mind = d;
			assignment = j;
		}
	}
	mindist[point_ind] = mind;
	return(assignment);
}

unsigned int assign_point_to_cluster_ordinary(const PREC *px, const PREC *CX, unsigned int dim,unsigned int nclus)
{
	PREC mind, d;
	unsigned int assignment=0, j;
	const PREC *pcx;

	mind = PREC_MAX;
	for ( j=0,pcx=CX ; j<nclus ; j++,pcx+=dim )
	{
		d = compute_distance(px,pcx,dim);
		if (d<mind)
		{
			mind = d;
			assignment = j;
		}
	}
	return(assignment);
}

unsigned int assign_point_to_cluster(unsigned int point_ind, const PREC *px, const PREC *CX, unsigned int dim,unsigned int nclus, unsigned int old_assignment, PREC *mindist, BOUND_PREC *s, BOUND_PREC *cl_dist, BOUND_PREC *low_b)
{
	PREC mind, d;
	unsigned int assignment, j, counter;
	const PREC *pcx;
	bool up_to_date = false,use_low_b=true;;
	unsigned int bias;

	bias  = point_ind*nclus;
	if (low_b==NULL)use_low_b=false;

	mind = mindist[point_ind];

	if (mind+BOUND_EPS <= s[old_assignment])
	{
#ifdef KMEANS_DEBUG
		saved_two++;
#endif
		return(old_assignment);
	}

	assignment = old_assignment;
	counter = assignment*nclus;
	for ( j=0,pcx=CX ; j<nclus ; j++,pcx+=dim )
	{
		if (j==old_assignment)
		{
#ifdef KMEANS_DEBUG
			saved_three_one++;
#endif
			continue;
		}
		
		if (use_low_b && (mind+BOUND_EPS <= low_b[j+bias]))
		{
#ifdef KMEANS_DEBUG
			saved_three_two++;
#endif
			continue;
		}

		if (mind+BOUND_EPS <= cl_dist[counter+j])
		{
#ifdef KMEANS_DEBUG
			saved_three_three++;
#endif
			continue;
		}

		if (!up_to_date)
		{
			d = compute_distance(px,CX+assignment*dim,dim);
			mind = d;
			if(use_low_b) low_b[assignment+bias] = (BOUND_PREC)d;
			up_to_date = true;
		}

		if (!use_low_b)
			d = compute_distance(px,pcx,dim);
		else if ((mind > BOUND_EPS+low_b[j+bias]) || (mind > BOUND_EPS+cl_dist[counter+j]))
		{
			d =compute_distance(px,pcx,dim);
			low_b[j+bias] = (BOUND_PREC)d;
		}
		else
		{
#ifdef KMEANS_DEBUG
			saved_three_b++;
#endif
			continue;
		}

		if (d<mind)
		{
			mind = d;
			assignment = j;
			counter = assignment*nclus;
			up_to_date = true;
		}
	}
	mindist[point_ind] = mind;

	return(assignment);
}


PREC kmeans_run(PREC *CX,const PREC *X,unsigned int *c,unsigned int dim,unsigned int npts,unsigned int nclus,unsigned int maxiter)
{
	PREC *tCX;
	unsigned int *CN, *old_c, *rperm = NULL;
	PREC sse = 0.0;
	PREC *pcx, *tpcx, *mindist;
	bool *cluster_changed,use_low_b=true;
	const PREC *px;
	unsigned int i,j,k,nchanged,iteration,cnt;
	BOUND_PREC *s, *offset, *cl_dist, *low_b;

	tCX = (PREC *)calloc(npts * dim, sizeof(PREC));
	assert(tCX);

	/* number of points per cluster */
	CN = (unsigned int *) calloc(nclus, sizeof(unsigned int)); 
	assert(CN);
	
	/* old assignement of points to cluster */
	old_c = (unsigned int *) malloc(npts* sizeof(unsigned int));
	assert(old_c);

	/* assign to value which is out of range */
	for ( i=0 ; i<npts ; i++)
		old_c[i] = nclus;

#ifdef KMEANS_VERBOSE
	printf("compile without setting the KMEANS_VERBOSE flag for no output\n");
#endif

	low_b = (BOUND_PREC *) calloc(npts*nclus,sizeof(BOUND_PREC));
	if (low_b == NULL)
	{
#ifdef KMEANS_VERBOSE
		printf("not enough memory for lower bound, will compute without\n");
#endif
		use_low_b = false;
	}
	else
		assert(low_b);


	cl_dist = (BOUND_PREC *)calloc(nclus*nclus, sizeof(PREC));
	assert(cl_dist);

	s = (BOUND_PREC *) malloc(nclus*sizeof(BOUND_PREC));
	assert(s);

	offset = (BOUND_PREC *) malloc(nclus * sizeof(BOUND_PREC)); /* change in distance of a cluster mean after a iteration */
	assert(offset);

	mindist = (PREC *)malloc(npts * sizeof(PREC));
	assert(mindist);
	for (i=0;i<npts;i++)
		mindist[i] = PREC_MAX;

	cluster_changed = (bool *) malloc(nclus * sizeof(bool)); /* did the cluster change? */
	assert(cluster_changed);
	for ( j=0 ; j<nclus ; j++ )
		cluster_changed[j] = true;


	iteration = 0;
	nchanged = 1;
	while (iteration < maxiter || maxiter == 0)
	{
		
		/* compute cluster-cluster distances */
		compute_cluster_distances(cl_dist, s, CX, dim,nclus, cluster_changed);
		
		/* find nearest cluster center */
		if (iteration == 0)
		{
			for (i=0, px=X; i<npts ; i++,px+=dim)
			{
				c[i] = init_point_to_cluster(i,px,CX,dim,nclus,mindist,low_b,cl_dist);
				add_point_to_cluster(c[i],tCX,px,CN,dim);
			}
			nchanged = npts;
		}
		else
		{
			for ( j=0 ; j<nclus ; j++)
				cluster_changed[j] = false;

			nchanged = 0;
			for (i=0, px=X; i<npts ; i++,px+=dim)
			{
				c[i] = assign_point_to_cluster(i,px,CX,dim,nclus,old_c[i],mindist,s,cl_dist,low_b);

				if (old_c[i] == c[i]) continue;
				
				nchanged++;

				cluster_changed[c[i]] = true;
				cluster_changed[old_c[i]] = true;

				remove_point_from_cluster(old_c[i],tCX,px,CN,dim);
				add_point_to_cluster(c[i],tCX,px,CN,dim);
			}

		}


		/* fill up empty clusters */
		for (j=0 ; j<nclus ; j++)
		{
			if (CN[j]>0) continue;
			if (!rperm)
			{
				rperm = (unsigned int*)malloc(npts*sizeof(unsigned int));
				assert(rperm);
			}
			randperm(rperm,npts);
			i = 0; 
			while (rperm[i]<npts && CN[c[rperm[i]]]<2) i++;
			if (i==npts)continue;
			i = rperm[i];
#ifdef KMEANS_DEBUG
			printf("empty cluster [%d], filling it with point [%d]\n",j,i);
#endif
			cluster_changed[c[rperm[i]]] = true;
			cluster_changed[j] = true;
			px = X + i*dim;
			remove_point_from_cluster(c[i],tCX,px,CN,dim);
			c[i] = j;
			add_point_to_cluster(j,tCX,px,CN,dim);
			/* void the bounds */
			s[j] = (BOUND_PREC)0.0;
			mindist[i] = 0.0;
			if (use_low_b)
				for ( k=0 ; k<npts ; k++ )
					low_b[k*nclus+j] = (BOUND_PREC)0.0;
			
			nchanged++;

		}

		/* no assignment changed: done */
		if (nchanged==0) break; 

		/* compute the offset */
		for ( j=0,pcx=CX,tpcx=tCX; j<nclus ; j++,pcx+=dim,tpcx+=dim )
		{
			offset[j] = (BOUND_PREC)0.0;
			if (cluster_changed[j])
			{
				offset[j] = (BOUND_PREC)compute_distance(pcx,tpcx,dim);
				memcpy(pcx,tpcx,dim*sizeof(PREC));
			}
		}
		
		/* update the lower bound */
		if (use_low_b)
		{
			for ( i=0,cnt=0 ; i<npts ; i++ )
				for ( j=0 ; j<nclus ; j++,cnt++ )
				{
					low_b[cnt] -= offset[j];
					if (low_b[cnt]<(BOUND_PREC)0.0) low_b[cnt] = (BOUND_PREC)0.0;
				}
		}

		for ( i=0; i<npts; i++)
			mindist[i] += (PREC)offset[c[i]];

		memcpy(old_c,c,npts*sizeof(unsigned int));

#ifdef KMEANS_VERBOSE
		sse = compute_sserror(CX,X,c,dim,npts);
		printf("iteration %4d, #(changed points): %4d, sse: %4.2f\n",(int)iteration,(int)nchanged,sse);
#endif

#ifdef KMEANS_DEBUG
		printf("saved at 2) %d\n",saved_two);
		printf("saved at 3i) %d\n",saved_three_one);
		printf("saved at 3ii) %d\n",saved_three_two);
		printf("saved at 3iii) %d\n",saved_three_three);
		printf("saved at 3b) %d\n",saved_three_b);
		saved_two=0;
		saved_three_one=0;
		saved_three_two=0;
		saved_three_three=0;
		saved_three_b=0;
#endif

		iteration++;

	}

#ifdef KMEANS_DEBUG
	for (j=0;j<nclus;j++)
		if (CN[j]==0)
			printf("BUG: EMPTY CLUSTER AFTERT ALL\n");
#endif


	/* find nearest cluster center if iteration reached maxiter */
	if (nchanged>0)
		for (i=0, px=X; i<npts ; i++,px+=dim)
			c[i] = assign_point_to_cluster_ordinary(px,CX,dim,nclus);

	sse = compute_sserror(CX,X,c,dim,npts);

#ifdef KMEANS_VERBOSE
	printf("iteration %4d, #(changed points): %4d, sse: %4.2f\n",(int)iteration,(int)nchanged,sse);
#endif

	if(low_b) free(low_b);
	free(cluster_changed);
	free(mindist);
	free(s);
	free(offset);
	free(cl_dist);
	free(tCX);
	free(CN);
	free(old_c);
	if(rperm) free(rperm);

	return(sse);
}

PREC kmeans(PREC *CX,const PREC *X,unsigned int *assignment,unsigned int dim,unsigned int npts,unsigned int nclus,unsigned int maxiter, unsigned int restarts, bool randomInitialCentres)
{
	PREC sse, minsse;
	unsigned int res = restarts;
	unsigned int k,i;
	unsigned int *order, *bestassignment;
	PREC *bestCX;

    if (randomInitialCentres)
	{
		order = (unsigned int*)malloc(npts*sizeof(unsigned int));

		/* generate new starting point */
		randperm(order,npts);
		for (i=0; i<nclus; i++)
			for (k=0; k<dim; k++ )
				CX[(i*dim)+k] = (PREC)X[order[i]*dim+k];
		free(order);		
	}

/*
	if (CX==NULL)
	{
		order = (unsigned int*)malloc(npts*sizeof(unsigned int));

		CX = (double*)calloc(nclus*dim,sizeof(double));
		// generate new starting point
		randperm(order,npts);
		for (i=0; i<nclus; i++)
			for (k=0; k<dim; k++ )
				CX[(i*dim)+k] = (double)X[order[i]*dim+k];
		free(order);
		
	}
*/
	sse = kmeans_run(CX,X,assignment,dim,npts,nclus,maxiter);
	
	if (res>0)
	{
		minsse = sse;
		order = (unsigned int*)malloc(npts*sizeof(unsigned int));
		bestCX = (PREC*) malloc(dim*nclus*sizeof(PREC));
		bestassignment = (unsigned int*)malloc(npts*sizeof(unsigned int));

		memcpy(bestCX,CX,dim*nclus*sizeof(PREC));
		memcpy(bestassignment,assignment,npts*sizeof(unsigned int));

		while (res>0)
		{

			/* generate new starting point */
			randperm(order,npts);
			for (i=0; i<nclus; i++)
				for (k=0; k<dim; k++ )
					CX[(i*dim)+k] = (PREC)X[order[i]*dim+k];
		
			sse = kmeans_run(CX,X,assignment,dim,npts,nclus,maxiter);
			if (sse<minsse)
			{
#ifdef KMEANS_VERBOSE
				printf("found a better clustering with sse = %g\n",sse);
#endif
				minsse = sse;
				memcpy(bestCX,CX,dim*nclus*sizeof(PREC));
				memcpy(bestassignment,assignment,npts*sizeof(unsigned int));
			}
			res--;



		}
		memcpy(CX,bestCX,dim*nclus*sizeof(PREC));
		memcpy(assignment,bestassignment,npts*sizeof(unsigned int));
		sse = minsse;
		free(bestassignment);
		free(bestCX);
		free(order);
	}
	return(sse);
}
