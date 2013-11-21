========================
  Vocabulary Generation 
=======================

The vocabulary generation process involves creating the visual words and learning the Chow-Liu tree capturing first order word dependencies [A].

Visual words are determined by first extracting SURF features from a training set of images and then clustering them to obtain visual words. The SURF features are then quantized to the nearest visual word generating a bag-of-words representation of the image. For details please see [B].

The Chow Liu tree is learnt using word co-occurrence statistics through the procedure described in [A].  

The vocabulary generation process needs the following to be built: WordMaker, KMeans, pAcceleratedChowLiu_Fast or pAcceleratedChowLiu_CompactMemory. 

These processes also need config files. Demo config files for this example are in the config directory.

===================
  Quick Start
===================

1) Install ImageMagick from http://www.imagemagick.org
 (On most Linux systems this should already be installed. You can check by typing "convert" from a terminal.)

The sample training data for vocabulary learning is located in:
../Resources/Sample_Data/Images

The output will appear in the other sub-directories of
../Resources/Sample_Data/

The following steps assume that you have complied the binaries are placed them in ./bin:

2)  Open a terminal in the bin directory and call
  
   a)  ./pWordMaker WordMaker_SURFOnlyConfig.moos
 
	This extracts the SURF features from each image placed in the folder called ../Resources/Sample_Data/Surf 

   b) ./pKMeans KMeans_Config.moos
	
	KMeans clustering of the extracted SURF features to yield visual words forming the vocabulary.
	The number of clusters (k) and vocabulary name (Vocab) are specified in the config file. 
	The following appear in the folder ../Resources/Sample_Data/Vocabulary:

		Vocab.oxv	-	Visual word coordinates in SURF space forming the vocabulary.
		Assignments.txt	-	Visual word assigned to each feature during clustering
		config.params	-	Record of configuration parameters for each clustering run.
	
	Note: There are two options for K-Means implementation.
		EXACT uses MPI library, using triangle inequality bounds for fast distance computation. 
		It is very fast and returns exact KMeans solution, but requires O(n*k) memory for n points and k clusters.
		
		APPROX_OUT_OF_CORE returns an approximate solution (using kd-tree for nearest neighbour computation). 
		Can handle arbitrarly large input, but somewhat slower.
		This requires BUILD_KMEANS_WITH_EXTERNAL_MEMORY_CONTAINERS flag to be setup while building with CMake.
		Uses STXXL.    

	
   c)  ./pWordMaker WordMaker_QuantizeConfig.moos
 
	This quantizes the SURF features for each image against the visual words generated (Vocab.oxv) to yield the file Vocab.oxs in ../Resources/Sample_Data/Vocabulary. 
	Vocab.oxs is the bag-of-words representation of the images.

3) Learn Chow-Liu Tree

   ./pAcceleratedChowLiu_Fast LearnChowLiuTree_Config.moos
	or
   ./pAcceleratedChowLiu_CompactMemory LearnChowLiuTree_Config.moos
	
	This learns the Chow Liu tree [A]. The following .mat files appear in the folder ../Resources/Sample_Data/Vocabulary
	
		Vocab_ChowTree.mat		-	Index to parent in the learned tree for each observation variable.
		Vocab_Marginals.mat		-	Marginal probabilities for each visual word, p(e)	
		Vocab_RelevantConditionals.mat	-	Likelihood p(z =1 | parent(z) =1)
		Vocab_RelevantNotConditionals.mat-	Likelihood p(z =0 | parent(z) =0) 

The vocabulary generated (visual words and the Chow Liu Tree) can now be used to run FABMAP on your dataset.
These files are valid Matlab matrices, so you can also open then directly in Matlab.

===================
 References
===================

[A] "FAB-MAP: Probabilistic Localization and Mapping in the Space of Appearance", Mark Cummins and Paul Newman, International Journal or Robotics Research
http://www.robots.ox.ac.uk/~mjc/Papers/IJRR_2007.pdf

[B] "Video Google: A text retrieval approach to object matching in videos.", J. Sivic and A. Zisserman, Proceedings of the International Conference on Computer Vision, 2003. 

======================================

 
