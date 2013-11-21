#ifndef CONFIG_LOC_PROB_H
#define CONFIG_LOC_PROB_H 1

//******************************************
//            Data Association
//******************************************
//#define ALLOW_DATA_ASSOCIATION //If not allowed, we do not chunk matched images into places, 
                                 //Instead, at each step we compare each image to all previous images.

#define LOCAL_LOOP_CLOSURE_EXCLUSION_ZONE 10    //This setting effects only reporting of loop closures, not calculation of PDF.

//******************************************
//    Some Debugging flags
//******************************************

//#define BAE_OUTPUT_FORMAT

//#define DEBUG_VISUALIZATION

//#define DEBUG_CASES                            //Go step by step, print out lots of data

//#define DEBUG_FOR_ONLINE_MODE // Used for statements used while updating the online and hybrid modes. Enable while debugging online mode when data association is turned on.

#endif    //CONFIG_LOC_PROB_H