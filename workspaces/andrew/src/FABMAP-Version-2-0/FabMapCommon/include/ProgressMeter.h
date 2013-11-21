#ifndef PROGRESS_METER_XYZ_H
#define PROGRESS_METER_XYZ_H 1
#include <iomanip>

class ProgressMeter
{
    public:    
        ProgressMeter(unsigned int table_size)
        {
            v = (double) table_size;
            v_sq = v*v;
            last_percentage_reported = 0.0;
        }

        //Writes progress to the screen
        //for computation filling in an upper-triangular matrix
        //of v rows
        void WriteProgressUpperTriangle(unsigned int i) //i is the current row.
        {
            double percent_done = floor(100.0*(2.0*v*i - i*i)/(v_sq));
            if(fmod(percent_done,10.0) == 0.0)
            {
                if(percent_done - last_percentage_reported > 5.0)
                {
                    last_percentage_reported = percent_done;
                    cout <<  setprecision (3) <<  percent_done << "%...";
                }
            }
        }

        //Writes progress to the screen
        //for computation filling in an lower-triangular matrix
        //of v rows
        void WriteProgressLowerTriangle(unsigned int i) //i is the current row.
        {
            double percent_done = floor(100.0*(i*i)/(v_sq));
            if(fmod(percent_done,10.0) == 0.0)
            {
                if(percent_done - last_percentage_reported > 5.0)
                {
                    last_percentage_reported = percent_done;
                    cout << endl << setprecision (3) <<  percent_done << "%" << endl;
                }
            }
        }

        void WriteProgressLinear(unsigned int i) //i is the current row.
        {
            double percent_done = floor(100.0*(i/v));
            if(fmod(percent_done,10.0) == 0.0)
            {
                if(percent_done - last_percentage_reported > 5.0)
                {
                    last_percentage_reported = percent_done;
                    cout << endl << setprecision (3) <<  percent_done << "%" << endl;
                }
            }
        }


    private:
        double v;
        double v_sq;
        double last_percentage_reported;
};

#endif
