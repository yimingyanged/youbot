#ifndef EDGE_SE2_VIS_H
#define EDGE_SE2_VIS_H

#include "g2o/types/slam2d/edge_se2.h"

namespace g2o {
    class EdgeSE2Vis : public EdgeSE2
    {
        public:
            EdgeSE2Vis() : EdgeSE2(), from_loop_closure_(false){};
            void setLoopClosure(){from_loop_closure_ = true;};
            bool isLoopClosure(){return from_loop_closure_;};
        private:
            bool from_loop_closure_;
    };
};
#endif
