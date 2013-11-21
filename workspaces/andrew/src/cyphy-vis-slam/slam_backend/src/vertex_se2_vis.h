#ifndef VERTEX_SE2_VIS_H
#define VERTEX_SE2_VIS_H

#include "g2o/types/slam2d/vertex_se2.h"

namespace g2o {
    class VertexSE2Vis : public VertexSE2
    {
        public:
            VertexSE2Vis() : VertexSE2(), from_loop_closure_(false){};
            void setLoopClosure(){from_loop_closure_ = true;};
            bool isLoopClosure(){return from_loop_closure_;};
        private:
            bool from_loop_closure_;
    };
};
#endif
