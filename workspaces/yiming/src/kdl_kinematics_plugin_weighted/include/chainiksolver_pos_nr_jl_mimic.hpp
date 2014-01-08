#ifndef KDLCHAINIKSOLVERPOS_NR_JL_Mimic_HPP
#define KDLCHAINIKSOLVERPOS_NR_JL_Mimic_HPP

#include "kdl/chainiksolver.hpp"
#include "kdl/chainfksolver.hpp"

#include <moveit/kdl_kinematics_plugin/joint_mimic.hpp>

namespace KDL
{
    
    /**
     * Implementation of a general inverse position kinematics
     * algorithm based on Newton-Raphson iterations to calculate the
     * position transformation from Cartesian to joint space of a general
     * KDL::Chain. Takes joint limits into account.
     *
     * @ingroup KinematicFamily
     */
    class ChainIkSolverPos_NR_JL_Mimic : public ChainIkSolverPos
    {
    public:
        /**
         * Constructor of the solver, it needs the chain, a forward
         * position kinematics solver and an inverse velocity
         * kinematics solver for that chain.
         *
         * @param chain the chain to calculate the inverse position for
         * @param q_max the maximum joint positions
         * @param q_min the minimum joint positions
         * @param fksolver a forward position kinematics solver
         * @param iksolver an inverse velocity kinematics solver
         * @param maxiter the maximum Newton-Raphson iterations,
         * default: 100
         * @param eps the precision for the position, used to end the
         * iterations, default: epsilon (defined in kdl.hpp)
         *
         * @return
         */
        ChainIkSolverPos_NR_JL_Mimic(const Chain& chain,const JntArray& q_min, const JntArray& q_max, ChainFkSolverPos& fksolver,ChainIkSolverVel& iksolver,unsigned int maxiter=100,double eps=1e-6, bool position_ik = false);
        
        ~ChainIkSolverPos_NR_JL_Mimic();
        
        virtual int CartToJnt(const JntArray& q_init, const Frame& p_in, JntArray& q_out);
        
        virtual int CartToJntAdvanced(const JntArray& q_init, const Frame& p_in, JntArray& q_out, bool lock_redundant_joints);
        
        bool setMimicJoints(const std::vector<kdl_kinematics_plugin::JointMimic>& mimic_joints);
        
    private:
        const Chain chain;
        JntArray q_min;//These are the limits for the "reduced" state consisting of only active DOFs
        JntArray q_min_mimic;//These are the limits for the full state
        JntArray q_max;//These are the limits for the "reduced" state consisting of only active DOFs
        JntArray q_max_mimic;//These are the limits for the full state
        JntArray q_temp;
        ChainFkSolverPos& fksolver;
        ChainIkSolverVel& iksolver;
        JntArray delta_q;
        Frame f;
        Twist delta_twist;
        unsigned int maxiter;
        double eps;
        std::vector<kdl_kinematics_plugin::JointMimic> mimic_joints;
        void qToqMimic(const JntArray& q, JntArray& q_result); //Convert from the "reduced" state (only active DOFs) to the "full" state
        void qMimicToq(const JntArray& q, JntArray& q_result); //Convert from the "full" state to the "reduced" state
        bool position_ik;
        
    };
    
}

#endif