#ifndef MOVEIT_KDL_KINEMATICS_PLUGIN_JOINT_MIMIC
#define MOVEIT_KDL_KINEMATICS_PLUGIN_JOINT_MIMIC

namespace ipab_weighted_ik
{
    
    /** \brief A model of a mimic joint. Mimic joints are typically unactuated joints
     that are constrained to follow the motion of another joint. The constraint is linear, i.e.
     joint_angle_constrained_joint = joint_angle_mimicked_joint*multiplier + offset
     */
    class JointMimic
    {
    public:
        
        JointMimic() { this->reset(0); };
        
        /** \brief Offset for this joint value from the joint that it mimics */
        double offset;
        /** \brief Multiplier for this joint value from the joint that it mimics */
        double multiplier;
        /** \brief Index of the joint that this joint mimics in the vector of active degrees of freedom */
        unsigned int map_index;
        /** \brief Name of this joint */
        std::string joint_name;
        /** \brief If true, this joint is an active DOF and not a mimic joint*/
        bool active;
        
        void reset(unsigned int index)
        {
            offset = 0.0;
            multiplier = 1.0;
            map_index = index;
            active = false;
        };
    };
    
}

#endif
