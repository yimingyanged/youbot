/*
 * ObjdctInfo.h
 *
 *  Created on: Nov 20, 2013
 *      Author: raha
 */

#ifndef OBJDCTINFO_H_
#define OBJDCTINFO_H_

#include <string>
#include <kdl/frames.hpp>

namespace youbot_arm_control
{

class ObjectInfo
{
public:
  ObjectInfo();
  virtual
  ~ObjectInfo();

private:
  std::string _name,
              _description;
  int _objId;

  // offset between the objects pose and the point where to grip
  KDL::Frame _grippoint_offset;


};

} /* namespace youbot_arm_control */

#endif /* OBJDCTINFO_H_ */
