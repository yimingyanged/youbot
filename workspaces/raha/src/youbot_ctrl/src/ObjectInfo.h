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

namespace youbot_ctrl
{

class ObjectInfo
{
public:
  ObjectInfo();
  virtual
  ~ObjectInfo();

private:
  std::string name,
              description;
  int objId;

  // offset between the objects pose and the point where to grip
  KDL::frame _grippoint_offset;


};

} /* namespace youbot_ctrl */

#endif /* OBJDCTINFO_H_ */
