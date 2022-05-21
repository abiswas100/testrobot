
////////////////////////////////////
// Classes, structures, and functions
// to develop a facility management plan.
//
// Written by:  Matthew Peavy
//              Kyungki Kim
// (c) 2021
//

#ifndef UNL_ROBOTICS_INSPECTION_PLAN_H
#define UNL_ROBOTICS_INSPECTION_PLAN_H

//std
#include <vector>
#include <iostream>
#include <string>
#include <tuple>

namespace UNL_Robotics {

  enum class RotationDirection {eCCW =0, eCW};
  RotationDirection parseRotationDirection(const std::string&);
  
  struct Position_2D {
    double x;
    double y;
    double yaw;
  };

  struct Room {
    unsigned roomID;
    std::string roomName;
    Position_2D roomCenter;
    unsigned maxOccupancy;
  };

  class RoomRoute {
  public:
    RoomRoute(const Room& room, unsigned numRotations);

    const Room& room() const {return m_room;}
    unsigned getNumRotations() const {return m_numRotations;}

    void print(std::ostream&) const;

  private:
    Room m_room;
    unsigned m_numRotations;
  };

  //A series of room routes makes up an inspection route
  typedef std::vector<RoomRoute> InspectionRoute;
  
  std::tuple<bool, InspectionRoute> parseInspectionRoute(std::istream&);
}

std::ostream& operator<<(std::ostream&, const UNL_Robotics::InspectionRoute&);

#endif

