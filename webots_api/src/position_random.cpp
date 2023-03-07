#include "webots_api/position_random.h"

/**
 * @namespace webots_api
 */
namespace webots_api{
    /**
     * @class PositionRandom
     */

    //constructor
    PositionRandom::PositionRandom() {}

    //destructor
    PositionRandom::~PositionRandom() {}

    //position random
    double PositionRandom::positionRandom(const double max, const double min){

      std::default_random_engine e(time(0));
      std::uniform_real_distribution<double> u(max, min);

      return u(e);
    }


}//namespace gait
