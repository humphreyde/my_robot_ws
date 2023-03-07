#ifndef POSITION_RANDOM_H
#define POSITION_RANDOM_H

#include <iostream>
#include <ctime>
#include <random>

//using namespace std;

/**
 * @namespace webots_api
 */
namespace webots_api{
    /**
     * @class PositionRandom
     */
    class PositionRandom{
    public:
        /**
         * @brief constructor
         */
        PositionRandom();

        /**
         * @brief destructor
         */
        ~PositionRandom();

        double positionRandom(const double max, const double min);

    private:

    };
}

#endif //position_random
