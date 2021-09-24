
#include "shared/util/random.h"
util_random::Random rng_;

double gaussian(double mean, double std_dev) {
    return rng_.Gaussian(mean, std_dev);
}