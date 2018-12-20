/*
 * GradMagChannel.cpp
 *
 *  Created on: Jan 15, 2015
 *      Author: fds
 */
//#include "imResampleMex.hpp"
//#include "rgbConvertMex.hpp"

#include "GradMagChannel.h"
#include "../Channel/Functions.h"

GradMagChannel::~GradMagChannel() {
    free(orientation);
}

float * GradMagChannel::getMagnitude() const {
    return this->mag;
}

float * GradMagChannel::getOrientation() const {
    return this->orientation;
}
