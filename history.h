//      ******************************************************************
//      *                                                                *
//      *   history.h  -  History: a generic rolling ring-buffer of      *
//      *                 data points, fed by a running                  *
//      *                 accumulator that averages raw samples into     *
//      *                 the next data point.                           *
//      *                                                                *
//      *   Public interface:                                            *
//      *     addSample(v)    - feed raw sample into the running         *
//      *                       accumulator for the current data point   *
//      *     count()         - how many ring-buffer points are valid    *
//      *     get(i)          - point i, 0 = oldest, count()-1 = newest  *
//      *                                                                *
//      ******************************************************************

#ifndef HISTORY_H
#define HISTORY_H

#include <Arduino.h>
#include "config.h"

class History
{
public:
  //
  // Enter: samplesPerDataPointIn = how many addSample() calls get averaged
  //        into one ring-buffer point - the owner computes this from its
  //        own "how far back" span / HISTORY_POINTS / CONTROL_INTERVAL.
  //
  explicit History(uint16_t samplesPerDataPointIn) : samplesPerDataPoint(samplesPerDataPointIn) {}

  //
  // Accumulate one raw sample; once samplesPerDataPoint samples have been fed
  // in, their average becomes the next data  point (overwriting the
  // oldest one once the buffer is full) and the accumulator resets for the
  // next data point.
  void addSample(uint8_t value) {
    accumulatorSum += value;
    accumulatorCount++;
    if (accumulatorCount < samplesPerDataPoint) {
      return;
    }

    uint8_t point    = (uint8_t) (accumulatorSum / accumulatorCount);
    accumulatorSum   = 0;
    accumulatorCount = 0;

    //
    // write into the ring buffer, overwriting the oldest point once full
    //
    uint8_t writeIndex;
    if (pointCount < HISTORY_POINTS) {
      writeIndex = (head + pointCount) % HISTORY_POINTS;
      pointCount++;
    } else {
      writeIndex = head;
      head       = (head + 1) % HISTORY_POINTS;
    }
    points[writeIndex] = point;
  }

  //
  // how many points are valid so far (climbs from 0 up to HISTORY_POINTS,
  // then stays there as older points start being overwritten)
  //
  uint8_t count() const {
    return pointCount;
  }

  //
  // logical index 0 = oldest valid point, count()-1 = newest
  //
  uint8_t get(uint8_t logicalIndex) const {
    return points[(head + logicalIndex) % HISTORY_POINTS];
  }

private:
  const uint16_t samplesPerDataPoint; // addSample() calls averaged into one ring-buffer point

  uint8_t points[HISTORY_POINTS]; // the ring buffer itself
  uint8_t head       = 0;         // index of the oldest valid point
  uint8_t pointCount = 0;         // how many of points[] are valid

  uint32_t accumulatorSum   = 0; // running sum for the data point currently accumulating
  uint16_t accumulatorCount = 0; // samples folded into accumulatorSum so far
};

#endif // HISTORY_H
