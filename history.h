//      ******************************************************************
//      *                                                                *
//      *   history.h  -  History: a generic rolling ring-buffer of      *
//      *                 HISTORY_POINTS byte-valued samples, fed by a   *
//      *                 running accumulator that averages raw samples  *
//      *                 into one point every `samplesPerBucket` calls  *
//      *                 to addSample().                                *
//      *                                                                *
//      *   Public interface:                                            *
//      *     addSample(v)    - feed one already-rounded-to-a-byte raw   *
//      *                       sample into the running accumulator for  *
//      *                       the current bucket                       *
//      *     count()         - how many ring-buffer points are valid    *
//      *     get(i)          - point i, 0 = oldest, count()-1 = newest  *
//      *                                                                *
//      ******************************************************************

#ifndef HISTORY_H
#define HISTORY_H

#include <Arduino.h>
#include "config.h"

class History {
public:
  //
  // Enter: samplesPerBucketIn = how many addSample() calls get averaged
  //        into one ring-buffer point - the owner computes this from its
  //        own "how far back" span / HISTORY_POINTS / CONTROL_INTERVAL.
  //
  explicit History(uint16_t samplesPerBucketIn) : samplesPerBucket(samplesPerBucketIn) {}

  //
  // Accumulate one raw sample; once samplesPerBucket samples have been fed
  // in, their average becomes the next ring-buffer point (overwriting the
  // oldest one once the buffer is full) and the accumulator resets for the
  // next bucket. accumulatorSum is 32-bit because a full bucket can hold
  // thousands of samples (e.g. 3600 for a 48h/24-point history at a 2s
  // sampling interval) - at up to 255 per sample that overflows a 16-bit sum.
  //
  void addSample(uint8_t value) {
    accumulatorSum += value;
    accumulatorCount++;
    if (accumulatorCount < samplesPerBucket) {
      return;
    }

    uint8_t point = (uint8_t)(accumulatorSum / accumulatorCount);
    accumulatorSum = 0;
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
      head = (head + 1) % HISTORY_POINTS;
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
  const uint16_t samplesPerBucket;

  uint8_t points[HISTORY_POINTS];
  uint8_t head = 0;         // index of the oldest valid point
  uint8_t pointCount = 0;   // how many of points[] are valid

  uint32_t accumulatorSum = 0;
  uint16_t accumulatorCount = 0;
};

#endif  // HISTORY_H
