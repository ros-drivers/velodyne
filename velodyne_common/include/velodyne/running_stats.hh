/*
 *  Copyright (C) 2009 Austin Robot Technology, Jack O'Quin
 *
 *  License: Modified BSD Software License Agreement
 */

/** \file 
 *  \brief Class to keep running mean and variance of an input series.
 *
 *  This implementation works incrementally.  When given a new sample,
 *  it updates internal state, without storing the entire history.
 *
 *  TODO: replace this version with Knuth's more numerically stable
 *  algorithm.  See: "On-line_algorithm" section of Wikipedia article
 *  "Algorithms_for_calculating_variance".
 *
 *  \author Jack O'Quin
 *
 *  $Id$
 **/

#ifndef RUNNING_STATS_HH
#define RUNNING_STATS_HH

#include <cmath>

class RunningStats {
public:
  RunningStats()
  {
    reset();
  }

  // Add a new sample to the series.
  void inline addSample(double new_value)
  {
    ++N_;
    sum_ += new_value;
    sum2_ += new_value * new_value;
  }

  // Reset the sample data.
  void reset(void)
  {
    N_ = 0;
    sum_ = 0.0;
    sum2_ = 0.0;
  }

  // Return the sample mean.
  double inline getMean(void)
  {
    return (sum_ / N_);
  }

  // Return the sample variance using Bessel's correction.
  double inline getVariance(void)
  {
    double mean = getMean();
    if (N_ > 1)
      return (sum2_/(N_-1) - ((double)N_/(N_-1))*mean*mean);
    else
      return 0.0;
  }

  // Return the sample standard deviation using Bessel's correction.
  double inline getStandardDeviation(void)
  {
    return sqrt(getVariance());
  }

private:
  int    N_;                            // number of samples seen
  double sum_;                          // sum of samples seen
  double sum2_;                         // sum of squares of samples seen
};

#endif // RUNNING_STATS_HH
