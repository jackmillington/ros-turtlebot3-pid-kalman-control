/*
 * noise.cc
 * Copyright (C) 2022 Morgan McColl <morgan.mccoll@alumni.griffithuni.edu.au>
 *
 * Distributed under terms of the MIT license.
 */

#include <random>

double addNoise(double value, double stdDeviation) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> d(value, stdDeviation);
    return d(gen);
}

