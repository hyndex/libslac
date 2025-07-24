// SPDX-License-Identifier: Apache-2.0
// Copyright 2024 Joulepoint Private Limited (Author Chinmoy Bhuyan)
#pragma once

#include <cstdio>

#define LOG_INFO(fmt, ...) std::printf(fmt, ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...) std::fprintf(stderr, fmt, ##__VA_ARGS__)

