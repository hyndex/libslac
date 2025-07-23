// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 - 2022 Pionix GmbH and Contributors to EVerest
#pragma once

#include <cstdio>

#define LOG_INFO(fmt, ...) std::printf(fmt, ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...) std::fprintf(stderr, fmt, ##__VA_ARGS__)

