#pragma once

#include <ctime>
#include <string>
#include "commondef.h"


namespace DexRobot
{

enum TS_RADIX
{
    SECOND,
    MILLISECOND,
    MICROSECOND,
    NANOSECOND
};

/**
 * @brief current_timestamp  Retrieve current UTC timestamp in microseconds
 * @return A long int value represents microseconds since Epoch
 */
DEXHAND_API int64_t current_timestamp();
DEXHAND_API std::tm * timestamp_to_utc_datetime(int64_t timestamp, TS_RADIX src_ts_radix=MILLISECOND);
DEXHAND_API std::tm * timestamp_to_loc_datetime(int64_t timestamp, TS_RADIX src_ts_radix=MILLISECOND);

DEXHAND_API std::string timestamp_to_utc_string(int64_t timestamp, TS_RADIX src_ts_radix=MILLISECOND);
DEXHAND_API std::string timestamp_to_loc_string(int64_t timestamp, TS_RADIX src_ts_radix=MILLISECOND);

}
