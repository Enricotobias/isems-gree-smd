#pragma once

#include <stddef.h>
#include <stdint.h>

enum DayType : uint8_t {
  DAY_OFFTIME = 0,
  DAY_FULLTIME = 1,
  DAY_PARTTIME = 2
};

#define DAY_FULL DAY_FULLTIME
#define DAY_PART_TIME DAY_PARTTIME
#define DAY_LIBUR DAY_OFFTIME

static const uint8_t MAX_PERIODS_PER_DAY = 10;

struct TimePeriod {
  char start[6];
  char end[6];
  char mode[6];
  char fan[8];
  uint8_t set_temp;
  uint8_t min_temp;
  uint8_t max_temp;
};

struct DaySchedule {
  DayType type;
  uint8_t periodCount;
  TimePeriod periods[MAX_PERIODS_PER_DAY];
};

enum ScheduleResultCode : uint8_t {
  SCHEDULE_RESULT_OK = 0,
  SCHEDULE_RESULT_INVALID_TIME,
  SCHEDULE_RESULT_INVALID_DAY_TYPE,
  SCHEDULE_RESULT_TOO_MANY_PERIODS,
  SCHEDULE_RESULT_OVERLAP,
  SCHEDULE_RESULT_FULLTIME_CONFLICT
};

struct ScheduleResult {
  ScheduleResultCode code;
  int8_t day;
  int8_t period;
  int8_t conflictDay;
  int8_t conflictPeriod;
};

int scheduleParseMinute(const char* hhmm);
bool schedulePeriodsEqual(const TimePeriod& lhs, const TimePeriod& rhs);
bool schedulePeriodsAreContinuous(uint8_t firstDay,
                                  const TimePeriod& first,
                                  uint8_t secondDay,
                                  const TimePeriod& second);
void scheduleSortPeriods(DaySchedule& day);
ScheduleResult scheduleValidateWeek(const DaySchedule days[7]);
ScheduleResult scheduleInsertPeriods(DaySchedule& day,
                                     const TimePeriod* periods,
                                     uint8_t count,
                                     uint8_t* addedCount);
const TimePeriod* scheduleFindActivePeriod(const DaySchedule days[7],
                                           uint8_t currentDay,
                                           int currentMinute);

