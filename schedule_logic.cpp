#include "schedule_logic.h"

#include <string.h>

namespace {

static const int MINUTES_PER_DAY = 1440;
static const int MINUTES_PER_WEEK = 7 * MINUTES_PER_DAY;

struct MinuteRange {
  int start;
  int end;
};

ScheduleResult makeResult(ScheduleResultCode code,
                          int8_t day = -1,
                          int8_t period = -1,
                          int8_t conflictDay = -1,
                          int8_t conflictPeriod = -1) {
  ScheduleResult result = {code, day, period, conflictDay, conflictPeriod};
  return result;
}

bool sameTimeRange(const TimePeriod& lhs, const TimePeriod& rhs) {
  return strcmp(lhs.start, rhs.start) == 0 && strcmp(lhs.end, rhs.end) == 0;
}

uint8_t buildWeeklyRanges(uint8_t day,
                          const TimePeriod& period,
                          MinuteRange ranges[2]) {
  const int startMinute = scheduleParseMinute(period.start);
  const int endMinute = scheduleParseMinute(period.end);
  if (day > 6 || startMinute < 0 || endMinute < 0) return 0;

  const int absoluteStart = day * MINUTES_PER_DAY + startMinute;
  int absoluteEnd = day * MINUTES_PER_DAY + endMinute;
  if (endMinute < startMinute) absoluteEnd += MINUTES_PER_DAY;

  if (absoluteEnd < MINUTES_PER_WEEK) {
    ranges[0] = {absoluteStart, absoluteEnd};
    return 1;
  }

  ranges[0] = {absoluteStart, MINUTES_PER_WEEK - 1};
  ranges[1] = {0, absoluteEnd - MINUTES_PER_WEEK};
  return 2;
}

bool rangesOverlap(const MinuteRange& lhs, const MinuteRange& rhs) {
  return lhs.start <= rhs.end && rhs.start <= lhs.end;
}

bool periodsOverlap(uint8_t lhsDay,
                    const TimePeriod& lhs,
                    uint8_t rhsDay,
                    const TimePeriod& rhs) {
  MinuteRange lhsRanges[2] = {};
  MinuteRange rhsRanges[2] = {};
  const uint8_t lhsCount = buildWeeklyRanges(lhsDay, lhs, lhsRanges);
  const uint8_t rhsCount = buildWeeklyRanges(rhsDay, rhs, rhsRanges);
  if (lhsCount == 0 || rhsCount == 0) return false;

  for (uint8_t i = 0; i < lhsCount; ++i) {
    for (uint8_t j = 0; j < rhsCount; ++j) {
      if (rangesOverlap(lhsRanges[i], rhsRanges[j])) return true;
    }
  }
  return false;
}

}  // namespace

int scheduleParseMinute(const char* hhmm) {
  if (!hhmm || strlen(hhmm) != 5 || hhmm[2] != ':') return -1;
  if (hhmm[0] < '0' || hhmm[0] > '9' || hhmm[1] < '0' || hhmm[1] > '9' ||
      hhmm[3] < '0' || hhmm[3] > '9' || hhmm[4] < '0' || hhmm[4] > '9') {
    return -1;
  }

  const int hour = (hhmm[0] - '0') * 10 + (hhmm[1] - '0');
  const int minute = (hhmm[3] - '0') * 10 + (hhmm[4] - '0');
  if (hour > 23 || minute > 59) return -1;
  return hour * 60 + minute;
}

bool schedulePeriodsEqual(const TimePeriod& lhs, const TimePeriod& rhs) {
  return strcmp(lhs.start, rhs.start) == 0 && strcmp(lhs.end, rhs.end) == 0 &&
         strcmp(lhs.mode, rhs.mode) == 0 && strcmp(lhs.fan, rhs.fan) == 0 &&
         lhs.set_temp == rhs.set_temp && lhs.min_temp == rhs.min_temp &&
         lhs.max_temp == rhs.max_temp;
}

bool schedulePeriodsAreContinuous(uint8_t firstDay,
                                  const TimePeriod& first,
                                  uint8_t secondDay,
                                  const TimePeriod& second) {
  const int firstStart = scheduleParseMinute(first.start);
  const int firstEnd = scheduleParseMinute(first.end);
  const int secondStart = scheduleParseMinute(second.start);
  if (firstDay > 6 || secondDay > 6 || firstStart < 0 || firstEnd < 0 || secondStart < 0) {
    return false;
  }

  int firstEndAbsolute = firstDay * MINUTES_PER_DAY + firstEnd;
  if (firstEnd < firstStart) firstEndAbsolute += MINUTES_PER_DAY;

  int secondStartAbsolute = secondDay * MINUTES_PER_DAY + secondStart;
  while (secondStartAbsolute <= firstDay * MINUTES_PER_DAY + firstStart) {
    secondStartAbsolute += MINUTES_PER_WEEK;
  }
  return secondStartAbsolute == firstEndAbsolute + 1;
}

void scheduleSortPeriods(DaySchedule& day) {
  uint8_t count = day.periodCount;
  if (count > MAX_PERIODS_PER_DAY) count = MAX_PERIODS_PER_DAY;
  for (uint8_t i = 1; i < count; ++i) {
    TimePeriod current = day.periods[i];
    const int currentStart = scheduleParseMinute(current.start);
    uint8_t position = i;
    while (position > 0 &&
           scheduleParseMinute(day.periods[position - 1].start) > currentStart) {
      day.periods[position] = day.periods[position - 1];
      --position;
    }
    day.periods[position] = current;
  }
}

ScheduleResult scheduleValidateWeek(const DaySchedule days[7]) {
  if (!days) return makeResult(SCHEDULE_RESULT_INVALID_DAY_TYPE);

  for (uint8_t day = 0; day < 7; ++day) {
    const DaySchedule& current = days[day];
    if (current.type > DAY_PARTTIME) {
      return makeResult(SCHEDULE_RESULT_INVALID_DAY_TYPE, day);
    }
    if (current.periodCount > MAX_PERIODS_PER_DAY) {
      return makeResult(SCHEDULE_RESULT_TOO_MANY_PERIODS, day);
    }
    if (current.type == DAY_OFFTIME && current.periodCount != 0) {
      return makeResult(SCHEDULE_RESULT_INVALID_DAY_TYPE, day);
    }
    if (current.type == DAY_FULLTIME && current.periodCount != 1) {
      return makeResult(SCHEDULE_RESULT_INVALID_DAY_TYPE, day);
    }
    if (current.type == DAY_PARTTIME && current.periodCount == 0) {
      return makeResult(SCHEDULE_RESULT_INVALID_DAY_TYPE, day);
    }

    for (uint8_t period = 0; period < current.periodCount; ++period) {
      if (scheduleParseMinute(current.periods[period].start) < 0 ||
          scheduleParseMinute(current.periods[period].end) < 0) {
        return makeResult(SCHEDULE_RESULT_INVALID_TIME, day, period);
      }
    }
  }

  for (uint8_t lhsDay = 0; lhsDay < 7; ++lhsDay) {
    const DaySchedule& lhsSchedule = days[lhsDay];
    for (uint8_t lhsPeriod = 0; lhsPeriod < lhsSchedule.periodCount; ++lhsPeriod) {
      for (uint8_t rhsDay = lhsDay; rhsDay < 7; ++rhsDay) {
        const DaySchedule& rhsSchedule = days[rhsDay];
        const uint8_t rhsStart = (rhsDay == lhsDay) ? lhsPeriod + 1 : 0;
        for (uint8_t rhsPeriod = rhsStart; rhsPeriod < rhsSchedule.periodCount; ++rhsPeriod) {
          if (periodsOverlap(lhsDay,
                             lhsSchedule.periods[lhsPeriod],
                             rhsDay,
                             rhsSchedule.periods[rhsPeriod])) {
            return makeResult(SCHEDULE_RESULT_OVERLAP,
                              lhsDay,
                              lhsPeriod,
                              rhsDay,
                              rhsPeriod);
          }
        }
      }
    }
  }

  return makeResult(SCHEDULE_RESULT_OK);
}

ScheduleResult scheduleInsertPeriods(DaySchedule& day,
                                     const TimePeriod* periods,
                                     uint8_t count,
                                     uint8_t* addedCount) {
  if (addedCount) *addedCount = 0;
  if (!periods || count == 0) return makeResult(SCHEDULE_RESULT_INVALID_TIME);
  if (day.type == DAY_FULLTIME) return makeResult(SCHEDULE_RESULT_FULLTIME_CONFLICT);

  DaySchedule candidate = day;
  if (candidate.periodCount > MAX_PERIODS_PER_DAY) {
    return makeResult(SCHEDULE_RESULT_TOO_MANY_PERIODS);
  }
  if (candidate.type == DAY_OFFTIME) {
    memset(&candidate, 0, sizeof(candidate));
    candidate.type = DAY_PARTTIME;
  }
  if (candidate.type != DAY_PARTTIME) return makeResult(SCHEDULE_RESULT_INVALID_DAY_TYPE);

  uint8_t added = 0;
  for (uint8_t incoming = 0; incoming < count; ++incoming) {
    if (scheduleParseMinute(periods[incoming].start) < 0 ||
        scheduleParseMinute(periods[incoming].end) < 0) {
      return makeResult(SCHEDULE_RESULT_INVALID_TIME, -1, incoming);
    }

    bool duplicate = false;
    for (uint8_t existing = 0; existing < candidate.periodCount; ++existing) {
      if (schedulePeriodsEqual(candidate.periods[existing], periods[incoming])) {
        duplicate = true;
        break;
      }
      if (sameTimeRange(candidate.periods[existing], periods[incoming]) ||
          periodsOverlap(0, candidate.periods[existing], 0, periods[incoming])) {
        return makeResult(SCHEDULE_RESULT_OVERLAP, -1, incoming, -1, existing);
      }
    }
    if (duplicate) continue;
    if (candidate.periodCount >= MAX_PERIODS_PER_DAY) {
      return makeResult(SCHEDULE_RESULT_TOO_MANY_PERIODS, -1, incoming);
    }
    candidate.periods[candidate.periodCount++] = periods[incoming];
    ++added;
  }

  scheduleSortPeriods(candidate);
  day = candidate;
  if (addedCount) *addedCount = added;
  return makeResult(SCHEDULE_RESULT_OK);
}

const TimePeriod* scheduleFindActivePeriod(const DaySchedule days[7],
                                           uint8_t currentDay,
                                           int currentMinute) {
  if (!days || currentDay > 6 || currentMinute < 0 || currentMinute >= MINUTES_PER_DAY) {
    return nullptr;
  }

  const DaySchedule& today = days[currentDay];
  if (today.type != DAY_OFFTIME) {
    const uint8_t count = today.periodCount > MAX_PERIODS_PER_DAY
                              ? MAX_PERIODS_PER_DAY
                              : today.periodCount;
    for (uint8_t period = 0; period < count; ++period) {
      const int start = scheduleParseMinute(today.periods[period].start);
      const int end = scheduleParseMinute(today.periods[period].end);
      if (start < 0 || end < 0) continue;
      if (end >= start && currentMinute >= start && currentMinute <= end) {
        return &today.periods[period];
      }
      if (end < start && currentMinute >= start) {
        return &today.periods[period];
      }
    }
  }

  const uint8_t previousDay = (currentDay + 6) % 7;
  const DaySchedule& previous = days[previousDay];
  if (previous.type != DAY_OFFTIME) {
    const uint8_t count = previous.periodCount > MAX_PERIODS_PER_DAY
                              ? MAX_PERIODS_PER_DAY
                              : previous.periodCount;
    for (uint8_t period = 0; period < count; ++period) {
      const int start = scheduleParseMinute(previous.periods[period].start);
      const int end = scheduleParseMinute(previous.periods[period].end);
      if (start >= 0 && end >= 0 && end < start && currentMinute <= end) {
        return &previous.periods[period];
      }
    }
  }

  return nullptr;
}
