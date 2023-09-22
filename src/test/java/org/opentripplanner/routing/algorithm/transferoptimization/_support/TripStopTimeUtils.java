package org.opentripplanner.routing.algorithm.transferoptimization._support;

import org.opentripplanner.raptor.api.model.RaptorTripSchedule;
import org.opentripplanner.routing.algorithm.transferoptimization.model.TripStopTime;

public class TripStopTimeUtils {
  public static <T extends RaptorTripSchedule> TripStopTime<T> departure(T trip, int stopIndex) {
    return TripStopTime.departure(trip, trip.pattern().findStopPositionAfter(0, stopIndex));
  }

  public static <T extends RaptorTripSchedule> TripStopTime<T> arrival(T trip, int stopIndex) {
    return TripStopTime.arrival(trip, trip.pattern().findStopPositionAfter(0, stopIndex));
  }
}
