package org.opentripplanner.routing.algorithm.transferoptimization.api;

import java.util.List;
import org.opentripplanner.raptor.api.model.RaptorTripSchedule;
import org.opentripplanner.routing.algorithm.transferoptimization.model.TripToTripTransfer;

public interface TransferFilter<T extends RaptorTripSchedule> {
  List<List<TripToTripTransfer<T>>> filterTransfers(
    int boardStopPosFirstTrip,
    int alightStopPosLastTrip,
    List<List<TripToTripTransfer<T>>> possibleTransfers
  );
}
