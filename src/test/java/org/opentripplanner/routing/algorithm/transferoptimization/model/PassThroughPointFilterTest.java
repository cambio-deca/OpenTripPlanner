package org.opentripplanner.routing.algorithm.transferoptimization.model;


import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.opentripplanner.raptor._data.transit.TestAccessEgress.walk;
import static org.opentripplanner.raptor._data.transit.TestTransfer.transfer;

import java.util.List;
import org.junit.jupiter.api.Test;
import org.opentripplanner.raptor._data.RaptorTestConstants;
import org.opentripplanner.raptor._data.transit.TestTripSchedule;
import org.opentripplanner.raptor.api.request.PassThroughPoint;
import org.opentripplanner.routing.algorithm.transferoptimization._support.TripStopTimeUtils;


class PassThroughPointFilterTest implements RaptorTestConstants {
  private final TestTripSchedule lineABCD =  TestTripSchedule.schedule()
    .pattern("Line AD", STOP_A, STOP_B, STOP_C, STOP_D)
    .times("10:00 10:10 10:20 10:30").build();
  private final TestTripSchedule lineEFGH = TestTripSchedule.schedule()
    .pattern("Line EH", STOP_E, STOP_F, STOP_G, STOP_H)
    .times("11:00 11:10 11:20 11:30").build();
  private final TestTripSchedule lineIJKL = TestTripSchedule.schedule()
    .pattern("Line IL", STOP_I, STOP_J, STOP_K, STOP_L)
    .times("12:00 12:10 12:20 12:30").build();


  private final TripToTripTransfer<TestTripSchedule> txBE = tx(STOP_B, STOP_E);
  private final TripToTripTransfer<TestTripSchedule> txCF = tx(STOP_C, STOP_F);
  private final TripToTripTransfer<TestTripSchedule> txDG = tx(STOP_D, STOP_G);
  private final TripToTripTransfer<TestTripSchedule> txFI = tx(STOP_F, STOP_I);
  private final TripToTripTransfer<TestTripSchedule> txGJ = tx(STOP_G, STOP_J);
  private final TripToTripTransfer<TestTripSchedule> txHK = tx(STOP_H, STOP_K);

  List<TripToTripTransfer<TestTripSchedule>> transfersLine1to2 = List.of(txBE, txCF, txDG);
  List<TripToTripTransfer<TestTripSchedule>> transfersLine2to3 = List.of(txFI, txGJ, txHK);

  @Test
  void testFilterWithOneTransferPoint() {
    var subject = new PassThroughPointFilter<TestTripSchedule>(
      List.of(new PassThroughPoint(new int[]{ STOP_C }))
    );

    assertEquals(
      "",
      subject.filterTransfers(STOP_A, STOP_H, List.of(transfersLine1to2)).toString()
    );
  }

  private TripToTripTransfer<TestTripSchedule> tx(int fromStop, int toStop) {
    return new TripToTripTransfer<>(
      TripStopTimeUtils.arrival(lineByStop(fromStop), fromStop),
      TripStopTimeUtils.departure(lineByStop(toStop), toStop),
      transfer(toStop, D30s),
      null
    );
  }

  TestTripSchedule lineByStop(int stop) {
    return switch (stop) {
      case STOP_A, STOP_B, STOP_C, STOP_D -> lineABCD;
      case STOP_E, STOP_F, STOP_G, STOP_H -> lineEFGH;
      case STOP_I, STOP_J, STOP_K, STOP_L -> lineIJKL;
      default -> throw new IllegalArgumentException("Unknown stop index: " + stop);
    };
  }

}