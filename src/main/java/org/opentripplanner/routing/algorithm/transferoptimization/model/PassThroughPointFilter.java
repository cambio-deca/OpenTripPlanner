package org.opentripplanner.routing.algorithm.transferoptimization.model;

import java.util.ArrayList;
import java.util.BitSet;
import java.util.List;
import org.opentripplanner.framework.tostring.ToStringBuilder;
import org.opentripplanner.raptor.api.model.RaptorTripPattern;
import org.opentripplanner.raptor.api.model.RaptorTripSchedule;
import org.opentripplanner.raptor.api.request.PassThroughPoint;
import org.opentripplanner.routing.algorithm.transferoptimization.api.TransferFilter;

public class PassThroughPointFilter<T extends RaptorTripSchedule> implements TransferFilter<T> {

  private static final int NOT_SET = -999;

  private final List<PassThroughPoint> passThroughPoints;

  public PassThroughPointFilter(List<PassThroughPoint> passThroughPoints) {
    this.passThroughPoints = passThroughPoints;
  }

  @Override
  public List<List<TripToTripTransfer<T>>> filterTransfers(
    int boardStopPosFirstTrip,
    int alightStopPosLastTrip,
    List<List<TripToTripTransfer<T>>> originalTransfers
  ) {
    var newTransfers = new ArrayList<List<TripToTripTransfer<T>>>();
    var tripSegments = findTripSegments(
      boardStopPosFirstTrip,
      alightStopPosLastTrip,
      originalTransfers
    );

    /**
     *     A     B     C
     *  ---*-----*-----*-----(*)--*
     *     *----(*)----*------*---*----*----*------*
     *
     *
     */



    var passThroughPointIterator = this.passThroughPoints.iterator();
    var segmentIterator = tripSegments.iterator();
    var transfersIterator = originalTransfers.iterator();

    var passThroughPoint = passThroughPointIterator.next();
    var toSegment = segmentIterator.next();
    toSegment.decorateWithPassThroughPoints(passThroughPoint);

    while (segmentIterator.hasNext() && transfersIterator.hasNext()) {
      var transfers = transfersIterator.next();
      var fromSegment = toSegment;
      toSegment = segmentIterator.next();
      toSegment.decorateWithPassThroughPoints(passThroughPoint);

      transfers = filterTransfers(transfers, fromSegment, toSegment);

      if (fromSegment.hasPassThroughPoint && passThroughPointIterator.hasNext()) {
        passThroughPoint = passThroughPointIterator.next();
        fromSegment.decorateWithPassThroughPoints(passThroughPoint);
        toSegment.decorateWithPassThroughPoints(passThroughPoint);
        transfers = filterTransfers(transfers, fromSegment, toSegment);
      }
      newTransfers.add(transfers);
    }
    return newTransfers;
  }

  private List<TripToTripTransfer<T>> filterTransfers(
    List<TripToTripTransfer<T>> transfers,
    TripSegment<T> fromSegment,
    TripSegment<T> toSegment
  ) {
    if (!fromSegment.hasNoPassThroughPoint() && !toSegment.hasNoPassThroughPoint()) {
      return transfers;
    }
    var result = new ArrayList<TripToTripTransfer<T>>();
    int firstFromStopPos = fromSegment.firstPossibleAlightStopPos();
    int lastToStopPos = fromSegment.lastPossibleBoardStopPos();

    for (var it : transfers) {
      if (it.from().stopPosition() >= firstFromStopPos && it.to().stopPosition() <= lastToStopPos) {
        result.add(it);
      }
    }
    return result;
  }

  private List<TripSegment<T>> findTripSegments(
    int boardStopPosFirstTrip,
    int alightStopPosLastTrip,
    List<List<TripToTripTransfer<T>>> originalTransfers
  ) {
    int firstBoardStop = boardStopPosFirstTrip;
    int nextTripFirstBoardStop = Integer.MAX_VALUE;
    int lastAlightStop = Integer.MIN_VALUE;
    var list = new ArrayList<TripSegment<T>>();

    for (List<TripToTripTransfer<T>> transfers : originalTransfers) {
      T trip = transfers.get(0).from().trip();
      for (TripToTripTransfer<T> tx : transfers) {
        lastAlightStop = Math.max(lastAlightStop, tx.from().stopPosition());
        nextTripFirstBoardStop = Math.min(nextTripFirstBoardStop, tx.to().stopPosition());
      }
      list.add(new TripSegment<>(trip, firstBoardStop, lastAlightStop));
      firstBoardStop = nextTripFirstBoardStop;
    }
    list.add(
      new TripSegment<>(
        originalTransfers.get(originalTransfers.size() - 1).get(0).to().trip(),
        nextTripFirstBoardStop,
        alightStopPosLastTrip
      )
    );
    return list;
  }

  private static class TripSegment<T extends RaptorTripSchedule> {

    private final T trip;
    private final int fromStopPos;
    private final int toStopPos;
    private int firstPossibleAlightStopPos = -1;
    private int lastPossibleBoardStopPos = -1;
    private boolean hasPassThroughPoint = false;

    private final BitSet ptStops;

    TripSegment(T trip, int fromStopPos, int toStopPos) {
      this.trip = trip;
      this.fromStopPos = fromStopPos;
      this.toStopPos = toStopPos;
      this.ptStops = new BitSet(fromStopPos - toStopPos + 1);
    }

    void decorateWithPassThroughPoints(PassThroughPoint passThroughPoint) {
      this.hasPassThroughPoint = false;
      firstPossibleAlightStopPos = 10_000_000;
      lastPossibleBoardStopPos = -10_000_000;
      ptStops.clear();
      RaptorTripPattern pattern = trip.pattern();

      for (int pos = fromStopPos; pos < toStopPos; ++pos) {
        if (passThroughPoint.contains(pattern.stopIndex(pos))) {
          this.hasPassThroughPoint = true;
          this.firstPossibleAlightStopPos = Math.min(firstPossibleAlightStopPos, pos);
          this.lastPossibleBoardStopPos = Math.max(lastPossibleBoardStopPos, pos);
        }
      }

      // Include all stops, ife trip does not contain any pass-through points
      if (!hasPassThroughPoint) {
        firstPossibleAlightStopPos = fromStopPos;
        lastPossibleBoardStopPos = toStopPos;
      }
    }

    boolean hasNoPassThroughPoint() {
      return lastPossibleBoardStopPos == NOT_SET;
    }

    @Override
    public String toString() {
      return ToStringBuilder
        .of(TripSegment.class)
        .addStr("trip", trip.pattern().debugInfo())
        .addNum("fromStopPos=", fromStopPos)
        .addNum("toStopPos=", toStopPos)
        .addObj("ptStopIndexes", ptStops)
        .toString();
    }

    int firstPossibleAlightStopPos() {
      return firstPossibleAlightStopPos;
    }

    int lastPossibleBoardStopPos() {
      return lastPossibleBoardStopPos;
    }
  }
}
