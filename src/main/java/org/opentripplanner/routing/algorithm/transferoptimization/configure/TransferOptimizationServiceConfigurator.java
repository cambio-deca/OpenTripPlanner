package org.opentripplanner.routing.algorithm.transferoptimization.configure;

import java.util.List;
import java.util.Optional;
import java.util.function.IntFunction;
import javax.annotation.Nullable;
import org.opentripplanner.model.transfer.TransferService;
import org.opentripplanner.raptor.api.model.RaptorTripSchedule;
import org.opentripplanner.raptor.api.path.RaptorStopNameResolver;
import org.opentripplanner.raptor.api.request.PassThroughPoint;
import org.opentripplanner.raptor.spi.RaptorCostCalculator;
import org.opentripplanner.raptor.spi.RaptorTransitDataProvider;
import org.opentripplanner.routing.algorithm.transferoptimization.OptimizeTransferService;
import org.opentripplanner.routing.algorithm.transferoptimization.api.TransferFilter;
import org.opentripplanner.routing.algorithm.transferoptimization.api.TransferOptimizationParameters;
import org.opentripplanner.routing.algorithm.transferoptimization.model.MinCostFilterChain;
import org.opentripplanner.routing.algorithm.transferoptimization.model.MinSafeTransferTimeCalculator;
import org.opentripplanner.routing.algorithm.transferoptimization.model.OptimizedPathTail;
import org.opentripplanner.routing.algorithm.transferoptimization.model.PassThroughPointFilter;
import org.opentripplanner.routing.algorithm.transferoptimization.model.TransferWaitTimeCostCalculator;
import org.opentripplanner.routing.algorithm.transferoptimization.services.OptimizePathDomainService;
import org.opentripplanner.routing.algorithm.transferoptimization.services.TransferGenerator;
import org.opentripplanner.routing.algorithm.transferoptimization.services.TransferOptimizedFilterFactory;
import org.opentripplanner.routing.algorithm.transferoptimization.services.TransferServiceAdaptor;
import org.opentripplanner.transit.model.site.StopLocation;

/**
 * Responsible for assembly of the prioritized-transfer services.
 */
public class TransferOptimizationServiceConfigurator<T extends RaptorTripSchedule> {

  private final IntFunction<StopLocation> stopLookup;
  private final RaptorStopNameResolver stopNameResolver;
  private final TransferService transferService;
  private final RaptorTransitDataProvider<T> transitDataProvider;

  @Nullable
  private final List<PassThroughPoint> passThroughPoints;

  private final int[] stopBoardAlightCosts;
  private final TransferOptimizationParameters config;

  private TransferOptimizationServiceConfigurator(
    IntFunction<StopLocation> stopLookup,
    RaptorStopNameResolver stopNameResolver,
    TransferService transferService,
    RaptorTransitDataProvider<T> transitDataProvider,
    List<PassThroughPoint> passThroughPoints,
    int[] stopBoardAlightCosts,
    TransferOptimizationParameters config
  ) {
    this.stopLookup = stopLookup;
    this.stopNameResolver = stopNameResolver;
    this.transferService = transferService;
    this.transitDataProvider = transitDataProvider;
    this.passThroughPoints = passThroughPoints;
    this.stopBoardAlightCosts = stopBoardAlightCosts;
    this.config = config;
  }

  /**
   * Scope: Request
   */
  public static <
    T extends RaptorTripSchedule
  > OptimizeTransferService<T> createOptimizeTransferService(
    IntFunction<StopLocation> stopLookup,
    RaptorStopNameResolver stopNameResolver,
    TransferService transferService,
    RaptorTransitDataProvider<T> transitDataProvider,
    @Nullable List<PassThroughPoint> passThroughPoints,
    int[] stopBoardAlightCosts,
    TransferOptimizationParameters config
  ) {
    return new TransferOptimizationServiceConfigurator<T>(
      stopLookup,
      stopNameResolver,
      transferService,
      transitDataProvider,
      passThroughPoints,
      stopBoardAlightCosts,
      config
    )
      .createOptimizeTransferService();
  }

  private OptimizeTransferService<T> createOptimizeTransferService() {
    var pathTransferGenerator = createTransferGenerator(config.optimizeTransferPriority());
    var filter = createTransferOptimizedFilter(
      config.optimizeTransferPriority(),
      config.optimizeTransferWaitTime()
    );

    if (config.optimizeTransferWaitTime()) {
      var transferWaitTimeCalculator = createTransferWaitTimeCalculator();

      var transfersPermutationService = createOptimizePathService(
        pathTransferGenerator,
        filter,
        transferWaitTimeCalculator,
        transitDataProvider.multiCriteriaCostCalculator()
      );

      return new OptimizeTransferService<>(
        transfersPermutationService,
        createMinSafeTxTimeService(),
        transferWaitTimeCalculator
      );
    } else {
      var transfersPermutationService = createOptimizePathService(
        pathTransferGenerator,
        filter,
        null,
        transitDataProvider.multiCriteriaCostCalculator()
      );
      return new OptimizeTransferService<>(transfersPermutationService);
    }
  }

  private OptimizePathDomainService<T> createOptimizePathService(
    TransferGenerator<T> transferGenerator,
    MinCostFilterChain<OptimizedPathTail<T>> transferPointFilter,
    TransferWaitTimeCostCalculator transferWaitTimeCostCalculator,
    RaptorCostCalculator<T> costCalculator
  ) {
    return new OptimizePathDomainService<>(
      transferGenerator,
      costCalculator,
      transitDataProvider.slackProvider(),
      transferWaitTimeCostCalculator,
      createTransferFilter(),
      stopBoardAlightCosts,
      config.extraStopBoardAlightCostsFactor(),
      transferPointFilter,
      stopNameResolver
    );
  }

  private TransferFilter<T> createTransferFilter() {
    if (passThroughPoints == null) {
      return null;
    } else {
      return new PassThroughPointFilter<>(passThroughPoints);
    }
  }

  private MinSafeTransferTimeCalculator<T> createMinSafeTxTimeService() {
    return new MinSafeTransferTimeCalculator<>(transitDataProvider.slackProvider());
  }

  private TransferGenerator<T> createTransferGenerator(boolean transferPriority) {
    var transferServiceAdaptor = (transferService != null && transferPriority)
      ? TransferServiceAdaptor.<T>create(stopLookup, transferService)
      : TransferServiceAdaptor.<T>noop();

    return new TransferGenerator<>(transferServiceAdaptor, transitDataProvider);
  }

  private TransferWaitTimeCostCalculator createTransferWaitTimeCalculator() {
    return new TransferWaitTimeCostCalculator(
      config.backTravelWaitTimeFactor(),
      config.minSafeWaitTimeFactor()
    );
  }

  private MinCostFilterChain<OptimizedPathTail<T>> createTransferOptimizedFilter(
    boolean transferPriority,
    boolean optimizeWaitTime
  ) {
    return TransferOptimizedFilterFactory.filter(transferPriority, optimizeWaitTime);
  }
}
