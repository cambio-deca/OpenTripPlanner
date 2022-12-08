package org.opentripplanner.openstreetmap.wayproperty;

/**
 * Record that holds forward and back safety factors for cycling or walking.
 */
public record SafetyFeatures(Double forward, Double back) {}