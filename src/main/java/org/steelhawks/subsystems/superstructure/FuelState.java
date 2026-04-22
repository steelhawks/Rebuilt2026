package org.steelhawks.subsystems.superstructure;

public enum FuelState {
    NONE(false),
    EMPTY(false),
    INTAKING(false),
    HOLDING(true),
    INDEXING(false),
    SHOOTING(false),
    FERRY(false);

    final boolean hasBalls;

    FuelState(boolean hasBalls) {
        this.hasBalls = hasBalls;
    }
}
