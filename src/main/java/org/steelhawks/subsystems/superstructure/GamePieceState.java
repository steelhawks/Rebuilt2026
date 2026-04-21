package org.steelhawks.subsystems.superstructure;

public enum GamePieceState {
    EMPTY(false),
    INTAKING(false),
    HOLDING(true),
    INDEXING(false),
    SHOOTING(false),
    FERRY(false);

    boolean hasBalls;

    GamePieceState(boolean hasBalls) {
        this.hasBalls = hasBalls;
    }
}
