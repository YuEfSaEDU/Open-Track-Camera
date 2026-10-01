package com.opentrack.camera.detect

/** What the neural detector may lock on to (COCO classes of the bundled model). */
enum class TargetClasses(val labels: Set<String>) {
    /** Aircraft; helicopters are usually detected as "airplane" too (the model has no helicopter class). */
    AIRCRAFT(setOf("airplane")),

    /** Aircraft plus land and water vehicles (runway, road and harbour shots). */
    VEHICLES(setOf("airplane", "car", "truck", "bus", "motorcycle", "train", "boat")),
}
