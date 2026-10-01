package com.opentrack.camera.detect

/** What the neural detector may lock on to (COCO classes of the bundled model). */
enum class TargetClasses(val labels: Set<String>) {
    /** Aircraft: airplanes, helicopters and drones (the COCO model knows only "airplane"). */
    AIRCRAFT(setOf("airplane", "helicopter", "drone")),

    /** Aircraft plus land and water vehicles (runway, road and harbour shots). */
    VEHICLES(setOf("airplane", "car", "truck", "bus", "motorcycle", "train", "boat")),
}
