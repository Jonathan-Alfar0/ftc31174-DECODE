package org.firstinspires.ftc.teamcode.shooterConstants;

/**
 * ShooterConstants
 *
 * Provides dynamically calculated control values for the shooter and hood
 * based on a table of calibrated distance-to-RPM/hood-position points.
 * Values are linearly interpolated between the calibrated data points.
 */
public final class ShooterConstants {

    // Master table of calibration points.
    // Each entry is: {distance (in), rpm, hoodPosition}
    private static final double[][] SHOOTER_DATA = {
            // --- Big Close Triangle ---
            {33, 2480, 0.151},
            {34, 2480, 0.151},
            {35, 2480, 0.151},
            {36, 2485, 0.151},
            {37, 2485, 0.151},
            {38, 2485,0.151},
            {39, 2485, 0.151},
            {40, 2485, 0.151},
            {41, 2485, 0.151},
            {42, 2485,0.151},
            {43, 2485, 0.151},
            {44, 2485,0.151},
            {45, 2485, 0.151},
            {46, 2485, 0.151},
            {47, 2485, 0.151},
            {48, 2485, 0.151},
            {49, 2485, 0.151},
            {50, 2485, 0.151},
            {51, 2490, 0.151},
            {52, 2490, 0.151},
            {53, 2490, 0.151},
            {54, 2490, 0.151},
            {55, 2490, 0.151},
            {56, 2490, 0.151},
            {57, 2490, 0.151},
            {58, 2490, 0.151},
            {59, 2495, 0.151},
            {60, 2495, 0.151},
            {61, 2495, 0.151},
            {62, 2495, 0.151},
            {63, 2490, 0.151},
            {64, 2500, 0.151},
            {65, 2500, 0.151},
            {66, 2500, 0.151},
            {67, 2500, 0.151},
            {68, 2500, 0.151},
            {69, 2500, 0.151},
            {70, 2510, 0.151},
            {71, 2510, 0.151},
            {72, 2510, 0.157},
            {73, 2510, 0.157},
            {74, 2510, 0.157},
            {75, 2510, 0.157},
            {76, 2510, 0.157},
            {77, 2510, 0.157},
            {78, 2510, 0.157},
            {79, 2510, 0.157},
            {80, 2510, 0.157},
            {81, 2510, 0.157},
            {82, 2510, 0.157},
            {83, 2550, 0.157},
            {84, 2550, 0.157},
            {85, 2600, 0.157},
            {86, 2600, 0.157},
            {87, 2600, 0.157},
            {88, 2600, 0.157},
            {89, 2650, 0.157},
            {90, 2650, 0.157},
            {91, 2650, 0.157},
            {92, 2650, 0.157},

            // --- Middle Points (Cant Shoot) for better accuracy ---
            {93, 2650, 0.157},
            {94, 2650, 0.157},
            {95, 2650, 0.157},
            {96, 2650, 0.157},
            {97, 2650, 0.157},
            {98, 2650, 0.157},
            {99, 2650, 0.163},
            {100, 2720, 0.163},
            {101, 2720, 0.163},
            {102, 2720, 0.163},
            {103, 2720, 0.163},
            {104, 2720, 0.163},
            {105, 2720, 0.163},
            {106, 2720, 0.163},
            {107, 2720, 0.163},
            {108, 2720, 0.163},
            {109, 2720, 0.163},
            {110, 2720, 0.163},
            {111, 2720, 0.163},
            {112, 2755, 0.163},
            {113, 2755, 0.163},
            {114, 2755, 0.163},
            {115, 2775, 0.163},
            {116, 2880, 0.163},
            {117, 2900, 0.163},
            {118, 3000, 0.163},

            // --- Back Small Triangle ---
            {119, 3100, 0.175},
            {120, 3100, 0.175},
            {121, 3100, 0.175},
            {122, 3100, 0.175},
            {123, 3100, 0.175},
            {124, 3100, 0.175},
            {125, 3100, 0.175},
            {126, 3100, 0.175},
            {127, 3100, 0.175},
            {128, 3100, 0.175},
            {129, 3100, 0.175},
            {130, 3100, 0.175},
            {131, 3100, 0.175},
            {132, 3100, 0.175},
            {133, 3100, 0.175},
            {134, 3100, 0.175},
            {135, 3100, 0.175},
            {136, 3100, 0.175},
            {137, 3100, 0.175},
            {138, 3100, 0.175},
            {139, 3100, 0.175},
            {140, 3100, 0.175},
            {141, 3100, 0.175},
            {142, 3100, 0.175},
            {143, 3100, 0.175},
            {144, 3100, 0.175},
            {145, 3100, 0.175},
            {146, 3100, 0.175},
            {147, 3175, 0.175},
            {148, 3180, 0.175},
            {149, 3180, 0.175},
            {150, 3180, 0.175},
            {151, 3185, 0.175},
            {152, 3185, 0.175},
            {153, 3185, 0.175},
            {154, 3185, 0.175},
            {155, 3185, 0.175},
            {156, 3185, 0.175},
            {157, 3205, 0.175},
            {158, 3205, 0.175},
            {159, 3205, 0.175},
            {160, 3205, 0.175},
            {161, 3220, 0.175},
            {162, 3220, 0.175}
    };

    private ShooterConstants() {}

    /**
     * Calculates the target hood position based on the distance to the target.
     * It finds the nearest entry in the SHOOTER_DATA table to determine the hood position.
     * This is designed to work with step-like changes in hood position from the table.
     * @param distance The distance to the target in inches.
     * @return The calculated hood servo position.
     */
    public static double hoodPosition(double distance) {
        // Handle distances outside the calibrated range
        if (distance <= SHOOTER_DATA[0][0]) {
            return SHOOTER_DATA[0][2];
        }
        if (distance >= SHOOTER_DATA[SHOOTER_DATA.length - 1][0]) {
            return SHOOTER_DATA[SHOOTER_DATA.length - 1][2];
        }

        // Find the two data points that bracket the current distance
        int i = 0;
        while (i < SHOOTER_DATA.length - 2 && distance > SHOOTER_DATA[i + 1][0]) {
            i++;
        }

        double d1 = SHOOTER_DATA[i][0];
        double h1 = SHOOTER_DATA[i][2];
        double d2 = SHOOTER_DATA[i + 1][0];
        double h2 = SHOOTER_DATA[i + 1][2];

        // If hood positions are the same, no need to check which is closer
        if (h1 == h2) {
            return h1;
        }

        // Return the hood position of the nearest distance entry
        if ((distance - d1) < (d2 - distance)) {
            return h1;
        } else {
            return h2;
        }
    }

    /**
     * Calculates the target shooter RPM based on the distance to the target.
     * It linearly interpolates between the points defined in the SHOOTER_DATA table.
     * For distances outside the table or in the gap, it uses the RPM of the nearest data point.
     * @param distance The distance to the target in inches.
     * @return The calculated target RPM.
     */
    public static double targetRPM(double distance) {
        // Handle distances outside the calibrated range
        if (distance <= SHOOTER_DATA[0][0]) {
            return SHOOTER_DATA[0][1];
        }
        if (distance >= SHOOTER_DATA[SHOOTER_DATA.length - 1][0]) {
            return SHOOTER_DATA[SHOOTER_DATA.length - 1][1];
        }

        // Find the two data points that bracket the current distance
        int i = 0;
        while (i < SHOOTER_DATA.length - 2 && distance > SHOOTER_DATA[i + 1][0]) {
            i++;
        }

        double d1 = SHOOTER_DATA[i][0];
        double r1 = SHOOTER_DATA[i][1];
        double d2 = SHOOTER_DATA[i + 1][0];
        double r2 = SHOOTER_DATA[i + 1][1];

        // Check if the distance falls in the gap between the two triangles (index 7 to 8)
        if (i == 7 && distance > d1 && distance < d2) {
            // Return the RPM of the closer endpoint of the gap
            return (distance - d1 < d2 - distance) ? r1 : r2;
        }

        // Perform linear interpolation
        return interpolate(distance, d1, d2, r1, r2);
    }

    /**
     * Performs linear interpolation between two points.
     */
    private static double interpolate(double x, double x1, double x2, double y1, double y2) {
        if (x1 == x2) return y1;
        return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
    }
}
