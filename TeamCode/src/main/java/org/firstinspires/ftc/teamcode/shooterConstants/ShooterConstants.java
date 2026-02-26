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
            {33, 2480, 0.157},
            {34, 2480, 0.157},
            {35, 2480, 0.157},
            {36, 2485, 0.157},
            {37, 2485, 0.157},
            {38, 2485,0.157},
            {39, 2485, 0.157},
            {40, 2485, 0.157},
            {41, 2485, 0.157},
            {42, 2485,0.157},
            {43, 2485, 0.157},
            {44, 2485,0.157},
            {45, 2485, 0.157},
            {46, 2485, 0.157},
            {47, 2485, 0.157},
            {48, 2485, 0.157},
            {49, 2485, 0.157},
            {50, 2485, 0.157},
            {51, 2490, 0.157},
            {52, 2490, 0.157},
            {53, 2490, 0.157},
            {54, 2490, 0.157},
            {55, 2490, 0.157},
            {56, 2490, 0.157},
            {57, 2490, 0.157},
            {58, 2490, 0.157},
            {59, 2495, 0.157},
            {60, 2495, 0.157},
            {61, 2495, 0.157},
            {62, 2495, 0.157},
            {63, 2490, 0.157},
            {64, 2500, 0.157},
            {65, 2500, 0.157},
            {66, 2500, 0.157},
            {67, 2500, 0.157},
            {68, 2500, 0.157},
            {69, 2500, 0.157},
            {70, 2510, 0.157},
            {71, 2510, 0.157},
            {72, 2510, 0.163},
            {73, 2510, 0.163},
            {74, 2510, 0.163},
            {75, 2510, 0.163},
            {76, 2510, 0.163},
            {77, 2510, 0.163},
            {78, 2510, 0.163},
            {79, 2510, 0.163},
            {80, 2510, 0.163},
            {81, 2510, 0.163},
            {82, 2510, 0.163},
            {83, 2550, 0.163},
            {84, 2550, 0.163},
            {85, 2600, 0.163},
            {86, 2600, 0.163},
            {87, 2600, 0.163},
            {88, 2600, 0.163},
            {89, 2600, 0.163},
            {90, 2600, 0.163},
            {91, 2600, 0.163},
            {92, 2600, 0.163},

            // --- Middle Points (Cant Shoot) for better accuracy ---
            {93, 2650, 0.163},
            {94, 2650, 0.163},
            {95, 2650, 0.163},
            {96, 2650, 0.163},
            {97, 2650, 0.163},
            {98, 2650, 0.163},
            {99, 2650, 0.221},
            {100, 2650, 0.221},
            {101, 2650, 0.221},
            {102, 2700, 0.221},
            {103, 2700, 0.221},
            {104, 2700, 0.221},
            {105, 2700, 0.221},
            {106, 2700, 0.221},
            {107, 2700, 0.221},
            {108, 2700, 0.221},
            {109, 3020, 0.221},
            {110, 3020, 0.221},
            {111, 3020, 0.221},
            {112, 3020, 0.221},
            {113, 3020, 0.221},
            {114, 3020, 0.221},
            {115, 3020, 0.221},
            {116, 3020, 0.221},
            {117, 3020, 0.221},
            {118, 3020, 0.221},

            // --- Back Small Triangle ---
            {119, 3020, 0.221},
            {120, 3020, 0.221},
            {121, 3020, 0.221},
            {122, 3020, 0.221},
            {123, 3020, 0.221},
            {124, 3020, 0.221},
            {125, 3020, 0.221},
            {126, 3020, 0.221},
            {127, 3020, 0.221},
            {128, 3020, 0.221},
            {129, 3020, 0.221},
            {130, 3020, 0.221},
            {131, 3020, 0.199},
            {132, 3020, 0.199},
            {133, 3020, 0.199},
            {134, 3020, 0.199},
            {135, 3020, 0.199},
            {136, 3020, 0.199},
            {137, 3020, 0.199},
            {138, 3020, 0.199},
            {139, 3020, 0.211},
            {140, 3020, 0.211},
            {141, 3020, 0.211},
            {142, 3020, 0.211},
            {143, 3020, 0.211},
            {144, 3020, 0.211},
            {145, 3020, 0.211},
            {146, 3020, 0.211},
            {147, 3020, 0.211},
            {148, 3020, 0.211},
            {149, 3020, 0.211},
            {150, 3020, 0.211},
            {151, 3020, 0.211},
            {152, 3020, 0.211},
            {153, 3020, 0.211},
            {154, 3020, 0.211},
            {155, 3020, 0.211},
            {156, 3050, 0.211},
            {157, 3050, 0.211},
            {158, 3050, 0.211},
            {159, 3050, 0.211},
            {160, 3050, 0.211},
            {161, 3080, 0.211},
            {162, 3080, 0.211}
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
