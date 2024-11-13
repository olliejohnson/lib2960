/**
 * Copyright 2024 Ryan Fitz-Gerald
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the “Software”), to 
 * deal in the Software without restriction, including without limitation the 
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or 
 * sell copies of the Software, and to permit persons to whom the Software is 
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in 
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
 * DEALINGS IN THE SOFTWARE.
 */

package ftc.lib2960.subsystems;

import frc.lib2960.util.*;
import frc.lib2960.controllers.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.networktables.*;

import java.util.Optional;

/**
 * Manages a one single game piece stage of an intake system
 */
public abstract class IntakeStageBase extends SubsystemBase {
    /**
     * Settings Object
     */
    public class Settings {
        public final double full_speed;
        /** < Full speed voltage of the intake stage */
        public final double slow_speed;

        /** < Slow speed voltage of the intake stage */

        /**
         * Constructor
         * 
         * @param full_speed Full speed voltage of the intake stage
         * @param slow_speed Slow speed voltage of the intake stage
         */
        public Settings(double full_speed, double slow_speed) {
            this.full_speed = full_speed;
            this.slow_speed = slow_speed;
        }
    }

    /**
     * Automatically runs the intake stage until a game piece is at the outfeed
     */
    public class AutoCommand extends Command {
        private final IntakeStageBase intake;

        /** < Instance of the intake stage */

        /**
         * Constructor
         * 
         * @param intake instance of the intake stage
         */
        public AutoCommand(IntakeStageBase intake) {
            this.intake = intake;

            addRequirements(intake);
        }

        /**
         * Runs intake if there is a game piece infeed is ready and turns it off when a
         * part is at the outfeed sensor. Fast speed is used if a game piece is not at
         * the the slowdown sensor, if no slowdown sensor is provided, or the outfeed is
         * clear.
         */
        @Override
        public void execute() {
            // Get current state of sensors
            boolean infeed_ready = atInfeedReady();
            boolean outfeed_ready = isOutfeedReady();
            boolean at_outfeed = atOutfeedStop();
            Optional<Boolean> at_slowdown = atSlowDown();

            // Check run state of the stage
            boolean run_stage = !at_outfeed && infeed_ready;
            boolean slow_down = at_slowdown.isPresent() &&
                    at_slowdown.get().booleanValue() && !outfeed_ready;

            // Set the motor voltage
            double voltage = 0;

            if (run_stage) {
                voltage = intake.settings.full_speed;

                if (slow_down)
                    voltage = intake.settings.slow_speed;
            }

            intake.setVoltage(voltage);
        }

        /**
         * Ends command if game piece is at the outfeed sensor and the infeed sensor is
         * not present
         * 
         * @return true if the game piece is at the outfeed sensor and the infeed sensor
         *         is not
         *         present
         */
        @Override
        public boolean isFinished() {
            return atOutfeedSensor() && !at_infeed.isPresent();
        }
    }

    /**
     * Sets manual control over the intake stage
     */
    public class ManualCommand extends command {
        private final IntakeStageBase intake;
        /** < Instance of the intake stage */
        private boolean is_forward;
        /** < Intake forward flag */
        private boolean is_slowdown;

        /** < Intake slowdown flag */

        /**
         * Constructor.
         * 
         * - is_forward is set to true
         * - is_slowdown is set to false
         * 
         * @param intake instance of the intake stage
         */
        public ManualCommand(IntakeStageBase intake) {
            this.intake = intake;
            this.is_forward = true;
            this.is_slowdown = false;

            addRequirements(intake);
        }

        /**
         * Constructor
         * 
         * @param intake      instance of the intake stage
         * @param is_forward  Runs intake forward if true, reverse if false
         * @param is_slowdown Runs intake at slowdown speed if true, at full speed if
         *                    false
         */
        public ManualCommand(IntakeStageBase intake, boolean is_forward, boolean is_slowdown) {
            this.intake = intake;
            this.is_forward = is_forward;
            this.is_slowdown = is_slowdown;
        }

        /**
         * Runts the intake manually
         */
        @Override
        public void execute() {
            double voltage = is_slowdown ? intake.settings.slow_speed : intake.settings.full_speed;
            voltage *= is_forward ? 1 : -1;
            intake.setVoltage(voltage);
        }

        /**
         * Sets the forward state of the command
         * 
         * @param is_forward Runs intake forward if true, reverse if false
         */
        public void setForward(boolean is_forward) {
            this.is_forward = is_forward;
        }

        /**
         * Sets the slowdown state of the command
         * 
         * @param is_slowdown Runs intake at slowdown speed if true, at full speed if
         *                    false
         */
        public void setSlowdown(boolean is_slowdown) {
            this.is_slowdown = is_slowdown;
        }
    }

    public final Settings settings;
    /** < settings object */

    private final AutoCommand auto_cmd;
    /** < Internal automatic command */
    private final ManualCommand manual_cmd;

    /** < Internal manual command */

    /**
     * Constructor
     * 
     * @param settings
     */
    public IntakeStageBase(Settings settings) {
        this.settings = settings;

        auto_cmd = new AutoCommand(this);
        manual_cmd = new ManualCommand(this);

        setDefaultCommand(auto_cmd);
    }

    /**
     * Runs intake automatically
     */
    public void runAuto() {
        Command current_cmd = getCurrentCommand();
        if (current_cmd != getDefaultCommand())
            current_cmd.cancel();
    }

    /**
     * Runs intake manually
     * 
     * @param is_forward  Runs intake forward if true, reverse if false
     * @param is_slowdown Runs intake at slowdown speed if true, at full speed if
     *                    false
     */
    public void runManual(boolean is_forward, boolean is_slowdown) {
        manual_cmd.setForward(is_forward, is_slowdown);
        if (getCurrentCommand() != manual_cmd)
            manual_cmd.schedule();
    }

    /**
     * Generates an automatic command for the intake stage
     * 
     * @return new automatic command for the intake stage
     */
    public AutoCommand getAutoCommand() {
        return new AutoCommand(this);
    }

    /**
     * Generates a manual command for the intake stage
     * 
     * @param is_forward  Runs intake forward if true, reverse if false
     * @param is_slowdown Runs intake at slowdown speed if true, at full speed if
     *                    false
     * @return new manual command for the intake stage
     */
    public ManualCommand getManualCommand(boolean is_forward, boolean is_slowdown) {
        return new ManualCommand(this, is_forward, is_slowdown);
    }

    /**
     * Runs the subsystem's periodic operations
     */
    @Override
    public void periodic() {
        updateUI();
    }

    /**
     * Updates the subsystem's Shuffleboard Outputs
     */
    private void updateUI() {
        // TODO: Implement
    }

    // Abstract Methods
    /**
     * Checks if the intake stage is ready to infeed a game piece
     * 
     * @return true if ready to infeed, false otherwise
     */
    public abstract boolean isInfeedReady();

    /**
     * Checks if the intake stage is ready to outfeed a game piece
     * 
     * @return true if ready to outfeed, false otherwise
     */
    public abstract boolean isOutfeedReady();

    /**
     * Checks if a game piece is at the outfeed stop position
     * 
     * @return true if a game piece is at the outfeed stop position, false otherwise
     */
    public abstract boolean atOutfeedStop();

    /**
     * Checks if a game piece is at the slowdown position of the intake stage
     * 
     * @return true if a game piece is at the outfeed stop position, false
     *         otherwise. Returns empty if slowdown condition is not provided.
     */
    public Optional<Boolean> atSlowDown() {
        return Optional.empty();
    }

    /**
     * Gets the current voltage of the intake stage motor
     * @return  current voltage of the intake stage motor
     */
    public abstract double getVoltage();

    /**
     * Gets the current current of the intake stage motor
     * @return  current current of the intake stage motor
     */
    public abstract double getCurrent();

    /**
     * Sets the desired output voltage of the intake stage motor
     * @param voltage   desired output voltage of the intake stage motor
     */
    public abstract void setVoltage(double voltage);
}