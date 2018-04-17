package org.firstinspires.ftc.teamcode;

/**
 * Created by arulgupta on 3/16/18.
 */

public class ServoValues {

    //                           JEWEL SYSTEM

        //  Minimum Positions
        public static final double MIN_POSITION_DOWN = 0;
        public static final double MIN_POSITION_FLICK = 0;

        //  Maximum Positions
        public static final double MAX_POSITION_DOWN = 1;
        public static final double MAX_POSITION_FLICK = 1;

        //  Initial Positions
        public static final double START_POSITION_DOWN = 0.87;
        public static final double START_POSITION_FLICK = 0.3;

        //  Significant Positions

            //  Centered Positions
            public static final double CENTER_POSITION_DOWN = 0.87;
            public static final double CENTER_POSITION_FLICK = 0.37;

            //  Flick Positions
            public static final double LOW_POSITION_DOWN = 0.45;
            public static final double RIGHT_POSITION_FLICK = CENTER_POSITION_FLICK - 0.36;
            public static final double LEFT_POSITION_FLICK = CENTER_POSITION_FLICK + 0.36;


            //  Increments
            public static final double INCREMENT_POSITION_DOWN = 0.01;
            public static final long INCREMENT_FREQUENCY_DOWN = 50;

    //--------------Glyph System------------------

        //  Minimum Positions
        public static final double MIN_POSITION_PULL = 0;

        //  Maximum Positions
        public static final double MAX_POSITION_PULL = 1;

        //  Initial Positions
        public static final double START_POSITION_PULL = 1;

        //  Significant Positions
        public static final double LOW_POSITION_PULL = 0.4;
        public static final double HIGH_POSITION_PULL = 0.5;


    //-------------AutoGlyph System-----------------
        //  Minimum Positions
        public static final double MIN_POSITION_RGRAB = 0.2;
        public static final double MIN_POSITION_LGRAB = 0.4;

        //  Maximum Positions
        public static final double MAX_POSITION_RGRAB = 1;
        public static final double MAX_POSITION_LGRAB = 0.98;

        //  Initial Positions
        public static final double START_POSITION_RGRAB = 0;
        public static final double START_POSITION_LGRAB = 1;

        //  Significant Positions
            // Grab Positions
            public static final double RGRAB_POSITION = 0.2;
            public static final double LGRAB_POSITION = 0.6;
            // Ready to Grab Positions
            public static final double R_READY_POSITION = 0.5;
            public static final double L_READY_POSITION = 0;


    //--------------Relic System------------------

        //  Minimum Positions
        public static final double MIN_POSITION_CLAW = 0;
        public static final double MIN_POSITION_WRIST = 0;

        //  Maximum Positions
        public static final double MAX_POSITION_CLAW = 1;
        public static final double MAX_POSITION_WRIST = 1;

        //  Initial Positions
        public static final double START_POSITION_CLAW = 0.7;
        public static final double START_POSITION_WRIST = 0.8;

        //  Significant Positions

            //  Grab/Pick Positions
            public static final double GRAB_POSITION_CLAW = 0;
            public static final double OPEN_POSITION_CLAW = 0;

            public static final double LOW_POSITION_WRIST = 0;
            public static final double HIGH_POSITION_WRIST = 0;

            //  Increments
            public static final double INCREMENT_POSITION_CLAW = 0.01;
            public static final double INCREMENT_POSITION_WRIST = 0.01;


}
