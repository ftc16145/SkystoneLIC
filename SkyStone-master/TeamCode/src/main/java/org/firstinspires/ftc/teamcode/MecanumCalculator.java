package org.firstinspires.ftc.teamcode;

import java.util.function.DoubleSupplier;

public class MecanumCalculator {
    double kPX, kPY, kPG, tx, ty, tg;
    public MecanumCalculator( double kPX, double kPY, double kPG ){
        setP( kPX, kPY, kPG );
    }
    public void setP( double kPX, double kPY, double kPG ){
        this.kPX = kPX;
        this.kPY = kPY;
        this.kPG = kPG;
    }

    public void setTarget( double tx, double ty, double tg ){
        this.tx = tx;
        this.ty = ty;
        this.tg = tg;
    }


    // given current = 0
    // If target = +, out = -
    // If robot goes wrong way, negate the kP
    public double[] getError( double xpos, double ypos, double gpos ){
        double ex = xpos - tx;
        double ey = ypos - ty;
        double eg = gpos - tg;
        return new double[]{ ex, ey, eg };

    }

    public double[] getOutput( double xpos, double ypos, double gpos ){
        double[] error = getError( xpos, ypos, gpos );
        double ox = Math.max( Math.min( kPX * error[0], 1 ), -1 );
        double oy = Math.max( Math.min( kPY * error[1], 1 ), -1 );

        if( Math.abs( error[0] ) > 750 ){
            oy = 0;
        }else{
            ox = 0;
        }

        double og = Math.max( Math.min( kPG * error[2], 1 ), -1 );

        return new double[]{ -ox, -oy, -og };
    }

    public boolean[] getDoneIndiv( double xpos, double ypos, double gpos ){
        double[] error = getError( xpos, ypos, gpos );
        boolean dx = Math.abs( error[0] ) < 750;
        boolean dy = Math.abs( error[1] ) < 750;
        boolean dg = Math.abs( error[2] ) < 1;
        return new boolean[]{ dx, dy, dg };
    }

    public boolean getDoneOverall( double xpos, double ypos, double gpos ){
        boolean[] dones = getDoneIndiv( xpos, ypos, gpos );
        return dones[0] && dones[1] && dones[2];
    }

}
