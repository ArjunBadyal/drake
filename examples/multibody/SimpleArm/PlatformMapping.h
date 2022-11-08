//
// Created by arjunbadyal on 24/04/2022.
//
#include <memory>
#include <utility>
#include <cmath>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"


double ind(double &t, double a, double b) {
    if (t >= a && t <= b) {
        return 1;
    } else {
        return 0;
    }
}

/*class GravityComp
{
private:

    double PreTheta(double x)
    {
        double theta =  (-7909 * pow(x,13) + (13439/2)*pow(x,12) + 3153 * pow(x,11))/2500;
        return theta;
    }

public:

    double PreGrav(double t){

        double PreGravComp = -ind(t,0,1) *(2*9.81* sin(PreTheta(t))) -ind(t,1,2) * (2*9.81* sin(PreTheta(2-t)));
        return PreGravComp;
    }

};*/

struct Operation {



    double PrePick(double t) {
//drake::log()->info(t);
        if (t < 2) {
            /*return -ind(t, 0, 1) * pow(t, 10) * (34683 + 80634 * t - 102817 * pow(t, 2)) / 2500 +
                   -ind(t, 1, 2) * pow(2-t, 10) * (34683 + 80634 * (2-t) - 102817 * pow(2-t, 2)) / 2500;*/
/*
            return GravityComp().PreGrav(t)  7.2903*(-ind(t, 0, 1) * -33*pow(t,9)* (18694*pow(t,2) - 13439*t - 5255)/1250 - ind(t, 1, 2) * 33*pow(2-t,9)*(18694*pow(t,2) - 61337*t + 42643)/1250);
*/
            double ans = 14.40105*(-ind(t, 0, 1) * -33*pow(t,9)* (18694*pow(t,2) - 13439*t - 5255)/1250 - ind(t, 1, 2)*33*pow(2-t,9)* (18694*pow(2-t,2) - 13439*(2-t) - 5255)/1250) ;
            //drake::log()->info(ans);
            return ans;
/*GravityComp().PreGrav(t)*/

/*-1.8096;*/
        } else {
            return 0.0;
        }
    }

    double PrePlace(double t) {
        if (t < 2) {
            /*return ind(t, 0, 1) * pow(t, 10) * (34683 + 80634 * t - 102817 * pow(t, 2)) / 2500 +
                   ind(t, 1, 2) * M_PI / 4 -
                   ind(t, 2, 3) * pow(t, 10) * (34683 + 80634 * t - 102817 * pow(t, 2)) / 2500;*/
/*
            return 14.40105*(ind(t, 0, 1) * -33*pow(t,9)* (18694*pow(t,2) - 13439*t - 5255)/1250 +  ind(t, 1, 1.570796) * 0 - ind(t, 1.570796, 2.570796) * 33*pow(2.570796-t,9)* (18694*pow(2.570796-t,2) - 13439*(2.570796-t) - 5255)/1250);
*/
            double ans = -2*14.40105*(-ind(t, 0, 1) * -33*pow(t,9)* (18694*pow(t,2) - 13439*t - 5255)/1250 - ind(t, 1, 2)*33*pow(2-t,9)* (18694*pow(2-t,2) - 13439*(2-t) - 5255)/1250) ;
            //drake::log()->info(ans);

    return ans;
        } else
            return 0;
    }

    double Ret(double t) {
        if (t < 2) {
            /*return -ind(t, 0, 1) * (pow(t, 10) * (34683 + 80634 * t - 102817 * pow(t, 2))) / 2500 +
                   ind(t, 1, 2) * (pow(t, 10) * (34683 + 80634 * t - 102817 * pow(t, 2))) / 2500;*/
            return
/*GravityComp().PreGrav(t)*/
                    14.40105*(-ind(t, 0, 1) * -33*pow(t,9)* (18694*pow(t,2) - 13439*t - 5255)/1250 - ind(t, 1, 2)*33*pow(2-t,9)* (18694*pow(2-t,2) - 13439*(2-t) - 5255)/1250) ;

        } else
            return 0;
    }
};




/*class OperationSystem : public drake::systems::LeafSystem<double>
{
    void DoCalcOutput(const Context<double>&, Eigen::VectorBlock<VectorX<double>>*) const;
};*/

Operation Operation;
