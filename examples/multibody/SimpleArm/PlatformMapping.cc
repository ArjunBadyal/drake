//
// Created by arjunbadyal on 11/04/2022.
//
#include <cmath>


using namespace std;


    struct Operation {

        double ind(double t, double a, double b) {
            if (t >= a && t <= b) {
                return 1;
            } else {
                return 0;
            }
        }

        double PrePick(double t) {
            if (t < 5) {
                return -ind(t, 0, 1) * pow(t, 10) * (34683 + 80634 * t - 102817 * pow(t, 2)) / 2500 +
                       ind(t, 1, 2) * pow(t, 10) * (34683 + 80634 * t - 102817 * pow(t, 2)) / 2500;
            } else {
                return 0;
            }
        }

        int PrePlace(double t) {
            if (t < 5) {
                return ind(t, 0, 1) * pow(t, 10) * (34683 + 80634 * t - 102817 * pow(t, 2)) / 2500 +
                       ind(t, 1, 2) * M_PI / 4 -
                       ind(t, 2, 3) * pow(t, 10) * (34683 + 80634 * t - 102817 * pow(t, 2)) / 2500;
            } else
                return 0;
        }

        double Ret(double t) {
            if (t < 5) {
                return -ind(t, 0, 1) * (pow(t, 10) * (34683 + 80634 * t - 102817 * pow(t, 2))) / 2500 +
                       ind(t, 1, 2) * (pow(t, 10) * (34683 + 80634 * t - 102817 * pow(t, 2))) / 2500;
            } else
                return 0;
        }
    };


/* bool detectObject(double ObjectIRvoltage)
{
    if (ObjectIRvoltage >= 3.46382) {
        return true;
    }
    else {
        return false;
    }
}

bool detectGoal(double GoalIRvoltage)
{
    cin >> prompt;
    if prompt == "T"
    if (GoalIRvoltage >= 3.46382) {
        return true;
    }
    else {
        return false;
    }
}
*/

