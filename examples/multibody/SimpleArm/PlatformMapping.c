//
// Created by arjunbadyal on 11/04/2022.
//
#include <math.h>
#include "PlatformMapping.h"
#include <string.h>
#include <stdio.h>
#include <string.h>

double ind(double t, double a, double b) {
  if (t >= a && t <= b) {
    return 1;
  } else {
    return 0;
  }
}

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

double Operation(char cmd[], double t){
  if (strcmp(cmd, "PrePick") == 1)
  {
    return PrePick(t);
  }

  else if (strcmp(cmd, "PrePlace") == 1)
  {
    return PrePlace(t);
  }

  else if (strcmp(cmd, "Return") == 1)
  {
    return Ret(t);
  }

  else
  {
    printf("Invalid input, halting simulation.");
    return 0;
  }
}

