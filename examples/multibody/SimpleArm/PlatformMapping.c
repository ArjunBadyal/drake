//
// Created by arjunbadyal on 11/04/2022.
//
#include <math.h>
#include "PlatformMapping.h"
#include <string.h>
#include <stdio.h>

double PrePicked = 1;


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
t = t-2;
    /*return ind(t, 0, 1) * pow(t, 10) * (34683 + 80634 * t - 102817 * pow(t, 2)) / 2500 +
           ind(t, 1, 2) * M_PI / 4 -
           ind(t, 2, 3) * pow(t, 10) * (34683 + 80634 * t - 102817 * pow(t, 2)) / 2500;*/
    /*
                return 14.40105*(ind(t, 0, 1) * -33*pow(t,9)* (18694*pow(t,2) - 13439*t - 5255)/1250 +  ind(t, 1, 1.570796) * 0 - ind(t, 1.570796, 2.570796) * 33*pow(2.570796-t,9)* (18694*pow(2.570796-t,2) - 13439*(2.570796-t) - 5255)/1250);
    */
    double ans = -2*14.40105*(-ind(t, 0, 1) * -33*pow(t,9)* (18694*pow(t,2) - 13439*t - 5255)/1250 - ind(t, 1, 2)*33*pow(2-t,9)* (18694*pow(2-t,2) - 13439*(2-t) - 5255)/1250) ;
    //drake::log()->info(ans);
    printf("preplacing");
    return ans;
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

double Operation(double Objectdistance, double Goaldistance,  double t) {
  char * cmd = dmodel(Event1(Objectdistance), Event2(Goaldistance), t);
  //char * cmd = dmodel(entry);
  if (strcmp(cmd, "PrePick") == 0) {
    //printf("PrePicking");
    return PrePick(t);


  }

  else if (strcmp(cmd, "PrePlace") == 0) {
    //printf("PrePlacing");
    return PrePlace(t);

  }

  else if (strcmp(cmd, "Return") == 0) {
    return Ret(t);
  }

  else {
    //printf("Invalid input, halting actuation.");
    return 0;
  }
}

  char * Event1(double Objectdistance){

    if (Objectdistance < 4.75111 && Objectdistance > 4.749)
    {
      return "detectObject";
    }
    else{
      return "!detectObject";
    }


  }

  char * Event2(double Goaldistance){

    if (Goaldistance < 4.75111 && Goaldistance > 4.749)
    {
      return "detectGoal";
    }
    else{
      return "!detectGoal";
    }
  }

  char * dmodel(char * event1 ,char * event2, double t){
    /*printf("%s", event1);
    printf("%f\\n",t);
    printf("%s\\n",event2);*/
    if (t <2 &&  strcmp(event1,"detectObject") == 0){

      return "PrePick";
      //printf("Too far away :(");

    }
    else if (t>=2 && t <3 && strcmp(event2,"detectGoal") == 0)
    {
      return "PrePlace";
      printf("PrePlacing :)");
    }

    else if (t>=3 && t <5)
    {
      return "Return";
      printf("Returning:)");
    }


    else{
      return "Invalid Input";
    }

  }



