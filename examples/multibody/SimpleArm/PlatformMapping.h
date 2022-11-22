//
// Created by arjunbadyal on 24/04/2022.
//
#pragma once



#ifdef __cplusplus
extern "C" {
#endif


double ind(double t, double a, double b);

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
double Operation(char cmd[], double t);

double PrePick(double t);

double PrePlace(double t);

double Ret(double t);


/*class OperationSystem : public drake::systems::LeafSystem<double>
{
    void DoCalcOutput(const Context<double>&, Eigen::VectorBlock<VectorX<double>>*) const;
};*/

//Operation Operation;
#ifdef __cplusplus
}
#endif