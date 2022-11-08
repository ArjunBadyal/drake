//
// Created by arjunbadyal on 18/04/2022.
//
#include <iostream>
#include <cmath>
#include <string>
#include "drake/examples/multibody/SimpleArm/PlatformMapping.h"
#include "drake/systems/framework/context.h"


using namespace std;

int main(){
    string input;
    bool LoopCond = true;

    while(LoopCond){

        cout << "Enter Operation:" << endl;
        cin >> input;
        //input = std::cin.get();
        if (input == "PrePick") {
            cout << "PrePicking" << endl;
        }
        else {
            cout << "Invalid Input" << endl;
            LoopCond = false;
        }

    }
    return 0;
}