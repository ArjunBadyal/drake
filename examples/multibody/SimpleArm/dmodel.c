//
// Created by arjunbadyal on 18/04/2022.
//

char * dmodel(char * event){
  if (event == "detectObject") {
    return "PrePick";
  }
  else if (event =="detectGoal") {
    return "PrePlace";
  }
  else{
    return "Invalid Input";
  }
}


int main(){

    return 0;
}