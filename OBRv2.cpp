
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <tf/tf.h>
#include <iostream>
#include <math.h>
#include "std_msgs/Float32.h"
#include "FLIE-master/flie.h"

using namespace std;

float sen1, sen2, sen3, sen4, senGreenLeft, senGreenRigth, senIR1, senIR2, senIR3, senIR4, senIR5;
int Erro = 0;
int Erro_Ant = 0;
int TestGreen;
int senLinha [4];

void sub0Callback(const std_msgs::Float32::ConstPtr& Sen1){
  sen1 = Sen1->data;
}
void sub1Callback(const std_msgs::Float32::ConstPtr& Sen2){
  sen2 = Sen2->data;
}
void sub2Callback(const std_msgs::Float32::ConstPtr& Sen3){
  sen3 = Sen3->data;
}
void sub3Callback(const std_msgs::Float32::ConstPtr& Sen4){
  sen4 = Sen4->data;
}
void sub4Callback(const std_msgs::Float32::ConstPtr& SenGreenLeft){
  senGreenLeft = SenGreenLeft->data;
}
void sub5Callback(const std_msgs::Float32::ConstPtr& SenGreenRigth){
  senGreenRigth = SenGreenRigth->data;
}
void sub6Callback(const std_msgs::Float32::ConstPtr& SDist1){
  senIR1 = SDist1->data;
}
void sub7Callback(const std_msgs::Float32::ConstPtr& SDist2){
  senIR2 = SDist2->data;
}
void sub8Callback(const std_msgs::Float32::ConstPtr& SDist3){
  senIR3 = SDist3->data;  
}
void sub9Callback(const std_msgs::Float32::ConstPtr& SDist4){
  senIR4 = SDist4->data;  
}
void sub10Callback(const std_msgs::Float32::ConstPtr& SDist5){
  senIR5 = SDist5->data;  
}
  
int main(int argc, char **argv){
  ros::init(argc, argv, "project_OBR");

  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	ros::Subscriber sub0 = n.subscribe("/Slinha1", 1000, sub0Callback);
	ros::Subscriber sub1 = n.subscribe("/Slinha2", 1000, sub1Callback);
	ros::Subscriber sub2 = n.subscribe("/Slinha3", 1000, sub2Callback);
	ros::Subscriber sub3 = n.subscribe("/Slinha4", 1000, sub3Callback);
	ros::Subscriber sub4 = n.subscribe("/SGreenLeft", 1000, sub4Callback);
	ros::Subscriber sub5 = n.subscribe("/SGreenRigth", 1000, sub5Callback);
	ros::Subscriber sub6 = n.subscribe("/IREsqX", 1000, sub6Callback);
	ros::Subscriber sub7 = n.subscribe("/IREsq", 1000, sub7Callback);
	ros::Subscriber sub8 = n.subscribe("/IRMeio", 1000, sub8Callback);
	ros::Subscriber sub9 = n.subscribe("/IRDir", 1000, sub9Callback);
	ros::Subscriber sub10 = n.subscribe("/IRDirX", 1000, sub10Callback);
	
  ros::Rate loop_rate(10);
	ros::Time last_time, actual_time;
	
//================= Sistema Fuzzy composto por 3 entradas (Dist) e uma saída (vel. angular) para desvio obstáculo =================
//####################################################################################################################################
	
//Deve-se definir um sistema de controle que ira conter as regras.
fuzzy_control fc_obs;
fuzzy_set cat[14];
linguisticvariable Left_Sensor, Midle_Sensor, Rigth_Sensor, Vel_Ang;
rule infrule[27];

//Define-se os conjuntos fuzzy para a variavel linguistica Left_Sensor
cat[0].setname("PT"); //PT - Perto
cat[0].setrange(0,+0.4);
cat[0].setval(0, 0, 0.1, 0.2);

cat[1].setname("MD"); //PT - Médio
cat[1].setrange(0,+0.4);
cat[1].setval(0.1, 0.2, 0.3);

cat[2].setname("LG"); //LG - Longe
cat[2].setrange(0,+0.4);
cat[2].setval(0.2, 0.3, 0.4, 0.4);

//Define-se a Variavel linguistica Left_Sensor
Left_Sensor.setname("Left_Sensor");
Left_Sensor.includecategory(&cat[0]);
Left_Sensor.includecategory(&cat[1]);
Left_Sensor.includecategory(&cat[2]);

//Define-se os conjuntos fuzzy para a variavel linguistica Midle_Sensor
cat[3].setname("PT"); //PT - Perto
cat[3].setrange(0,+0.4);
cat[3].setval(0, 0, 0.1, 0.2);

cat[4].setname("MD"); //PT - Médio
cat[4].setrange(0,+0.4);
cat[4].setval(0.1, 0.2, 0.3);

cat[5].setname("LG"); //LG - Longe
cat[5].setrange(0,+0.4);
cat[5].setval(0.2, 0.3, 0.4, 0.4);

//Define-se a Variavel linguistica Midle_Sensor
Midle_Sensor.setname("Midle_Sensor");
Midle_Sensor.includecategory(&cat[3]);
Midle_Sensor.includecategory(&cat[4]);
Midle_Sensor.includecategory(&cat[5]);

//Define-se os conjuntos fuzzy para a variavel linguistica Rigth_Sensor
cat[6].setname("PT"); //PT - Perto
cat[6].setrange(0,+0.4);
cat[6].setval(0, 0, 0.1, 0.2);

cat[7].setname("MD"); //PT - Médio
cat[7].setrange(0,+0.4);
cat[7].setval(0.1, 0.2, 0.3);

cat[8].setname("LG"); //LG - Longe
cat[8].setrange(0,+0.4);
cat[8].setval(0.2, 0.3, 0.4, 0.4);

//Define-se a Variavel linguistica Rigth_Sensor
Rigth_Sensor.setname("Rigth_Sensor");
Rigth_Sensor.includecategory(&cat[6]);
Rigth_Sensor.includecategory(&cat[7]);
Rigth_Sensor.includecategory(&cat[8]);

//Define-se os conjuntos fuzzy para a variavel linguistica Vel_Ang
cat[9].setname("NR"); // NR - Negativa Rápida
cat[9].setrange(-1,+1);
cat[9].setval(-1, -1, -0.5, -0.2);

cat[10].setname("ND"); // ND - Negativa Devagar
cat[10].setrange(-1,+1);
cat[10].setval(-0.5, -0.2, 0);

cat[11].setname("QP"); // QP - Quaze Parado
cat[11].setrange(-1,+1);
cat[11].setval(-0.05, 0, 0.05);

cat[12].setname("PD"); // PD - Positiva Devagar
cat[12].setrange(-1,+1);
cat[12].setval(0, 0.2, 0.5);

cat[13].setname("PR"); // PR - Positiva Rápida
cat[13].setrange(-1,+1);
cat[13].setval(0.2, 0.5, 1, 1);

//Define-se a variavel linguistica Vel_Ang
Vel_Ang.setname("Vel_Ang");
Vel_Ang.includecategory(&cat[9]);
Vel_Ang.includecategory(&cat[10]);
Vel_Ang.includecategory(&cat[11]);
Vel_Ang.includecategory(&cat[12]);
Vel_Ang.includecategory(&cat[13]);

//Define-se o metodo defuzzificacao: MAXIMUM, AVERAGEOFMAX, or CENTEROFAREA/CENTROID
fc_obs.set_defuzz(CENTROID);

//Define-se o fuzzy_control pela entrada fuzzy( Erro) e saida (Vel_Ang) )
fc_obs.definevars(Left_Sensor, Midle_Sensor, Rigth_Sensor, Vel_Ang);

//Deve-se incluir cada regra fuzzy no fuzzy_control
fc_obs.insert_rule("PT","PT","PT","PR"); // If Left_Sensor is Perto and Midle_Sensor is Perto and... then Vel_Ang is Positiva Rápida
fc_obs.insert_rule("PT","PT","MD","PR");
fc_obs.insert_rule("PT","PT","LG","PR");
fc_obs.insert_rule("PT","MD","PT","PR");
fc_obs.insert_rule("PT","MD","MD","PD");
fc_obs.insert_rule("PT","MD","LG","PR");
fc_obs.insert_rule("PT","LG","PT","QP");
fc_obs.insert_rule("PT","LG","MD","PD");
fc_obs.insert_rule("PT","LG","LG","PR");

fc_obs.insert_rule("MD","PT","PT","ND");
fc_obs.insert_rule("MD","PT","MD","PR");
fc_obs.insert_rule("MD","PT","LG","PR");
fc_obs.insert_rule("MD","MD","PT","ND");
fc_obs.insert_rule("MD","MD","MD","PD");
fc_obs.insert_rule("MD","MD","LG","PD");
fc_obs.insert_rule("MD","LG","PT","ND");
fc_obs.insert_rule("MD","LG","MD","QP");
fc_obs.insert_rule("MD","LG","LG","PD");

fc_obs.insert_rule("LG","PT","PT","NR");
fc_obs.insert_rule("LG","PT","MD","PR");
fc_obs.insert_rule("LG","PT","LG","PR");
fc_obs.insert_rule("LG","MD","PT","NR");
fc_obs.insert_rule("LG","MD","MD","ND");
fc_obs.insert_rule("LG","MD","LG","PD");
fc_obs.insert_rule("LG","LG","PT","ND");
fc_obs.insert_rule("LG","LG","MD","QP");
fc_obs.insert_rule("LG","LG","LG","QP");

	ROS_WARN("!!! Simulacao Rodando !!!");
	
	while (ros::ok()){
  
	geometry_msgs::Twist msg;
	
 // Testa os sensores de linha para as possíveis condições de curva
  if ((senLinha[0] == 0) && (senLinha[1] == 0) && (senLinha[2] == 0) && (senLinha[3] == 0)){// --|--
  	TestGreen = 0;}
  if ((senLinha[0] == 0) && (senLinha[1] == 0) && (senLinha[2] == 0) && (senLinha[3] == 1)){// --|
  	TestGreen = 1;}
  if ((senLinha[0] == 1) && (senLinha[1] == 0) && (senLinha[2] == 0) && (senLinha[3] == 0)){// |--
  	TestGreen = -1;}
  	
 //=================================== Virar 180 graus ==============================================
 //##################################################################################################
 
  if ((senGreenLeft == 1) && (senGreenRigth == 1) && (TestGreen == 0)){ 
    int x = 0;
     
    while (x == 0){          
     msg.angular.z = 0.3;
     msg.linear.x = 0;
     pub.publish(msg);
  	 loop_rate.sleep(); 
  	 ros::spinOnce();
    
    senLinha[0] = sen1; senLinha[1] = sen2; senLinha[2] = sen3; senLinha[3] = sen4;  
  for (int i = 0; i <4; i++){
  	if (senLinha[i] < 1){
  		senLinha[i] = 0;
  	}
  	else{
  		senLinha[i] = 1;
  	}
  } 
  
  	if ((senLinha[0] == 1) && (senLinha[1] == 0) && (senLinha[2] == 0) && (senLinha[3] == 1)){
  		x = 1;
  			}   	 
    	}
    }
  	
 //=================================== Virar 90 graus à direita =====================================
 //##################################################################################################
 
  	if ((senGreenLeft == 0) && (senGreenRigth == 1) && (TestGreen == -1)){
    int x = 0;
     
    while (x == 0){          
     msg.angular.z = 0.3;
     msg.linear.x = 0.04;
     pub.publish(msg);
  	 loop_rate.sleep(); 
  	 ros::spinOnce();
    
    senLinha[0] = sen1; senLinha[1] = sen2; senLinha[2] = sen3; senLinha[3] = sen4;  
  for (int i = 0; i <4; i++){
  	if (senLinha[i] < 1){
  		senLinha[i] = 0;
  	}
  	else{
  		senLinha[i] = 1;
  	}
  }  
  
  	if ((senLinha[0] == 1) && (senLinha[1] == 0) && (senLinha[2] == 0) && (senLinha[3] == 1)){ // Se Erro = 0
  		x = 1;
  			}   	 
    	}
    }
    
  // ==================================== Segue linha ===========================================
  // ############################################################################################
    
  senLinha[0] = sen1; senLinha[1] = sen2; senLinha[2] = sen3; senLinha[3] = sen4;  
  for (int i = 0; i <4; i++){ // Teste de linha preta ou pista branca
  	if (senLinha[i] < 1){
  		senLinha[i] = 0;
  	}
  	else{
  		senLinha[i] = 1;
  	}
  }  
  
  // Atribui um valor de Erro a partir da posição dos sensores com relação à linha.
  if((senLinha[0] == 0) && (senLinha[1] == 1) && (senLinha[2] == 1) && (senLinha[3] == 1)){
  	Erro = -3;}
	else if((senLinha[0] == 0) && (senLinha[1] == 0) && (senLinha[2] == 1) && (senLinha[3] == 1)){
  	Erro = -2;}
 	else if((senLinha[0] == 1) && (senLinha[1] == 0) && (senLinha[2] == 1) && (senLinha[3] == 1)){
  	Erro = -1;}  
  else if((senLinha[0] == 1) && (senLinha[1] == 0) && (senLinha[2] == 0) && (senLinha[3] == 1)){
  	Erro = 0;}
  else if((senLinha[0] == 1) && (senLinha[1] == 1) && (senLinha[2] == 0) && (senLinha[3] == 1)){
  	Erro = 1;}
  else if((senLinha[0] == 1) && (senLinha[1] == 1) && (senLinha[2] == 0) && (senLinha[3] == 0)){
  	Erro = 2;}
  else if((senLinha[0] == 1) && (senLinha[1] == 1) && (senLinha[2] == 1) && (senLinha[3] == 0)){
  	Erro = 3;}  	
  	else{
  	Erro = Erro_Ant;
  	}
  	
  	Erro_Ant = Erro;
  	
  	if ((senLinha[0] == 1 & senLinha[1] == 1 & senLinha[2] == 1 & senLinha[3] == 1) & (Erro == 1 || Erro == -1)){
    	Erro_Ant = 0;
   	}    
    
  // Cálculo PID para segue linha
  	actual_time = ros::Time::now();
    double dt = (actual_time - last_time).toSec();
    last_time = actual_time;    
    
    float P = Erro;
    float D = (Erro - Erro_Ant)/dt;
    float Kp = 0.15;
    float Kd = 0.05;
    
  if (senIR1 == 0){
        senIR1 = 0.3;}        
	if (senIR2 == 0){
        senIR2 = 0.3;}        
	if (senIR3 == 0){
        senIR3 = 0.3;}        
	if (senIR4 == 0){
        senIR4 = 0.3;}        
	if (senIR5 == 0){
        senIR5 = 0.3;}
        
    float SIR1 = min(senIR1, senIR2);
    float SIR2 = senIR3;
    float SIR3 = min(senIR4, senIR5);
    
    if ((SIR1 < 0.25) || (SIR2 < 0.25) || (SIR3 < 0.25)){

    msg.angular.z = fc_obs.make_inference(SIR1, SIR2, SIR3);
    //msg.angular.z = 1.2;
    msg.linear.x = 0.04;
    }
    else{
    
    msg.angular.z = Kp * P + Kd * D;
    msg.linear.x = 0.04;   
		}
		
    pub.publish(msg);
    ros::spinOnce();
    //ROS_WARN("Erro>>%i\n", Erro);
  }
  return 0;
  
  ROS_WARN("!!! Simulação Parada !!!");
}
