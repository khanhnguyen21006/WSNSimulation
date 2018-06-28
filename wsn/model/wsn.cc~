/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "wsn.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/aodv-helper.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/ipv4-list-routing-helper.h"

namespace ns3 {

/* ... */

// static void ReceivePacket (Ptr<Socket> socket)
//    {
//       Ptr<Packet> packet = socket->Recv ();
     
     
//     // uint32_t D;
      
//       //  std::string os;               
//     uint8_t *buffer = new uint8_t[packet->GetSize ()];
//     packet->CopyData(buffer, packet->GetSize ());
//     std::string s = std::string((char*)buffer);
//     int data = atoi(s.c_str());
//     if(data<1000000)data=data*10; //value = 45
//     }	

int posX(Ptr<Node> node)
{
  Ptr<MobilityModel> mob = node->GetObject<MobilityModel>();
  Vector pos = mob->GetPosition ();
  return pos.x;

}
 int posY(Ptr<Node> node)
{
  Ptr<MobilityModel> mob = node->GetObject<MobilityModel>();
  Vector pos = mob->GetPosition ();
  return pos.y;

}

  Sensor::Sensor(void){

	this->Create(25);
 
	temp = new double[400]; 
	hump = new double[400]; 
	


}


 void Sensor::setCluster(Ptr<Node> cluster)
{
  Cluster=cluster;
}

Ptr<Node> Sensor::getCluster(void)
{
  return Cluster;
}
Sensor::~Sensor(){
	delete[] temp;
	delete[] hump;
	
}
Cluster::Cluster(int numCluster,int numNode){
 this->Create(numCluster);
data=new std::ostringstream[numCluster];
}
Cluster::~Cluster(){

}


UAV::UAV(int numCluster,int numNode){
numClus=numCluster;
  numNodes=numNode;

// temp = new double *[numClus]; 
//   for(int i=0;i<numClus;i++)
//   {
//     temp[i]=new double[numNodes];
// }
}

UAV::~UAV(){

 }

 Temp::Temp(std::map<int,double> m)
 {
  map=m;
 }
std::map<int,double> Temp::getData(void)
{
  return map;
}

}
